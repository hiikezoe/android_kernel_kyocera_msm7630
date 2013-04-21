/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h> 
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/android_alarm.h>
#include <linux/wakelock.h>

#define   LED_DEBUG 0

#if LED_DEBUG
#define DEBUG_PRINT( arg... )   printk( KERN_INFO "[KCLIGHT][DRV]:" arg )
#else
#define DEBUG_PRINT( arg... )
#endif

#define DEBUG_PRINT_ERR( arg... )   printk( KERN_ERR "[KCLIGHT][DRV]:" arg )

#define  LED_INDIVIDUAL_ON

enum light_index_enum{
    MOBILELIGHT_INDEX = 0,
    LEDLIGHT_INDEX,
    LIGHT_INDEX_MAX
};

#define LED_DRV_NAME           "KCledlight"

#define LED_SLAVE_ADDR_W    0xEA

#define I2C_BUS_NUMBER         4
#define I2C_RETRIES_NUM        5

#define LED_WRITE_BUF_LEN      2

#define MOBILELIGHT_INFO      "mobilelightinfo"
#define LED_INFO              "ledinfo"

#define MOBILELIGHT_MAX_BRIGHT_VAL          0xFFFFFFFFu
#define LEDLIGHT_MAX_BRIGHT_VAL             0xFFFFFFFFu

#define REG_ADDR_00         0x00
#define REG_ADDR_01         0x01
#define REG_ADDR_02         0x02
#define REG_ADDR_03         0x03
#define REG_ADDR_04         0x04

#define REG_ADDR_00_INIT    0x40
#define REG_ADDR_01_INIT    0x00
#define REG_ADDR_02_INIT    0x00
#define REG_ADDR_03_INIT    0x00
#define REG_ADDR_04_INIT    0x00

#if 1 
#define VLED_ON_VAL            0xC0
#define VLED_OFF_VAL           0x40
#else
#define VLED_ON_VAL            0x80
#define VLED_OFF_VAL           0x00
#endif

#define INDLED_OFF_VAL          0x00
#define INDLED_RED_ON_VAL       0xC8
#define INDLED_GREEN_ON_VAL     0xC4
#define INDLED_BLUE_ON_VAL      0xC2

#define LEDLIGHT_COL_CHECK      0xFFFFFFFFu
#define LEDLIGHT_COL_BLACK      0x000000

#define LEDLIGHT_ON_VAL         0xCE
#define LEDLIGHT_OFF_VAL        0xC0

#define LEDLIGHT_CURVAL_MAX     0x04
#define LEDLIGHT_CURVAL_RED_MAX     0x00
#define LEDLIGHT_CURVAL_GREEN_MAX   0x00
#define LEDLIGHT_CURVAL_BLUE_MAX    0x00

#define MOBILELIGHT_OFF_MODE                0x00
#define MOBILELIGHT_BATTTERM_LOW_MODE       0x01
#define MOBILELIGHT_BATTTERM_HIGH_MODE      0x02
#define MOBILELIGHT_VIEWING_MODE            0x03

#define MOBILELIGHT_BATTTERM_LOW_VAL        0x15
#define MOBILELIGHT_BATTTERM_HIGH_VAL       0x0A
#define MOBILELIGHT_VIEWING_VAL             0x00
#define MOBILELIGHT_ON_VAL                  0x80
#define MOBILELIGHT_OFF_VAL                 0x00

#define FLG_ON              0x01
#define FLG_OFF             0x00

#define VLED_NO_TOUCH  0x10
#define LEDLIGHT_BLINK_NUM  (4)
#define LEDLIGHT    'L'
#define LEDLIGHT_SET_BLINK      _IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define MIN_BLINK_ALARM_SET_DELAY  100

enum color_pattern_enum{
    COLOR_PATTERN_1 = 0,
    COLOR_PATTERN_2,
    COLOR_PATTERN_3,
    COLOR_PATTERN_4,
    COLOR_PATTERN_5,
    COLOR_PATTERN_6,
    COLOR_PATTERN_7,
    COLOR_PATTERN_MAX
};
static uint8_t    guc_led_on_flg = FLG_OFF;
static uint8_t    guc_mobilelight_on_flg = FLG_OFF;
static uint8_t  guc_reg0_val = FLG_OFF;

enum blink_control_enum{
    NO_BLINK_REQUEST = 0,
    BLINK_REQUEST_ON,
    BLINK_REQUEST_OFF
};

struct light_led_data_type {
    struct led_classdev     st_cdev;
    struct i2c_client       *pst_client ;
    uint32_t                ul_id;
    uint32_t                ul_value;
    uint32_t                ul_flags;
    struct work_struct      work;
    struct mutex            lock;
    spinlock_t              value_lock;
    uint8_t                 uc_indled_val;

    struct alarm            led_alarm;  
    struct wake_lock        work_wake_lock;
    uint32_t                blink_control; 
    uint32_t                blink_on_delay;
    uint32_t                blink_off_delay;
    uint32_t                blink_off_color;
    ktime_t                 last_elapsed_time;
    uint32_t                saved_color_value;
};

static struct light_led_data_type *gpst_light_led_data = NULL;

static uint32_t const gul_color_pattern_value[COLOR_PATTERN_MAX] = {
    0x000000FF,
    0x00FF0000,
    0x0000FF00,
    0x00FF00FF,
    0x0000FFFF,
    0x00FFFF00,
    0x00C0C0C0
};

typedef struct _t_ledlight_ioctl
{
    uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;

#if 0
static void   debug_msg_out( struct light_led_data_type *pst_light_led_data ) ;
#endif

static void led_alarm(struct alarm *alarm)
{
    struct light_led_data_type *pst_light_led_data =
        container_of(alarm, struct light_led_data_type, led_alarm);
    DEBUG_PRINT("LED DEBUG:led_alarm Call\n");
    wake_lock(&pst_light_led_data->work_wake_lock);
    schedule_work(&pst_light_led_data->work);
}

static void led_set_alarm(struct light_led_data_type *pst_light_led_data, int32_t msec)
{
    
    int32_t interval_sec;
    int32_t interval_nsec; 
    ktime_t interval ;
    ktime_t next;
    
    if(msec < MIN_BLINK_ALARM_SET_DELAY)
    {
        msec = MIN_BLINK_ALARM_SET_DELAY;
    }
    interval_sec = msec/MSEC_PER_SEC;
    interval_nsec= (msec%MSEC_PER_SEC) * NSEC_PER_MSEC; 
    interval = ktime_set(interval_sec, interval_nsec);
    
    pst_light_led_data->last_elapsed_time = alarm_get_elapsed_realtime();

    next = ktime_add(pst_light_led_data->last_elapsed_time, interval);

    DEBUG_PRINT("LED DEBUG:led_set_alarm Call :elapsed_realtime=%d.%d : interval=%d.%d : next=%d.%d \n",
        pst_light_led_data->last_elapsed_time.tv.sec,pst_light_led_data->last_elapsed_time.tv.nsec,
        interval.tv.sec,interval.tv.nsec,
        next.tv.sec,next.tv.nsec);
    alarm_start_range(&pst_light_led_data->led_alarm, next, next);
}

static int32_t ledlight_write (struct i2c_client *pst_client, uint8_t uc_reg, uint8_t uc_val)
{
    int32_t lret = 0;
    uint32_t ul_tryn = 0 ;
    struct light_led_data_type *pst_light_led_data = i2c_get_clientdata(pst_client);
    u8 ucwritebuf[LED_WRITE_BUF_LEN] ;
    uint8_t uc_len;
    struct i2c_msg msg ;

    DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x\n",
            __func__,
            uc_reg,
            uc_val) ;

    DEBUG_PRINT("%s() start pst_client=0x%08x pst_light_led_data=0x%08x\n",
            __func__, 
            (unsigned int)pst_client ,
            (unsigned int)pst_light_led_data);


    DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x, guc_reg0_val=0x%02x\n",
            __func__,
            uc_reg,
            uc_val,
        guc_reg0_val);
    if( REG_ADDR_00 == uc_reg )
    {
        if( guc_reg0_val == uc_val )
        {
            goto noupdate ;
        }
        else
        {
            guc_reg0_val = uc_val ;
        }
    }

    ucwritebuf[0] = uc_reg ;
    ucwritebuf[1] = uc_val ;
    uc_len = sizeof(ucwritebuf) ;

    msg.addr = pst_light_led_data->pst_client->addr;
    msg.flags = 0;
    msg.len = uc_len;
    msg.buf = &ucwritebuf[0];

    DEBUG_PRINT("%s() start uc_reg=0x%02x uc_val=0x%02x, guc_reg0_val=0x%02x\n",
            __func__,
            uc_reg,
            uc_val,
            guc_reg0_val);

    do{
        DEBUG_PRINT("%s() i2c_transfer() call\n", __func__);
        lret = i2c_transfer(pst_light_led_data->pst_client->adapter, &msg, 1) ;
        DEBUG_PRINT("%s() i2c_transfer() call end lret=%d\n", __func__ , lret );
    }while ((lret != 1) && (++ul_tryn < I2C_RETRIES_NUM));

    if(lret != 1){
        DEBUG_PRINT_ERR("%s(): uc_reg 0x%02x, uc_val 0x%02x lret %d\n",
                __func__, 
                ucwritebuf[0],
                ucwritebuf[1],
                lret);
        lret = -1 ;
    }else{
        lret = 0 ;
    }
    DEBUG_PRINT("%s() end\n", __func__);

    return lret;

noupdate:
    return lret;
}

static int32_t led_color_pattern_check(uint32_t ul_colorval)
{
    int32_t lret = 0 ;
    int32_t lmatch_pattern = 0;

    DEBUG_PRINT("%s() start \n", __func__ );

    for ( lmatch_pattern=0; lmatch_pattern<COLOR_PATTERN_MAX; lmatch_pattern++ )
    {
        if ( ul_colorval == gul_color_pattern_value[lmatch_pattern] )
        {
            DEBUG_PRINT("%s() color pattern match %d \n", __func__, lmatch_pattern);
            lret = 1;
            break;
        }
    }

    DEBUG_PRINT("%s() end\n", __func__ );

    return lret ;
}
static void led_set(struct light_led_data_type *pst_light_led_data)
{
    int32_t lret = 0;
    uint32_t ul_colorval ;
    uint8_t uc_color_red, uc_color_green, uc_color_blue ;
    uint8_t uc_color_red_val, uc_color_green_val, uc_color_blue_val ;
    uint8_t uc_vledctrl ;
    DEBUG_PRINT("%s() start \n", __func__ );

    pst_light_led_data->uc_indled_val = INDLED_OFF_VAL ;

    if( LEDLIGHT_COL_BLACK == (pst_light_led_data->ul_value&0x00FFFFFF)){
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00, LEDLIGHT_OFF_VAL);
        if (lret) {
            DEBUG_PRINT_ERR("%s()can't set 3LED light data ul_value LEDLIGHT_OFF_VAL (0x%02x) \n",
                    __func__,
                    LEDLIGHT_OFF_VAL);
            return;
        }
        uc_vledctrl = ( pst_light_led_data->ul_value&0xFF000000 )>>24 ;
        if(( FLG_OFF == guc_mobilelight_on_flg )&&
           ( VLED_NO_TOUCH != (uc_vledctrl&VLED_NO_TOUCH)))
        {
            DEBUG_PRINT("%s() guc_mobilelight_on_flg(%d) \n", __func__, guc_mobilelight_on_flg ) ;
            mdelay(1) ;
            DEBUG_PRINT("%s() mdelay(1) \n", __func__ ) ;
            lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , VLED_OFF_VAL);
            if (lret) {
                DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value VLED_OFF_VAL (0x%02x) \n",
                        __func__,
                        VLED_OFF_VAL);
                return;
            }
        }
        guc_led_on_flg = FLG_OFF;
        DEBUG_PRINT("%s() guc_led_on_flg(%d) \n", __func__, guc_led_on_flg ) ;
    }
    else {

        uc_vledctrl = ( pst_light_led_data->ul_value&0xFF000000 )>>24 ;
        if(( FLG_OFF == guc_mobilelight_on_flg ) &&
           ( VLED_NO_TOUCH != (uc_vledctrl&VLED_NO_TOUCH)))
        {
            lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , VLED_ON_VAL);
            if (lret) {
                DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value VLED_ON_VAL|guc_reg0_val (0x%02x) \n",
                        __func__,
                        VLED_ON_VAL);
                return;
            }
            mdelay(1) ;
            DEBUG_PRINT("%s() mdelay(1) \n", __func__ ) ;
        }
        ul_colorval = (pst_light_led_data->ul_value&0x00FFFFFF) ;

        uc_color_red = (u8)((ul_colorval>>(16+3)) & 0x1f) ;
        uc_color_green = (u8)((ul_colorval>>(8+3)) & 0x1f) ;
        uc_color_blue = (u8)((ul_colorval>>(3)) & 0x1f) ;
        if ( led_color_pattern_check(ul_colorval) )
        {
            if(INDLED_OFF_VAL != uc_color_red)
            {
                pst_light_led_data->uc_indled_val |= INDLED_RED_ON_VAL ;
                uc_color_red = LEDLIGHT_CURVAL_RED_MAX ;
            }
            if(INDLED_OFF_VAL != uc_color_green)
            {
                pst_light_led_data->uc_indled_val |= INDLED_GREEN_ON_VAL ;
                uc_color_green = LEDLIGHT_CURVAL_GREEN_MAX ;
            }
            if(INDLED_OFF_VAL != uc_color_blue)
            {
                pst_light_led_data->uc_indled_val |= INDLED_BLUE_ON_VAL ;
                uc_color_blue = LEDLIGHT_CURVAL_BLUE_MAX ;
            }
        }

        if( uc_color_red <= LEDLIGHT_CURVAL_MAX )
        {
            uc_color_red_val = uc_color_red ;
        }
        else
        {
            uc_color_red_val = LEDLIGHT_CURVAL_MAX ;
        }
        if( uc_color_green <= LEDLIGHT_CURVAL_MAX )
        {
            uc_color_green_val = uc_color_green ;
        }
        else
        {
            uc_color_green_val = LEDLIGHT_CURVAL_MAX ;
        }
        if( uc_color_blue <= LEDLIGHT_CURVAL_MAX )
        {
            uc_color_blue_val = uc_color_blue ;
        }
        else
        {
            uc_color_blue_val = LEDLIGHT_CURVAL_MAX ;
        }
        DEBUG_PRINT("%s() uc_color_red=0x%x uc_color_red_val=0x%02x\n",
                __func__,
                uc_color_red,
                uc_color_red_val);

        DEBUG_PRINT("%s() uc_color_green=0x%02x uc_color_green_val=0x%02x\n",
                __func__,
                uc_color_green,
                uc_color_green_val);

        DEBUG_PRINT("%s() uc_color_blue=0x%02x uc_color_blue_val=0x%02x\n",
                __func__,
                uc_color_blue,
                uc_color_blue_val);


#ifdef LED_INDIVIDUAL_ON
        if(INDLED_OFF_VAL != uc_color_red_val)
        {
            pst_light_led_data->uc_indled_val |= INDLED_RED_ON_VAL ;
        }
#endif
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_02, uc_color_red_val);
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_red_val (0x%02x) \n",
                    __func__,
                    uc_color_red);
            return;
        }

#ifdef LED_INDIVIDUAL_ON
        if( INDLED_OFF_VAL != uc_color_green_val)
        {
            pst_light_led_data->uc_indled_val |= INDLED_GREEN_ON_VAL ;
        }
#endif
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_03 , uc_color_green_val);
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_green_val (0x%02x) \n",
                    __func__,
                    uc_color_green);
            return;
        }

#ifdef LED_INDIVIDUAL_ON
        if( INDLED_OFF_VAL != uc_color_blue_val )
        {
            pst_light_led_data->uc_indled_val |= INDLED_BLUE_ON_VAL ;
        }
#endif
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_04, uc_color_blue_val);
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set 3LED light data ul_value uc_color_blue_val (0x%02x) \n",
                    __func__,
                    uc_color_blue);
            return;
        }

#ifdef LED_INDIVIDUAL_ON
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , pst_light_led_data->uc_indled_val);
#else
        ret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , LEDLIGHT_ON_VAL);
#endif
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value led_lighting_val (0x%02x) \n",
                    __func__,
                    pst_light_led_data->uc_indled_val);
            return;
        }
        guc_led_on_flg = FLG_ON;
    }
    DEBUG_PRINT("%s() end\n", __func__ );
}

static void mobilelight_set(struct light_led_data_type *pst_light_led_data)
{
    int32_t lret = 0 ;
    uint8_t uc_value ;
    int8_t uc_vledctrl ;

    DEBUG_PRINT("%s() start \n", __func__);

#if 0
    debug_msg_out(pst_light_led_data) ;
#endif

    if( MOBILELIGHT_OFF_MODE == (pst_light_led_data->ul_value&0x00FFFFFF) )
    {
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_01 , MOBILELIGHT_OFF_VAL);
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value MOBILELIGHT_OFF_VAL (0x%02x) \n",
                    __func__,
                    MOBILELIGHT_OFF_VAL );
            return;
        }
        uc_vledctrl = ( pst_light_led_data->ul_value&0xFF000000 )>>24 ;
        if(( FLG_OFF == guc_led_on_flg )&&
           ( VLED_NO_TOUCH != (uc_vledctrl&VLED_NO_TOUCH))){
                DEBUG_PRINT("%s() guc_led_on_flg(%d) \n", __func__, guc_led_on_flg ) ;
            mdelay(1) ;
                DEBUG_PRINT("%s() mdelay(1) \n", __func__ ) ;
            lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , VLED_OFF_VAL);
            if (lret) {
                DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value VLED_OFF_VAL (0x%02x) \n",
                        __func__,
                        VLED_OFF_VAL );
                return;
            }
        }
        guc_mobilelight_on_flg = FLG_OFF ;
        DEBUG_PRINT("%s() guc_mobilelight_on_flg(%d) \n", __func__, guc_mobilelight_on_flg ) ;
    }
    else
    {
        switch( (pst_light_led_data->ul_value&0x00FFFFFF) ) 
        {
            case MOBILELIGHT_BATTTERM_LOW_MODE:
                uc_value = MOBILELIGHT_BATTTERM_LOW_VAL ;
                break ;
            case MOBILELIGHT_BATTTERM_HIGH_MODE:
                uc_value = MOBILELIGHT_BATTTERM_HIGH_VAL ;
                break ;
            case MOBILELIGHT_VIEWING_MODE:
                uc_value = MOBILELIGHT_VIEWING_VAL ;
                break ;
            default :
                uc_value = MOBILELIGHT_BATTTERM_LOW_VAL ;
                break ;
        }

        DEBUG_PRINT("%s() pst_light_led_data->ul_value=%d uc_value=%d \n",
                __func__,
                pst_light_led_data->ul_value ,
                uc_value );

        uc_vledctrl = ( pst_light_led_data->ul_value&0xFF000000 )>>24 ;
        if(( FLG_OFF == guc_led_on_flg ) &&
           ( VLED_NO_TOUCH != (uc_vledctrl&VLED_NO_TOUCH)))
        {
            lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_00 , VLED_ON_VAL );

            DEBUG_PRINT("%s() VLED_ON_VAL|guc_reg0_val (0x%02x) \n",
                    __func__,
                    VLED_ON_VAL);
            if (lret) {
                DEBUG_PRINT_ERR("%s() can't set mobile light data ul_value VLED_ON_VAL|guc_reg0_val (0x%02x) \n",
                        __func__,
                        VLED_ON_VAL|guc_reg0_val);
                return;
            }
            mdelay(1) ;
            DEBUG_PRINT("%s() mdelay(1) \n", __func__ ) ;
        }
        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_01 , uc_value);
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set mobile light data uc_value (0x%02x) \n",
                    __func__,
                    uc_value );
            return;
        }

        lret = ledlight_write(pst_light_led_data->pst_client, REG_ADDR_01 , MOBILELIGHT_ON_VAL|uc_value);
        if (lret) {
            DEBUG_PRINT_ERR("%s() can't set mobile light data uc_value MOBILELIGHT_ON_VAL (0x%02x) \n",
                    __func__,
                    MOBILELIGHT_ON_VAL );
            return;
        }

        guc_mobilelight_on_flg = FLG_ON ;
        DEBUG_PRINT("%s() guc_mobilelight_on_flg=(0x%02x) \n",
                __func__,
                guc_mobilelight_on_flg ) ;
    }
    DEBUG_PRINT("%s() end\n", __func__);
}

static void light_led_set(struct led_classdev *pst_cdev, enum led_brightness value)
{
    struct light_led_data_type *pst_light_led_data;

    pst_light_led_data = container_of(pst_cdev, struct light_led_data_type, st_cdev); 

    DEBUG_PRINT("%s() start name=%s value=0x%08x\n",
            __func__,
            pst_cdev->name,
            value);

    mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    pst_light_led_data->ul_value = value ;
    pst_light_led_data->saved_color_value = pst_light_led_data->ul_value;
    mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    schedule_work(&pst_light_led_data->work);

}

static void light_led_work(struct work_struct *work)
{
    struct light_led_data_type *pst_light_led_data = container_of(work,
                     struct light_led_data_type, work);
    uint32_t       delay_time = 0;
    unsigned long  flags;

    DEBUG_PRINT("%s() name=%s\n", __func__, pst_light_led_data->st_cdev.name );
    DEBUG_PRINT("%s() ul_value=0x%08x\n", __func__, pst_light_led_data->ul_value );

    if(!strcmp(pst_light_led_data->st_cdev.name, MOBILELIGHT_INFO))
    {
        mobilelight_set(pst_light_led_data) ;
    }
    else if(!strcmp(pst_light_led_data->st_cdev.name, LED_INFO)) 
    {
        mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
        DEBUG_PRINT("LED DEBUG:light_led_work Call Bink=%2x \n",pst_light_led_data->blink_control);
        if(pst_light_led_data->blink_control == BLINK_REQUEST_ON)
        {
            pst_light_led_data->ul_value = pst_light_led_data->saved_color_value;
            if(pst_light_led_data->blink_off_color)
            {
                pst_light_led_data->saved_color_value |= (VLED_NO_TOUCH<<24);
            } else
            {
                pst_light_led_data->saved_color_value &= 0x00FFFFFF;
            }
        }else if(pst_light_led_data->blink_control == BLINK_REQUEST_OFF)
        {
            pst_light_led_data->ul_value = pst_light_led_data->blink_off_color;
            pst_light_led_data->ul_value |= pst_light_led_data->blink_off_color ? (VLED_NO_TOUCH<<24):0;
        }

        led_set(pst_light_led_data);

        if(pst_light_led_data->blink_control == NO_BLINK_REQUEST)
        {
            alarm_cancel(&pst_light_led_data->led_alarm);
            wake_unlock(&pst_light_led_data->work_wake_lock);
        }
        else
        {
            if(pst_light_led_data->blink_control == BLINK_REQUEST_ON)
            {
                    pst_light_led_data->blink_control = BLINK_REQUEST_OFF;
                    delay_time = pst_light_led_data->blink_on_delay;
            }
            else if(pst_light_led_data->blink_control == BLINK_REQUEST_OFF)
            {
                    pst_light_led_data->blink_control = BLINK_REQUEST_ON;
                    delay_time = pst_light_led_data->blink_off_delay;
            }
            
            pst_light_led_data->last_elapsed_time = alarm_get_elapsed_realtime();
            local_irq_save(flags);
            led_set_alarm(pst_light_led_data,delay_time);
            wake_unlock(&pst_light_led_data->work_wake_lock);
            local_irq_restore(flags);
        }
        mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    }

}

static  enum led_brightness light_led_get(struct led_classdev *pst_cdev)
{
    int32_t lret = 0 ;

    DEBUG_PRINT(" disapproval Call -light_led_get- ");

    return lret ;
}

static int32_t light_led_init_reg( struct light_led_data_type *pst_light_led_data )
{
    int32_t lret = 0;

    DEBUG_PRINT("%s() start\n", __func__ );

    DEBUG_PRINT("%s() end\n", __func__);

    return lret ;
}

static int32_t leds_ioctl(struct inode* inode, struct file* filp, uint32_t cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    T_LEDLIGHT_IOCTL st_ioctl;

    switch (cmd) {
    case LEDLIGHT_SET_BLINK:
        ret = copy_from_user(&st_ioctl, argp, sizeof(T_LEDLIGHT_IOCTL));
        if (ret) {
            printk("error : leds_ioctl(cmd = LEDLIGHT_SET_BLINK)\n" );
            return -EFAULT;
        }
        alarm_cancel(&gpst_light_led_data[LEDLIGHT_INDEX].led_alarm);
        DEBUG_PRINT("%s:%d,%d,%d,%d\n",__func__,st_ioctl.data[0],st_ioctl.data[1],st_ioctl.data[2],st_ioctl.data[3]);
        mutex_lock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
        gpst_light_led_data[LEDLIGHT_INDEX].blink_control       = st_ioctl.data[0];
        gpst_light_led_data[LEDLIGHT_INDEX].blink_on_delay      = st_ioctl.data[1];
        gpst_light_led_data[LEDLIGHT_INDEX].blink_off_delay     = st_ioctl.data[2];
        gpst_light_led_data[LEDLIGHT_INDEX].blink_off_color     = st_ioctl.data[3];
        mutex_unlock(&gpst_light_led_data[LEDLIGHT_INDEX].lock);
    break;
  default:
    return -ENOTTY;
  }
  return 0;
}

static int32_t 
leds_open( struct inode* inode, struct file* filp )
{
    return 0;
}

static int32_t 
leds_release( struct inode* inode, struct file* filp )
{
    return 0;
}

static struct file_operations leds_fops = {
    .owner      = THIS_MODULE,
    .open       = leds_open,
    .release    = leds_release,
    .ioctl      = leds_ioctl,
};
static struct miscdevice leds_device = {
    .minor      = MISC_DYNAMIC_MINOR,
    .name       = "leds-ledlight",
    .fops       = &leds_fops,
};

static void led_class_setinfo(struct light_led_data_type *pst_light_led_data, uint32_t ul_ledindex)
{
    
    switch( ul_ledindex )
    {
        case MOBILELIGHT_INDEX :
            pst_light_led_data[ul_ledindex].st_cdev.name               = MOBILELIGHT_INFO;
            pst_light_led_data[ul_ledindex].st_cdev.max_brightness     = MOBILELIGHT_MAX_BRIGHT_VAL;
        break ;

        case LEDLIGHT_INDEX :
            pst_light_led_data[ul_ledindex].st_cdev.name               = LED_INFO;
            pst_light_led_data[ul_ledindex].st_cdev.max_brightness     = (unsigned int)LEDLIGHT_MAX_BRIGHT_VAL;
        break ;
        
    }

    pst_light_led_data[ul_ledindex].st_cdev.brightness_set     = light_led_set;
    pst_light_led_data[ul_ledindex].st_cdev.brightness_get     = light_led_get;
    pst_light_led_data[ul_ledindex].st_cdev.brightness         = LED_OFF;
    pst_light_led_data[ul_ledindex].st_cdev.flags              = 0;
}

static int32_t __devinit light_led_probe(struct i2c_client *pst_client,
            const struct i2c_device_id *id)
{
    int32_t lret=0;
    struct light_led_data_type *pst_light_led_data;
    int32_t i ; 

    DEBUG_PRINT("%s() start pst_client=0x%08x idname=%s\n", __func__, (unsigned int)pst_client, id->name);

    pst_light_led_data = kzalloc(sizeof(struct light_led_data_type)*LIGHT_INDEX_MAX, GFP_KERNEL);
    if (!pst_light_led_data) {
        DEBUG_PRINT_ERR("%s() failed to allocate driver data\n", __func__);
        return -1;
    }
    for( i=0; i<LIGHT_INDEX_MAX; i++ )
    {
        INIT_WORK(&pst_light_led_data[i].work, light_led_work);
    }
    i2c_set_clientdata(pst_client, pst_light_led_data);

    gpst_light_led_data = pst_light_led_data;
    pst_light_led_data[MOBILELIGHT_INDEX].pst_client  = pst_client;
    pst_light_led_data[LEDLIGHT_INDEX].pst_client     = pst_client;

    for( i=0; i<LIGHT_INDEX_MAX; i++ )
    {
        mutex_init(&pst_light_led_data[i].lock);
        spin_lock_init(&pst_light_led_data[i].value_lock);

        DEBUG_PRINT("%s() light_led_init_reg() call \n", __func__ );
        if ( i == 0){
            lret = light_led_init_reg( &(pst_light_led_data[i]) ) ;
            if(lret){
                DEBUG_PRINT_ERR("%s() light_led_init_reg() lret=%d\n", __func__, lret );
                goto fail_id_check;
            }
        }

        led_class_setinfo(pst_light_led_data, i) ;

        lret = led_classdev_register(&pst_light_led_data[i].pst_client->dev, &pst_light_led_data[i].st_cdev);
        if (lret) {
            DEBUG_PRINT_ERR("%s() unable to register led %s\n",
                    __func__,
                    pst_light_led_data[i].st_cdev.name);
            goto fail_id_check;
        }
        pst_light_led_data[i].blink_control     = NO_BLINK_REQUEST; 
        pst_light_led_data[i].blink_on_delay    = 0;
        pst_light_led_data[i].blink_off_delay   = 0;
        pst_light_led_data[i].blink_off_color   = 0;
        pst_light_led_data[i].saved_color_value = pst_light_led_data[LEDLIGHT_INDEX].ul_value;
    }

    wake_lock_init(&pst_light_led_data[LEDLIGHT_INDEX].work_wake_lock, WAKE_LOCK_SUSPEND,"led-lights");
    alarm_init(&pst_light_led_data[LEDLIGHT_INDEX].led_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,led_alarm);

    DEBUG_PRINT("%s() end\n", __func__ );
    return lret;

fail_id_check:
    while( i-- )
    {
        led_classdev_unregister(&pst_light_led_data[i].st_cdev);
    }

    kfree(pst_light_led_data);
    return lret;

}

static int32_t __exit light_led_remove(struct i2c_client *pst_client)
{
    int32_t lret = 0 ;
    struct light_led_data_type *pst_light_led_data = i2c_get_clientdata(pst_client);
    int32_t i ;

    DEBUG_PRINT("%s() start\n", __func__ );

    for( i=0; i<LIGHT_INDEX_MAX; i++ )
    {
        led_classdev_unregister(&pst_light_led_data[i].st_cdev);
    }

    kfree(pst_light_led_data);

    DEBUG_PRINT("%s() end\n", __func__ );
    return lret;
}

static int32_t light_led_suspend(struct i2c_client *pst_client, pm_message_t mesg)
{
    int32_t lret= 0 ;
    DEBUG_PRINT("%s() start \n", __func__ ) ;
    return lret;
}

static int32_t light_led_resume(struct i2c_client *pst_client)
{
    int32_t lret= 0 ;
    DEBUG_PRINT("%s() start \n", __func__ ) ;
    return lret;
}

static const struct i2c_device_id ledlight_id[] = {
    { LED_DRV_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, ledlight_id);

static struct i2c_driver light_led_driver = {
    .driver = {
        .name   = LED_DRV_NAME,
    },
    .probe      = light_led_probe,
    .remove     = __exit_p(light_led_remove),
    .suspend    = light_led_suspend,
    .resume     = light_led_resume,
    .id_table   = ledlight_id,
};

static int32_t __init light_led_init(void)
{
    int32_t lret=0;
    DEBUG_PRINT("%s() start\n", __func__ );

    DEBUG_PRINT("%s() i2c_add_driver() call\n", __func__ );
    lret=i2c_add_driver(&light_led_driver);
    DEBUG_PRINT("%s() i2c_add_driver() call end lret=%d\n", __func__ , lret );

    misc_register(&leds_device);

    DEBUG_PRINT("%s() end\n", __func__ );
    return lret ;

}
module_init(light_led_init);

static void __exit light_led_exit(void)
{
    DEBUG_PRINT("%s() start\n", __func__ );
    misc_deregister(&leds_device);
    return i2c_del_driver(&light_led_driver);
}
module_exit(light_led_exit);

#if 0
static void debug_msg_out( struct light_led_data_type *pst_light_led_data )
{
    DEBUG_PRINT("%s() <<LOCALSTRUCT>> \n", __func__);
    DEBUG_PRINT("%s()   id=%d \n",
              __func__,
              pst_light_led_data->id );
    DEBUG_PRINT("%s()   value=0x%08x\n", 
              __func__,
              pst_light_led_data->value );
    DEBUG_PRINT("%s()   led_on_flg=%d\n",
              __func__,
              guc_led_on_flg );
    DEBUG_PRINT("%s()   mobilelight_on_flg=%d\n",
              __func__,
              guc_mobilelight_on_flg );

    DEBUG_PRINT("%s() <<LED CLASS DEV>>\n",  __func__ );
    DEBUG_PRINT("%s()   name=%s \n",
            __func__,
            pst_light_led_data->st_cdev.name );
    DEBUG_PRINT("%s()   brightness=0x%08x \n",
            __func__,
            pst_light_led_data->st_cdev.brightness );
    DEBUG_PRINT("%s()   max_brightness=0x%08x \n",
            __func__,
            pst_light_led_data->st_cdev.max_brightness );
    DEBUG_PRINT("%s()   flags=%d \n",
            __func__,
            pst_light_led_data->st_cdev.flags );

}
#endif

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("ledlight driver");
MODULE_LICENSE("GPL");

