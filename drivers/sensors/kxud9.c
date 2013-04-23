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
#include <linux/moduleparam.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sensors/kxud9.h>
#include <linux/workqueue.h>

/* #define CONFIG_KXUD9_DEBUG */

#define I2C_RETRIES    5

#define ACC_DRIVER_NAME "accsns"
#define I2C_BUS_NUMBER  0

#define INVALID_VALUE 0xf0000000
#define AXIS_CNT 3
#define AXIS_X   0
#define AXIS_Y   1
#define AXIS_Z   2

#define AXIS_REG_SIZE         2
#define AXIS_LOW_OFFSET       1

#define INVALID_NUM          -1
#define DEFAULT_FREQ         50
#define ONESEC_MS          1000
#define PREV_INDEX            1
#define SETTING_0G         2048
#define WEIGHT_1G           819
#define MODE_0             0x00
#define MODE_1             0x01
#define MODE_2             0x02
#define MODE_3             0x03
#define MODE_INVALID    INVALID_NUM
#define DEVIDE_HALF          2
#define OFFSET_SUMPLE_NUM  100
#define OFFSET_AVE_NUM       1

#define I2C_REG_LENGTH       1
#define I2C_READ_SIZE        2
#define I2C_WRITE_SIZE       1
#define LITTLE_WAIT          1
#define I2C_WRITE_REG_SIZE   2

#define POWER_UP_DELAY      16

#define I2C_INDEX_REG        0
#define I2C_INDEX_VAL        1
#define I2C_BYTE_MASK       0xff

#define POWER_ENABLE         1

#define SHIT_REG             4
#define LOW_BYTE_MASK       0x000f
#define HIGH_BYTE_MASK      0x0ff0

#define CALIB_CONTINUE       0
#define CALIB_FINISH         1
#define RESET_WAIT           5

struct acceleration {
    int32_t nX;
    int32_t nY;
    int32_t nZ;
};

typedef struct tMovFilterWork {
    int32_t* m_pSamplWork[AXIS_CNT];
    int32_t  m_unSum[AXIS_CNT];
    int32_t  m_unCnt[AXIS_CNT];
    uint8_t  m_ucAveN;
} MovFilterWork;

typedef struct tCalibrationCtrl {
    bool     m_bFilterEnable;
    bool     m_bWaitSetting;
    uint16_t m_unSmpN;
    MovFilterWork m_tFilterWork;
    int32_t  m_nCalX;
    int32_t  m_nCalY;
    int32_t  m_nCalZ;
    int32_t  m_nCurrentSampleNum;
    int32_t  m_nSummationX;
    int32_t  m_nSummationY;
    int32_t  m_nSummationZ;
    int32_t  m_nMode;
    struct mutex m_tCalibMutex;
} CalibrationCtrl;

static CalibrationCtrl s_tCalibCtrl;
static struct i2c_driver accsns_driver;
static struct i2c_client *client_accsns = NULL;
static atomic_t g_flgEna;
static atomic_t g_CurrentSensorEnable;
static struct delayed_work s_tDelay;
static struct acceleration s_tLatestAccData;
static atomic_t s_nDelay;
static struct mutex s_tDataMutex;

static atomic_t g_nCalX;
static atomic_t g_nCalY;
static atomic_t g_nCalZ;

static int32_t mov_acc_avg (MovFilterWork* pCtrl, int32_t sample,int32_t axis)
{
#ifdef CONFIG_KXUD9_DEBUG
  int32_t i = 0;
  
#endif
  if (pCtrl->m_pSamplWork[axis] == NULL) {
    pCtrl->m_pSamplWork[axis] = kzalloc(sizeof(int32_t) * pCtrl->m_ucAveN, GFP_KERNEL);
    
    if (pCtrl->m_pSamplWork[axis] == NULL) {
      printk("ACC-Calib:Memory crisis\n");
      return INVALID_VALUE;
    }
    memset(pCtrl->m_pSamplWork[axis], 0x00, sizeof(int32_t) * pCtrl->m_ucAveN);
  }

  if (pCtrl->m_unCnt[axis]++ < pCtrl->m_ucAveN) {
    pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-PREV_INDEX) % pCtrl->m_ucAveN] = sample;
    pCtrl->m_unSum[axis] += sample;
#ifdef CONFIG_KXUD9_DEBUG
    printk("ACC-Calib:List %d:",pCtrl->m_unCnt[axis]);
    for (i = 0; i < pCtrl->m_ucAveN; i++) {
      printk("%d,",pCtrl->m_pSamplWork[axis][i]);
    }
    printk("\n");
#endif
    
    return INVALID_VALUE;
  } else {
    pCtrl->m_unSum[axis] -= pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-PREV_INDEX) % pCtrl->m_ucAveN];
    pCtrl->m_unSum[axis] += sample;
    pCtrl->m_pSamplWork[axis][(pCtrl->m_unCnt[axis]-1) % pCtrl->m_ucAveN] = sample;
  }

#ifdef CONFIG_KXUD9_DEBUG
  printk("List %d:",pCtrl->m_unCnt[axis]);
  for (i = 0; i < pCtrl->m_ucAveN; i++) {
    printk("%d,",pCtrl->m_pSamplWork[axis][i]);
  }
  printk("\n");
#endif
  
  return pCtrl->m_unSum[axis] / pCtrl->m_ucAveN;
}

static void accsns_calibration_periodic
(
const struct acceleration* accData
)
{
  int32_t nRet = 0;
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));

  printk("calibPeriodic:%d\n",s_tCalibCtrl.m_nCurrentSampleNum);
  if (s_tCalibCtrl.m_nCurrentSampleNum == s_tCalibCtrl.m_unSmpN) {
    s_tCalibCtrl.m_bWaitSetting = true;
  } else {
    if (s_tCalibCtrl.m_bFilterEnable == true) {
      nRet = mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nX, AXIS_X);
      if (nRet != INVALID_VALUE) {
        s_tCalibCtrl.m_nCurrentSampleNum++;
        s_tCalibCtrl.m_nSummationX += nRet;
      }
      nRet = mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nY, AXIS_Y);
      if (nRet != INVALID_VALUE) {
        s_tCalibCtrl.m_nSummationY += nRet;
      }
      nRet = mov_acc_avg(&(s_tCalibCtrl.m_tFilterWork), accData->nZ, AXIS_Z);
      if (nRet != INVALID_VALUE) {
        s_tCalibCtrl.m_nSummationZ += nRet;
      }
    } else {
      s_tCalibCtrl.m_nSummationX += accData->nX;
      s_tCalibCtrl.m_nSummationY += accData->nY;
      s_tCalibCtrl.m_nSummationZ += accData->nZ;
      
      s_tCalibCtrl.m_nCurrentSampleNum++;
    }
    if (s_tCalibCtrl.m_nCurrentSampleNum == s_tCalibCtrl.m_unSmpN) {
      switch (s_tCalibCtrl.m_nMode) {
      case MODE_0:
        s_tCalibCtrl.m_nCalX = SETTING_0G - (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN);
        s_tCalibCtrl.m_nCalY = SETTING_0G - (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN);
        s_tCalibCtrl.m_nCalZ = 0;
        printk("AccCalib:mode0 complete calX = %d sumX = %u smp=%u\n", s_tCalibCtrl.m_nCalX, s_tCalibCtrl.m_nSummationX, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode0 complete calY = %d sumY = %u smp=%u\n", s_tCalibCtrl.m_nCalY, s_tCalibCtrl.m_nSummationY, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode0 complete calZ = %d sumZ = %u smp=%u\n", s_tCalibCtrl.m_nCalZ, s_tCalibCtrl.m_nSummationZ, s_tCalibCtrl.m_unSmpN);
        break;
      case MODE_1:
        s_tCalibCtrl.m_nCalX = SETTING_0G - (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN);
        s_tCalibCtrl.m_nCalY = SETTING_0G - (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN);
        s_tCalibCtrl.m_nCalZ = SETTING_0G - (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN)+WEIGHT_1G; 

        printk("AccCalib:mode1 complete calX = %d sumX = %u smp=%u\n", s_tCalibCtrl.m_nCalX, s_tCalibCtrl.m_nSummationX, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode1 complete calY = %d sumY = %u smp=%u\n", s_tCalibCtrl.m_nCalY, s_tCalibCtrl.m_nSummationY, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode1 complete calZ = %d sumZ = %u smp=%u\n", s_tCalibCtrl.m_nCalZ, s_tCalibCtrl.m_nSummationZ, s_tCalibCtrl.m_unSmpN);
        break;
      case MODE_2:
        s_tCalibCtrl.m_nCalX = (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN);
        s_tCalibCtrl.m_nCalY = (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN);
        s_tCalibCtrl.m_nCalZ = (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode2 complete calX = %d sumX = %u smp=%u\n", s_tCalibCtrl.m_nCalX, s_tCalibCtrl.m_nSummationX, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode2 complete calY = %d sumY = %u smp=%u\n", s_tCalibCtrl.m_nCalY, s_tCalibCtrl.m_nSummationY, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode2 complete calZ = %d sumZ = %u smp=%u\n", s_tCalibCtrl.m_nCalZ, s_tCalibCtrl.m_nSummationZ, s_tCalibCtrl.m_unSmpN);
        break;
      case MODE_3:
        s_tCalibCtrl.m_nCalX = SETTING_0G - ((s_tCalibCtrl.m_nCalX + (s_tCalibCtrl.m_nSummationX / s_tCalibCtrl.m_unSmpN)) /DEVIDE_HALF);
        s_tCalibCtrl.m_nCalY = SETTING_0G - ((s_tCalibCtrl.m_nCalY + (s_tCalibCtrl.m_nSummationY / s_tCalibCtrl.m_unSmpN)) /DEVIDE_HALF);
        s_tCalibCtrl.m_nCalZ = SETTING_0G - ((s_tCalibCtrl.m_nCalZ + (s_tCalibCtrl.m_nSummationZ / s_tCalibCtrl.m_unSmpN)) /DEVIDE_HALF); 
        printk("AccCalib:mode3 complete calX = %d sumX = %u smp=%u\n", s_tCalibCtrl.m_nCalX, s_tCalibCtrl.m_nSummationX, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode3 complete calY = %d sumY = %u smp=%u\n", s_tCalibCtrl.m_nCalY, s_tCalibCtrl.m_nSummationY, s_tCalibCtrl.m_unSmpN);
        printk("AccCalib:mode3 complete calZ = %d sumZ = %u smp=%u\n", s_tCalibCtrl.m_nCalZ, s_tCalibCtrl.m_nSummationZ, s_tCalibCtrl.m_unSmpN);
        break;
      default:
        printk("ACC-Calib: Mode Err!!\n");
        break;
      }

      kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_X]);
      kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Y]);
      kfree(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Z]);
      s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_X] = NULL;
      s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Y] = NULL;
      s_tCalibCtrl.m_tFilterWork.m_pSamplWork[AXIS_Z] = NULL;
    }
  }
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
}

static int32_t accsns_calibration_mode(void)
{
  int32_t i = 0;
  
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));

  s_tCalibCtrl.m_bFilterEnable        = false;
  s_tCalibCtrl.m_bWaitSetting         = true;

  s_tCalibCtrl.m_unSmpN               = OFFSET_SUMPLE_NUM; 
  s_tCalibCtrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;   
  s_tCalibCtrl.m_nCalX                = 0;
  s_tCalibCtrl.m_nCalY                = 0;
  s_tCalibCtrl.m_nCalZ                = 0;

  s_tCalibCtrl.m_nCurrentSampleNum    = 0;
  s_tCalibCtrl.m_nSummationX          = 0;
  s_tCalibCtrl.m_nSummationY          = 0;
  s_tCalibCtrl.m_nSummationZ          = 0;
  s_tCalibCtrl.m_nMode                = MODE_INVALID; 
  
  for (i = 0; i < AXIS_CNT; i++) {
    if(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
      printk("ACC Calib:err occurd \n");
    } else {
      s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] = NULL;
    }
  }
  memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_CNT);
  memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_CNT);

  printk("Enter AccCalibration Mode\n");
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
  
  return 0;
}

static int32_t accsns_calibration_start
(
 int32_t argMode
)
{
  int32_t delay = 0;
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
  s_tCalibCtrl.m_bWaitSetting = false;
  s_tCalibCtrl.m_nMode = argMode;

  s_tCalibCtrl.m_nCurrentSampleNum    = 0;
  s_tCalibCtrl.m_nSummationX          = 0;
  s_tCalibCtrl.m_nSummationY          = 0;
  s_tCalibCtrl.m_nSummationZ          = 0;

  memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_CNT);
  memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_CNT);

  delay = atomic_read(&s_nDelay);
  printk("Start AccCalibration Mode %d[%d]\n", s_tCalibCtrl.m_nMode,delay);
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

  schedule_delayed_work(&s_tDelay,msecs_to_jiffies(delay));

  return 0;
}

static int32_t accsns_calibration_is_wait
(
 int32_t* argCal
)
{
  int32_t iRet = 0;
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
  if (s_tCalibCtrl.m_bWaitSetting == true) {
    argCal[AXIS_X] = s_tCalibCtrl.m_nCalX;
    argCal[AXIS_Y] = s_tCalibCtrl.m_nCalY;
    argCal[AXIS_Z] = s_tCalibCtrl.m_nCalZ;
  }
  iRet = s_tCalibCtrl.m_bWaitSetting;
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

  printk("calib Wait %d\n", iRet);
  return iRet;
}

static int32_t accsns_get_calibration_value
(
 int32_t* argCal
)
{
  int32_t iRet = 0;
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
  if (s_tCalibCtrl.m_bWaitSetting == true) {
    argCal[AXIS_X] = s_tCalibCtrl.m_nCalX;
    argCal[AXIS_Y] = s_tCalibCtrl.m_nCalY;
    argCal[AXIS_Z] = s_tCalibCtrl.m_nCalZ;
  }
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));

  return iRet;
}

int32_t accsns_i2c_readm
(
 uint8_t* arg_ucpReadData,
 int32_t arg_iLength
)
{
  int32_t iRet = INVALID_NUM;
  int32_t iTries = 0;
  
  struct i2c_msg msgs[] = {
    {
      .addr = client_accsns->addr,
      .flags = 0,
      .len = I2C_REG_LENGTH,
      .buf = arg_ucpReadData,
    },
    {
      .addr = client_accsns->addr,
      .flags = I2C_M_RD,
      .len = arg_iLength,
      .buf = arg_ucpReadData,
    },
  };
  
  do {
    iRet = i2c_transfer(client_accsns->adapter, msgs, I2C_READ_SIZE);
    if (iRet != I2C_READ_SIZE) msleep(LITTLE_WAIT);
  } while ((iRet != I2C_READ_SIZE) && (++iTries < I2C_RETRIES));
 
  if (iRet != I2C_READ_SIZE) {

    dev_err(&client_accsns->adapter->dev, "read transfer error\n");
    iRet = -EIO;
  } else {
    iRet = 0;
  }
  return iRet;
}

int32_t accsns_i2c_writem
(
 uint8_t* ucpWriteData,
 int32_t iLength
 )
{
  int32_t iRet = INVALID_NUM;
  int32_t iTries = 0;
#ifdef CONFIG_KXUD9_DEBUG
  int32_t i;
#endif
  struct i2c_msg msg[] = {
    {
      .addr = client_accsns->addr,
      .flags = 0,
      .len = iLength,
      .buf = ucpWriteData,
    },
  };
  
#ifdef CONFIG_KXUD9_DEBUG
  printk("[ACC] i2c_writem : ");
  for (i=0; i < iLength;i++) printk("0X%02X, ", *(ucpWriteData + i));
  printk("\n");
#endif
  
  do {
    iRet = i2c_transfer(client_accsns->adapter, msg, I2C_WRITE_SIZE);
    if (iRet != I2C_WRITE_SIZE) msleep(LITTLE_WAIT);
  } while ((iRet != I2C_WRITE_SIZE) && (++iTries < I2C_RETRIES));
  
  if (iRet != I2C_WRITE_SIZE) { 
    dev_err(&client_accsns->adapter->dev, "write transfer error\n");
    iRet = -EIO;
  } else {
    iRet = 0;
  }
  return iRet;
}

static int32_t accsns_power_down
(
 bool arg_bEnable
)
{
  uint8_t ucBuffer[I2C_WRITE_REG_SIZE];
  int32_t nDelay = 0;
  int32_t iRet  = INVALID_NUM; 

  printk("accsns_power_down %d\n", arg_bEnable);
  nDelay = atomic_read(&s_nDelay) + POWER_UP_DELAY;
  if(arg_bEnable == true) {
    ucBuffer[I2C_INDEX_VAL] = (~ENABLE & ENABLE) & I2C_BYTE_MASK;

    cancel_delayed_work_sync(&s_tDelay);
  } else {
    ucBuffer[I2C_INDEX_VAL] = (ENABLE | CLKHLD);
    schedule_delayed_work(&s_tDelay, msecs_to_jiffies(nDelay));
  }
  ucBuffer[I2C_INDEX_REG] = KXUD9_CTRL_REGB_REG;
  iRet = accsns_i2c_writem(ucBuffer, I2C_WRITE_REG_SIZE);
  return iRet;
}

void accsns_activate
(
 int32_t arg_iUpdate,
 int32_t arg_iEnable
 )
{
  bool bIsChanged = false;

  if (arg_iEnable != 0) {
    arg_iEnable = POWER_ENABLE;
  }
  
  if(arg_iEnable) {
    if(atomic_read(&g_CurrentSensorEnable) != POWER_ENABLE) {
      accsns_power_down(false);
      bIsChanged = true;
      msleep(POWER_UP_DELAY);
    }
  } else {
    if(atomic_read(&g_CurrentSensorEnable) != 0) {
      accsns_power_down(true);

      bIsChanged = true;
    }
  }
  if(bIsChanged) {
    atomic_set(&g_CurrentSensorEnable,arg_iEnable);
    msleep(LITTLE_WAIT);
  }
  if (arg_iUpdate) {
    atomic_set(&g_flgEna, arg_iEnable);
  }
}

int32_t accsns_measure
(
 void
)

{
  int32_t iRet = 0;
  uint8_t ucBuff[AXIS_CNT*AXIS_REG_SIZE];
  uint8_t* pucReg = &ucBuff[0];

  ucBuff[I2C_INDEX_REG] = KXUD9_XOUT_H_REG;
  iRet = accsns_i2c_readm(ucBuff,AXIS_CNT*AXIS_REG_SIZE);
  
  mutex_lock(&s_tDataMutex);
  s_tLatestAccData.nX = 
    (((int32_t)(pucReg[AXIS_X*AXIS_REG_SIZE]) << SHIT_REG) & HIGH_BYTE_MASK) | (((int32_t)(pucReg[AXIS_X*AXIS_REG_SIZE+AXIS_LOW_OFFSET]) >> SHIT_REG) & LOW_BYTE_MASK);
  s_tLatestAccData.nY = 
    (((int32_t)(pucReg[AXIS_Y*AXIS_REG_SIZE]) << SHIT_REG) & HIGH_BYTE_MASK) | (((int32_t)(pucReg[AXIS_Y*AXIS_REG_SIZE+AXIS_LOW_OFFSET]) >> SHIT_REG) & LOW_BYTE_MASK);
  s_tLatestAccData.nZ = 
    (((int32_t)(pucReg[AXIS_Z*AXIS_REG_SIZE]) << SHIT_REG) & HIGH_BYTE_MASK) | (((int32_t)(pucReg[AXIS_Z*AXIS_REG_SIZE+AXIS_LOW_OFFSET]) >> SHIT_REG) & LOW_BYTE_MASK);

#ifdef CONFIG_KXUD9_DEBUG
  printk("AccI2c: iRet=%d x:%d, y:%d, z:%d\n", 
    iRet,s_tLatestAccData.nX, s_tLatestAccData.nY, s_tLatestAccData.nZ);
#endif
  mutex_unlock(&s_tDataMutex);
  
  return iRet;
}

int32_t accsns_get_acceleration_data
(
 int32_t* arg_ipXYZ
)
{
  int32_t nRet = 0;

  int32_t nCalX,nCalY,nCalZ;

  nCalX = atomic_read(&g_nCalX);
  nCalY = atomic_read(&g_nCalY);
  nCalZ = atomic_read(&g_nCalZ);

  if (delayed_work_pending(&s_tDelay)) {
  }
  else {
    nRet = accsns_measure();
  }
  
  mutex_lock(&s_tDataMutex);
  arg_ipXYZ[0] = s_tLatestAccData.nX + (int16_t)nCalX;
  arg_ipXYZ[1] = s_tLatestAccData.nY + (int16_t)nCalY;
  arg_ipXYZ[2] = s_tLatestAccData.nZ + (int16_t)nCalZ;
  mutex_unlock(&s_tDataMutex);
  
#ifdef CONFIG_KXUD9_DEBUG
  printk("Acc_I2C, iRet=%d x:%d, y:%d, z:%d\n", 
    nRet,*(arg_ipXYZ), *(arg_ipXYZ + 1), *(arg_ipXYZ + 2));
#endif
  
  return nRet;
}

int32_t accsns_get_acceleration_pitch_and_roll_data
(
 int32_t* arg_ipData
 )
{
  int32_t iRet = 0;
  
  return iRet;
}

static void accsns_register_init
(
 void
 )
{
  uint8_t ucBuffer[I2C_WRITE_REG_SIZE];
  int32_t iRet = INVALID_NUM;

#ifdef CONFIG_KXUD9_DEBUG
  printk("[ACC] register_init\n");
#endif
  ucBuffer[I2C_INDEX_REG] = KXUD9_RESET_WRITE_REG;
  ucBuffer[I2C_INDEX_VAL] = SOFTWARE_POWER_UP_RESET_CODE;
  iRet = accsns_i2c_writem(ucBuffer, I2C_WRITE_REG_SIZE);

  msleep(RESET_WAIT);

  ucBuffer[I2C_INDEX_REG] = KXUD9_CTRL_REGC_REG;
  ucBuffer[I2C_INDEX_VAL] = CONFIG_2G_RANGE | CONFIG_50HZ_FREQUENCY;
  iRet = accsns_i2c_writem(ucBuffer, I2C_WRITE_REG_SIZE);
  if(iRet == 0) {
    iRet = accsns_power_down(true);
  }
}

static void accsns_work_func(struct work_struct *work)
{
  int32_t nRet = INVALID_NUM;
  int32_t delay = 0;
  bool bCalibIdle = false;
  
  if (atomic_read(&g_CurrentSensorEnable) != POWER_ENABLE) {
      printk("That is probably soft bug!! Illigal timing\n");
  }
  
  nRet = accsns_measure();
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
  bCalibIdle = s_tCalibCtrl.m_bWaitSetting;
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
  
  if (bCalibIdle == true) {
    nRet = CALIB_FINISH;
  } else {
    mutex_lock(&s_tDataMutex);
    accsns_calibration_periodic(&s_tLatestAccData);
    mutex_unlock(&s_tDataMutex);
  }

  if (nRet == CALIB_CONTINUE) {
    delay = atomic_read(&s_nDelay);
    schedule_delayed_work(&s_tDelay,msecs_to_jiffies(delay));
  }
}

static int32_t accsns_probe
(
 struct i2c_client *client,
 const struct i2c_device_id *id
 )
{
  int32_t i = 0;
  printk("[ACC] probe\n");

  mutex_init(&s_tDataMutex);
  mutex_init(&(s_tCalibCtrl.m_tCalibMutex));

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    dev_err(&client->adapter->dev, "client not i2c capable\n");
    return -ENOMEM;
  }
  
  mutex_lock(&(s_tCalibCtrl.m_tCalibMutex));
  s_tCalibCtrl.m_bFilterEnable        = false;
  s_tCalibCtrl.m_bWaitSetting         = true;
  s_tCalibCtrl.m_unSmpN               = OFFSET_SUMPLE_NUM; 

  s_tCalibCtrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;   
  s_tCalibCtrl.m_nCalX                = 0;
  s_tCalibCtrl.m_nCalY                = 0;
  s_tCalibCtrl.m_nCalZ                = 0;
 
  s_tCalibCtrl.m_nCurrentSampleNum    = 0;
  s_tCalibCtrl.m_nSummationX          = 0;
  s_tCalibCtrl.m_nSummationY          = 0;
  s_tCalibCtrl.m_nSummationZ          = 0;
  s_tCalibCtrl.m_nMode                = MODE_INVALID; 
  for (i = 0; i < AXIS_CNT; i++) {
    if(s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
      printk("ACC Calib:err occurd \n");
    } else {
      s_tCalibCtrl.m_tFilterWork.m_pSamplWork[i] = NULL;
    }
  }
  memset(s_tCalibCtrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_CNT);
  memset(s_tCalibCtrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_CNT);

  printk("Init AccCalibration Ctrl\n");
  mutex_unlock(&(s_tCalibCtrl.m_tCalibMutex));
  
  client_accsns = client;
  
  INIT_DELAYED_WORK(&s_tDelay, accsns_work_func);

  atomic_set(&g_nCalX,0);
  atomic_set(&g_nCalY,0);
  atomic_set(&g_nCalZ,0);

  accsns_register_init();
  accsns_activate(POWER_ENABLE, atomic_read(&g_flgEna));
  
  return 0;
}

static void accsns_shutdown
(
 struct i2c_client *client
 )
{
  printk("[ACC] shutdown\n");
  accsns_activate(0, 0);
}

static int32_t accsns_suspend
(
 struct i2c_client *client,
 pm_message_t mesg
 )
{
  printk("[ACC] suspend\n");
  accsns_activate(0, 0);
  
  return 0;
}

static int32_t accsns_resume
(
 struct i2c_client *client
 )
{
  printk("[ACC] resume\n");
  accsns_activate(0, atomic_read(&g_flgEna));
  
  return 0;
}

static __devexit int32_t accsns_remove
(
 struct i2c_client *client
 )
{
  accsns_activate(0, 0);

  cancel_delayed_work_sync(&s_tDelay);
  
  printk("[ACC] remove\n");
  return 0;
}

static const struct i2c_device_id kxud9_id[] = {
  { ACC_DRIVER_NAME, 0 },
  { }
};

static struct i2c_driver accsns_driver = {
  .probe     = accsns_probe,
  .id_table  = kxud9_id,
  .driver  = {
    .name  = ACC_DRIVER_NAME,
  },
  .shutdown  = accsns_shutdown,
  .suspend   = accsns_suspend,
  .resume    = accsns_resume,
  .remove    = accsns_remove,
};

static int32_t __init accsns_init
(
 void
 )
{
  struct i2c_board_info i2c_info;
  struct i2c_adapter *adapter;
  int32_t rc = INVALID_NUM;
  
#ifdef CONFIG_KXUD9_DEBUG
  printk("[ACC] init\n");
#endif
  atomic_set(&g_flgEna, 0);
  atomic_set(&g_CurrentSensorEnable,-1);
  
  rc = i2c_add_driver(&accsns_driver);
  if (rc != 0) {
    printk("can't add i2c driver\n");
    rc = -ENOTSUPP;
    return rc;
  }
  
  memset(&i2c_info, 0, sizeof(struct i2c_board_info));
  i2c_info.addr = I2C_SLAVE_WRITE_ADDR;
  strlcpy(i2c_info.type, ACC_DRIVER_NAME , I2C_NAME_SIZE);
  
  adapter = i2c_get_adapter(I2C_BUS_NUMBER);
  if (!adapter) {
    printk("can't get i2c adapter %d\n", I2C_BUS_NUMBER);
    rc = -ENOTSUPP;
    goto probe_done;
  }
  
  client_accsns = i2c_new_device(adapter, &i2c_info);
  client_accsns->adapter->timeout = 0;
  client_accsns->adapter->retries = 0;
  
  i2c_put_adapter(adapter);
  if (!client_accsns) {
    printk("can't add i2c device at 0x%x\n",(unsigned int)i2c_info.addr);
    rc = -ENOTSUPP;
  }
  
#ifdef CONFIG_KXUD9_DEBUG
  printk("accsns_open end !!!!\n");
#endif
  
 probe_done: 
  
  return rc;
}

static void __exit accsns_exit
(
 void
 )
{
#ifdef CONFIG_KXUD9_DEBUG
  printk("[ACC] exit\n");
#endif
  i2c_del_driver(&accsns_driver);
  i2c_unregister_device(client_accsns);
  client_accsns = NULL;
}

static void accsns_set_freq
(
 uint8_t freq
 )
{
  int32_t delay = 0;

  if (freq == 0) {
    freq = DEFAULT_FREQ;
  }

  atomic_set(&s_nDelay, ONESEC_MS/freq); 
  atomic_set(&s_nDelay, ONESEC_MS/freq);
  delay = atomic_read(&s_nDelay);

  printk("accsns freq = %d (%d)\n", freq,delay);
}


void accsns_set_offset
(
  int32_t* offsets
 )
{
  atomic_set(&g_nCalX, offsets[AXIS_X]);
  atomic_set(&g_nCalY, offsets[AXIS_Y]);
  atomic_set(&g_nCalZ, offsets[AXIS_Z]);

  printk("accsns set offset %d %d %d\n", offsets[AXIS_X], offsets[AXIS_Y], offsets[AXIS_Z]);
}

int32_t accsns_pedometer_get_time(void) { return INVALID_NUM; }
int32_t accsns_pedometer_clear(void) { return INVALID_NUM; }
int32_t accsns_pedometer_set_param(int32_t nStride, int32_t nWeight) { return INVALID_NUM; }
int32_t accsns_pedometer_enable(bool bEnable) { return INVALID_NUM; }
int32_t accsns_pedometer_get_value(int32_t* nSteps, int32_t* nCal) { return INVALID_NUM; }

int32_t accsns_get_i2c_err(void) { return 0; }
void accsns_error_reset(void) {}
void accsns_set_delay(int32_t delay) {}

module_init(accsns_init);
module_exit(accsns_exit);

EXPORT_SYMBOL(accsns_set_freq);
EXPORT_SYMBOL(accsns_get_acceleration_pitch_and_roll_data);
EXPORT_SYMBOL(accsns_get_acceleration_data);
EXPORT_SYMBOL(accsns_activate);
EXPORT_SYMBOL(accsns_i2c_readm);
EXPORT_SYMBOL(accsns_i2c_writem);
EXPORT_SYMBOL(accsns_calibration_mode);
EXPORT_SYMBOL(accsns_calibration_start);
EXPORT_SYMBOL(accsns_calibration_is_wait);
EXPORT_SYMBOL(accsns_get_calibration_value);
EXPORT_SYMBOL(accsns_set_offset);

EXPORT_SYMBOL(accsns_pedometer_get_time);
EXPORT_SYMBOL(accsns_pedometer_clear);
EXPORT_SYMBOL(accsns_pedometer_set_param);
EXPORT_SYMBOL(accsns_pedometer_enable);
EXPORT_SYMBOL(accsns_pedometer_get_value);

EXPORT_SYMBOL(accsns_get_i2c_err);
EXPORT_SYMBOL(accsns_error_reset);
EXPORT_SYMBOL(accsns_set_delay);

EXPORT_SYMBOL(accsns_suspend);
EXPORT_SYMBOL(accsns_resume);

MODULE_DESCRIPTION("Kionix Acc Device");
MODULE_AUTHOR("KCR");
MODULE_LICENSE("GPL v2");
