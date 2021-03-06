/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
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
#ifdef IMAGE_MODEM_PROC
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
#else
    #include <linux/types.h>
    #include <linux/module.h>
    #include <linux/init.h>
    #include <linux/fs.h>
    #include <linux/errno.h>

    #include <linux/miscdevice.h>
    #include <linux/sched.h>
    #include <linux/wait.h>
    #include <linux/uaccess.h>
    #include <linux/slab.h>

    #include <linux/kernel.h>
    #include <linux/list.h>
    #include <linux/module.h>
    #include <linux/mutex.h>

    #include <linux/dnand_cdev_driver.h>

static int32_t inter_param_check(
            dnand_data_type      *pbuf )
{
    int32_t    rtn;
    
    rtn    = DNAND_NO_ERROR;
    if( pbuf->cid >= DNAND_ID_ENUM_MAX )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    if( pbuf->pbuf == NULL )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    if( pbuf->size == 0 )
    {
        rtn    = DNAND_PARAM_ERROR;
    }
    return(rtn);
}

static int dnand_cdev_driver_open(
            struct inode        *inode,
            struct file         *file )
{
    printk(KERN_INFO "dnand_cdev_driveropen\n");
    return 0;
}
/*
static int dnand_cdev_driver_ioctl(
            struct inode        *inode,
            struct file         *filp,
            unsigned int        cmd,
            unsigned long       data )
*/
static long dnand_cdev_driver_ioctl(
            struct file         *filp,
            unsigned int        cmd,
            unsigned long       data )
{
    int                rtn,tmprtn;
    uint8_t            *p_kbuf;
    uint8_t            *p_ubuf;
    dnand_data_type    ddatabuf;

    rtn    = copy_from_user(&ddatabuf, (void *)data, sizeof(ddatabuf));
    if( rtn != 0 )
    {
        return(DNAND_PARAM_ERROR);
    }

    rtn    = inter_param_check(&ddatabuf);
    if( rtn != DNAND_NO_ERROR )
    {
        if( ddatabuf.size == 0 )
        {
            rtn    = DNAND_NO_ERROR;
        }
        return(rtn);
    }

    p_kbuf    = (uint8_t*)kmalloc(ddatabuf.size, GFP_KERNEL);
    if( p_kbuf == NULL )
    {
        return(DNAND_NOMEM_ERROR);
    }
    printk(KERN_DEBUG "%s: cmd:%d \n", __func__, cmd );

    p_ubuf    = ddatabuf.pbuf;

    switch(cmd)
    {
        case DNAND_CDEV_DRIVER_IOCTL_01:
        {
            rtn    = copy_from_user(p_kbuf, p_ubuf, ddatabuf.size);
            if( rtn != 0 )
            {
                rtn = (DNAND_PARAM_ERROR);
            }
            else
            {
                ddatabuf.pbuf    = p_kbuf;
                rtn    = kdnand_id_write(
                             ddatabuf.cid,
                             ddatabuf.offset,
                             ddatabuf.pbuf,
                             ddatabuf.size );
            }
        }
        break;
        
        case DNAND_CDEV_DRIVER_IOCTL_02:
        {
            ddatabuf.pbuf    = p_kbuf;
            rtn    = kdnand_id_read(
                         ddatabuf.cid,
                         ddatabuf.offset,
                         ddatabuf.pbuf,
                         ddatabuf.size );
            if( rtn == DNAND_NO_ERROR )
            {
                tmprtn    = copy_to_user(p_ubuf, p_kbuf, ddatabuf.size);
            }
            else
            {
                memset(p_kbuf, 0x00, ddatabuf.size);
                tmprtn    = copy_to_user(p_ubuf, p_kbuf, ddatabuf.size);
            }
            if( tmprtn != 0 )
            {
                rtn    = DNAND_PARAM_ERROR;
            }
        }
        break;
        
        default:
        {
            rtn    = DNAND_PARAM_ERROR;
        }
        break;
    }
    
    kfree(p_kbuf);
    return(rtn);
}

static const struct file_operations dnand_cdev_driverfops = {
    .owner      = THIS_MODULE,
    .open       = dnand_cdev_driver_open,
    /* .ioctl      = dnand_cdev_driver_ioctl, */
    .unlocked_ioctl = dnand_cdev_driver_ioctl,
};

static struct miscdevice dnandcdev = {
    .fops       = &dnand_cdev_driverfops,
    .name       = "dnand_cdev",
    .minor      = MISC_DYNAMIC_MINOR,
};

static int __init dnand_cdev_driver_init(void)
{
    printk(KERN_DEBUG "%s: \n", __func__);
    return misc_register(&dnandcdev);
}

static void __exit dnand_cdev_driver_exit(void)
{
    printk(KERN_DEBUG "%s: \n", __func__);
    misc_deregister(&dnandcdev);
}

module_init(dnand_cdev_driver_init);
module_exit(dnand_cdev_driver_exit);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("DNAND CDEV Driver");
MODULE_LICENSE("GPL v2");

#endif
