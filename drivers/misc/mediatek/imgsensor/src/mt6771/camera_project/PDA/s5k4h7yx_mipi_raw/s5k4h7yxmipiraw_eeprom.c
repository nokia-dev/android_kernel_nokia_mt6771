/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"
#include "../../../../cam_cal/inc/cam_cal.h"
#include "../../../../cam_cal/inc/cam_cal_define.h"
#include "s5k4h7yxmipiraw_eeprom.h"
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k4h7yxmipiraw_Sensor.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define PFX "S5K4H7MIPI_EEPROM"

///#define PFX "s5k4h7mipi_pdafotp"
#define LOG_INF(format, args...)		pr_debug(PFX "[%s] " format, __func__, ##args)
//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALINF(format, args...)    pr_info(PFX "[%s] " format, __func__, ##args)
#define CAM_CALDB(format, args...)     pr_debug(PFX "[%s] " format, __func__, ##args)
#define CAM_CALERR(format, args...)    printk(KERN_ERR format, ##args)
#else
#define CAM_CALINF(x,...)
#define CAM_CALDB(x,...)
#define CAM_CALERR(x, ...)
#endif

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "imgsensor_proc.h"

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP

#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)
#define S5K4H7_EEPROM_OFFSET		0x0000

#define CAM_CAL_MAX_BUF_SIZE 65536/*For Safety, Can Be Adjested*/

///#define S5K4H7MIPI_I2C_SPEED        100
#define S5K4H7MIPI_MAX_OFFSET		0xFFFF
#define DATA_SIZE 2048
#define S5K4H7_AWB_SIZE		8
#define S5K4H7_LSC_SIZE		1868
#define CAM_CAL_DRVNAME "CAM_CAL_DRV_S5K4H7MIPI"

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
typedef union {
        u8 Data[MAX_EEPROM_SIZE];
        EEPROM_MTK_TYPE       MtkEEPromData;
} EEPROM_DATA;

BYTE s5k4h7mipi_DCC_data[221] = {0};
BYTE s5k4h7mipi_SPC_data[DATA_SIZE] = {0};
EEPROM_DATA s5k4h7mipiraw_eeprom_data = {{0}};

static u8 *eeprom_4h7_data;
static u8 *eeprom_4h7_data_LSC;
static u32 g_u4Opened;
u8 eeprom_valid_flag_4h7;

static void get_4h7_lsc_data(void)
{
	unsigned short get_byte=0;
	unsigned int addr = 0x0A04;
	unsigned int Page = 0x00;
	int i = 0;
	int j = 0;
	int k = 0;

	Page = 0x17;//LSC Start Page S5K4H7 Spec
	for(j = 0; j < 29; j++){
		  /*Choose EEPROM LSC Page*/
		  otp_4h7_write_cmos_sensor_8(0x0A00,0x03);
			otp_4h7_write_cmos_sensor_8(0x0A02,Page);
			get_byte = otp_4h7_read_cmos_sensor(0x0A02);
			//printk("S5K4H7 Choose EEPROM Page = %d\n",get_byte);
			otp_4h7_write_cmos_sensor_8(0x0A00,0x01);

			do
			{
				mdelay(1);
				get_byte = otp_4h7_read_cmos_sensor(0x0A01);
			}while((get_byte & 0x01) != 1);

			addr = 0x0A04;
			for(k = 0; k < 64; k++){
				eeprom_4h7_data_LSC[i] = otp_4h7_read_cmos_sensor(addr);
				//printk("S5K4H7 Read i = %d addr = 0x%x and LSCData = 0x%x\n",i ,addr ,eeprom_4h7_data_LSC[i]);
				addr++;
				i++;
			}
			Page++;
	}

	otp_4h7_write_cmos_sensor_8(0x0A00,0x03);
	otp_4h7_write_cmos_sensor_8(0x0A02,Page);
	get_byte = otp_4h7_read_cmos_sensor(0x0A02);
	//printk("S5K4H7 Choose EEPROM Page = %d\n",get_byte);
	otp_4h7_write_cmos_sensor_8(0x0A00,0x01);

	do
	{
		mdelay(1);
		get_byte = otp_4h7_read_cmos_sensor(0x0A01);
	}while((get_byte & 0x01) != 1);

	/*The last page data*/
	addr = 0x0A04;
	for(j = 0; j < 16; j++){
		eeprom_4h7_data_LSC[i] = otp_4h7_read_cmos_sensor(addr);
		//printk("S5K4H7 Read i = %d addr = 0x%x and LSCData = 0x%x\n",i ,addr ,eeprom_4h7_data_LSC[i]);
		addr++;
		i++;
	}
	otp_4h7_write_cmos_sensor_8(0x0A00,0x00);
}

static void get_4h7_page_data(void)
{
	unsigned short get_byte=0;
	unsigned int addr = 0x0A04;
	unsigned int Page = 0x00;
	int i = 0;
	int j = 0;

	Page = 0x15; //Module and AWB Page
	//printk("S5K4H7 Choose EEPROM Page Start\n");
	otp_4h7_write_cmos_sensor_8(0x0A00,0x03);
	otp_4h7_write_cmos_sensor_8(0x0A02,Page);
	get_byte = otp_4h7_read_cmos_sensor(0x0A02);
	//printk("S5K4H7 Choose EEPROM Page = %d\n",get_byte);
	otp_4h7_write_cmos_sensor_8(0x0A00,0x01);

	do
	{
		mdelay(1);
		get_byte = otp_4h7_read_cmos_sensor(0x0A01);
	}while((get_byte & 0x01) != 1);

	/*For Module and AWB information*/
	for(j = 0; j < 35; j++){
		eeprom_4h7_data[i] = otp_4h7_read_cmos_sensor(addr);
		printk("ChunJeng Read addr = 0x%x and get_byte = 0x%x\n", addr, eeprom_4h7_data[i]);
		//printk("S5K4H7 Read i = %d addr = 0x%x and ModuleData = 0x%x\n",i ,addr ,eeprom_4h7_data[i]);

		addr++;
		i++;
	}
	otp_4h7_write_cmos_sensor_8(0x0A00,0x00);
}

static int cam_cal_read(u32 addr, u32 size, u8 *data)
{
    if (addr == 0x00000001)//MTK check ID
    {
        memcpy(data, eeprom_4h7_data + 10, size);
        printk("S5K4H7 cam cal read Read MTK Tag = 0x%x\n",data[0]);
        printk("S5K4H7 cam cal read Read MTK Tag = 0x%x\n",data[1]);
        printk("S5K4H7 cam cal read Read MTK Tag = 0x%x\n",data[2]);
        printk("S5K4H7 cam cal read Read MTK Tag = 0x%x\n",data[3]);
        return 4;
    }

    if (addr == 0x00000002)//LSC EEPROM
    {
    	  get_4h7_lsc_data();
        memcpy(data, eeprom_4h7_data_LSC + 3, size);
        printk("S5K4H7 cam_cal_read Read LSC EEPROM\n");
        printk("S5K4H7 cam cal read Read LSC[0] = 0x%x\n",data[0]);
        printk("S5K4H7 cam cal read Read LSC[1] = 0x%x\n",data[1]);
        printk("S5K4H7 cam cal read Read LSC[2] = 0x%x\n",data[2]);
        printk("S5K4H7 cam cal read Read LSC[3] = 0x%x\n",data[3]);
        return 4;
    }

    if (addr == 0x00000003)//AWB Unit EEPROM
    {
        memcpy(data, eeprom_4h7_data + 26, 4);
        printk("S5K4H7 cam_cal_read Read AWB Unit EEPROM\n");
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[0]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[1]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[2]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[3]);
        return 4;
    }

    if (addr == 0x00000004)//AWB Golden EEPROM
    {
        memcpy(data, eeprom_4h7_data + 30, 4);
        printk("S5K4H7 cam_cal_read Read AWB Golden EEPROM\n");
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[0]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[1]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[2]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[3]);
        return 4;
    }

    if (addr == 0x00000005)//Group1 EEPROM page21 page 23~52
    {
        get_4h7_lsc_data();
        memcpy(data, eeprom_4h7_data , 35);
        memcpy(data+35, eeprom_4h7_data_LSC, 1873);
        printk("S5K4H7 cam_cal_read Read AWB Golden EEPROM\n");
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[0]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[1]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[2]);
        printk("S5K4H7 cam_cal_read Read = 0x%x\n",data[3]);
        return 4;
    }
	return 0;
}

void CAM_CAL_GetOtpData_4h7(void)
{
	int i;
	get_4h7_page_data();
	for(i =0; i < 5; i++)
	snprintf(fih_s5k4h7yx_sernum,sizeof(fih_s5k4h7yx_sernum),"%s%02X", fih_s5k4h7yx_sernum, eeprom_4h7_data[20 + i]);
	fih_s5k4h7yx_sernum[5] = '\0';
}


#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
	struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	struct stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);

	/* Assume pointer is not change */
#if 1
	err |= get_user(p, (compat_uptr_t *)&data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
	struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
	struct stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long s5k4h7mipiraweeprom_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
	struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	struct stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[s5k4h7mipiraw_eeprom] s5k4h7mipiraw_eeprom_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;
    CAM_CALDB("[s5k4h7mipiraw_eeprom]s5k4h7mipiraweeprom_Ioctl_Compat, cmd:0x%x, COMPAT_CAM_CALIOC_G_READ:0x%x, COMPAT_CAM_CALIOC_S_WRITE:0x%x\n", 
	           (unsigned int)cmd, (unsigned int)COMPAT_CAM_CALIOC_G_READ, (unsigned int)COMPAT_CAM_CALIOC_S_WRITE); 
    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        CAM_CALDB("[s5k4h7mipiraw_eeprom]COMPAT Read CMD \n");
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[s5k4h7mipiraw_eeprom] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}

#endif

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    struct stCAM_CAL_INFO_STRUCT *ptempbuf = NULL;
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

	if (_IOC_DIR(a_u4Command) != _IOC_NONE) {
		pBuff = kmalloc(sizeof(struct stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (pBuff == NULL) {
            CAM_CALERR("[s5k4h7mipiraw_eeprom]ioctl allocate mem failed\n");
            return -ENOMEM;
        }

		if (copy_from_user((u8 *) pBuff, (u8 *) a_u4Param, sizeof(struct stCAM_CAL_INFO_STRUCT))) {
			/*get input structure address*/
			kfree(pBuff);
                CAM_CALERR("[s5k4h7mipiraw_eeprom]ioctl copy from user failed\n");
			return -EFAULT;
		}

		ptempbuf = (struct stCAM_CAL_INFO_STRUCT *)pBuff;

		if ((ptempbuf->u4Length <= 0) || (ptempbuf->u4Length > CAM_CAL_MAX_BUF_SIZE)) {
			kfree(pBuff);
			PK_DBG("Buffer Length Error!\n");
			return -EFAULT;
		}

		pu1Params = kmalloc(ptempbuf->u4Length, GFP_KERNEL);

		if (pu1Params == NULL) {
			kfree(pBuff);
        CAM_CALERR("[s5k4h7mipiraw_eeprom]ioctl allocate mem failed\n");
        return -ENOMEM;
    }

		if (copy_from_user((u8 *)pu1Params, (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pu1Params);
            CAM_CALERR("[s5k4h7mipiraw_eeprom]ioctl copy from user failed\n");
        return -EFAULT;
		}
	}
	if (ptempbuf == NULL) { /*It have to add */
		PK_DBG("ptempbuf is Null !!!");
		return -EFAULT;
	}

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:

            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[CAM_CAL] Read CMD length %d\n", ptempbuf->u4Length);
						i4RetValue = cam_cal_read(ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
            break;
        default :
      	     CAM_CALINF("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("[s5k4h7mipiraw_eeprom]ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}

static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[s5k4h7mipiraw_eeprom]CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("[s5k4h7mipiraw_eeprom]Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = s5k4h7mipiraweeprom_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
//#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("[s5k4h7mipiraw_eeprom]RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("[s5k4h7mipiraw_eeprom]Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("[s5k4h7mipiraw_eeprom]Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR("[s5k4h7mipiraw_eeprom]Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR("[s5k4h7mipiraw_eeprom]Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALDRV_S5K4H7MIPIRAW");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("[s5k4h7mipiraw_eeprom]Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    eeprom_4h7_data = kmalloc(35, GFP_KERNEL);
    if (NULL == eeprom_4h7_data) {
        CAM_CALDB(" allocate eeprom mem failed\n");
        return -ENOMEM;
    }
    eeprom_4h7_data_LSC = kmalloc(1873, GFP_KERNEL);
    if (NULL == eeprom_4h7_data_LSC) {
        CAM_CALDB(" allocate eeprom mem failed\n");
        return -ENOMEM;
    }

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
    CAM_CALDB("[s5k4h7mipiraw_eeprom]CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB("[s5k4h7mipiraw_eeprom]register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB("[s5k4h7mipiraw_eeprom]Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("[s5k4h7mipiraw_eeprom]failed to register s5k4h7mipiraw_eeprom driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("[s5k4h7mipiraw_eeprom]failed to register s5k4h7mipiraw_eeprom driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

MODULE_DESCRIPTION("CAM_CAL_S5K4H7MIPIRAW driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
