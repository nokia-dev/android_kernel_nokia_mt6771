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
#include "s5k5e9yxmipiraw_eeprom.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define PFX "S5K5E9YXMIPI_EEPROM"

///#define PFX "s5k5e9yxmipi_pdafotp"
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
#define S5K5E9_EEPROM_OFFSET		0x0000

#define CAM_CAL_MAX_BUF_SIZE 65536/*For Safety, Can Be Adjested*/

///#define S5K5E9YXMIPI_I2C_SPEED        100
#define S5K5E9YXMIPI_MAX_OFFSET		0xFFFF
#define DATA_SIZE 2048
#define S5K5E9_AWB_SIZE		8
#define S5K5E9_LSC_SIZE		1868
#define CAM_CAL_DRVNAME "CAM_CAL_DRV_S5K5E9YXMIPI"

static bool read_kbuffer = false;
static bool read_lsc = false;
static bool read_awb = false;

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
static int s5k5e9yxmipiraw_eeprom_read = 0;
typedef union {
        u8 Data[MAX_EEPROM_SIZE];
        EEPROM_MTK_TYPE       MtkEEPromData;
} EEPROM_DATA;

BYTE s5k5e9yxmipi_DCC_data[221] = {0};
BYTE s5k5e9yxmipi_SPC_data[DATA_SIZE] = {0};
EEPROM_DATA s5k5e9yxmipiraw_eeprom_data = {{0}};

static bool get_done;
static int last_size;
static int last_offset;

static bool selective_read_eeprom(kal_uint16 addr, BYTE *data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	if (addr > S5K5E9YXMIPI_MAX_OFFSET)
		return false;
	if (iReadRegI2C(pu_send_cmd, 2, (u8 *) data, 1, S5K5E9YXMIPI_EEPROM_DEVICE_ID) < 0)
		return false;
	return true;
}

static bool _read_s5k5e9yxmipi_eeprom(kal_uint16 addr, BYTE *data, int size)
{
	int i = 0;
	int offset = addr;
  int accum = 0;

	LOG_INF("enter _read_eeprom size = %d\n", size);
	for (i = 0; i < size; i++) {
		if (!selective_read_eeprom(offset, &data[i]))
			return false;

		LOG_INF("read_eeprom 0x%0x %d\n", offset, data[i]);
		if(read_kbuffer == false)
		   snprintf(fih_s5k5e9yx_kbuffer,sizeof(fih_s5k5e9yx_kbuffer),"%s\n0x%0x", fih_s5k5e9yx_kbuffer, data[i]);
		if(read_lsc == false){
			 if((i%17) == 0)
			    snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
			 else{
			 	  accum = accum |(data[i] & 0xff) << 0;
		      snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s%d  ", fih_s5k5e9yx_lsc, accum);
		   }
		 }
		if(read_awb == false){
			    if(i == 0)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nR=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 1)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nGr=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 2)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nGb=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 3)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nB=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 4)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nGolden R=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 5)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nGolden Gr=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 6)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nGolden Gb=0x%0x", fih_s5k5e9yx_wbc, data[i]);
			    else if(i == 7)
			     snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nGolden B=0x%0x", fih_s5k5e9yx_wbc, data[i]);
		 }
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
	read_kbuffer = true;
	read_lsc = true;
	read_awb = true;
	return true;
}


void read_s5k5e9yxmipi_SPC(BYTE *data)
{
	int size = 252;

	LOG_INF("read s5k5e9yxmipi SPC, size = %d\n", size);
	/**********************************************************
	 * if(!get_done || last_size != size || last_offset != addr) {
	 * if(!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_SPC_data, size)){
	 * get_done = 0;
	 * last_size = 0;
	 * last_offset = 0;
	 * return false;
	 * }
	 * }
	 **********************************************************/
	memcpy(data, s5k5e9yxmipi_SPC_data, size);
}


void read_s5k5e9yxmipi_DCC(kal_uint16 addr, BYTE *data, kal_uint32 size)
{

	addr = S5K5E9_EEPROM_OFFSET;
	size = 96;

	LOG_INF("read s5k5e9yxmipi DCC, size = %d\n", size);

	if (!get_done || last_size != size || last_offset != addr) {
		if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_SPC_data, size)) {
			get_done = 0;
			last_size = 0;
			last_offset = 0;

		}
	}

	memcpy(data, s5k5e9yxmipi_DCC_data, size);

}

int read_s5k5e9yxmipiraw_eeprom_size(u8 slave_id, u16 offset, u8* data,int size)
{
	int i = 0;

    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]read_s5k5e9yxmipiraw_eeprom_size start,offset=0x%x, size=0x%x\n", offset, size);
	for(i = 0; i < size; i++){
		if(selective_read_eeprom(offset+i, data+i) != 0)
        {
            CAM_CALERR("[s5k5e9yxmipiraw_eeprom]read s5k5e9yxmipiraw_eeprom i2c fail !!!\n");
            return -1;
        }
		///CAM_CALDB("[s5k5e9yxmipiraw_eeprom]s5k5e9yxmipiraw_eeprom_data.Data[0x%x] = 0x%x \n",offset+i,*(data+i));
	}
    ///CAM_CALDB("[s5k5e9yxmipiraw_eeprom]s5k5e9yxmipiraw_eeprom_data end\n");
	return 0;
}

int read_s5k5e9yxmipiraw_eeprom_data(void){

	kal_uint16 addr = S5K5E9_EEPROM_OFFSET;

    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]read_s5k5e9yxmipiraw_eeprom_data start!!!\n");
    memset(s5k5e9yxmipiraw_eeprom_data.Data,0,sizeof(EEPROM_MTK_TYPE));
    if (read_s5k5e9yxmipiraw_eeprom_size(S5K5E9YXMIPI_EEPROM_DEVICE_ID, addr, s5k5e9yxmipiraw_eeprom_data.Data, sizeof(EEPROM_MTK_TYPE)))
    {
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]read_s5k5e9yxmipiraw_eeprom_size fail !!!\n");
        return -1;
    }
    if( 1 != s5k5e9yxmipiraw_eeprom_data.Data[0] )
	{
	    CAM_CALERR("[s5k5e9yxmipiraw_eeprom]eeprom data flag error: 0x%x !!!\n", s5k5e9yxmipiraw_eeprom_data.Data[0]);
	    memset(s5k5e9yxmipiraw_eeprom_data.Data,0,sizeof(EEPROM_MTK_TYPE));
	    return -1;
	}
    spin_lock(&g_CAM_CALLock);
    s5k5e9yxmipiraw_eeprom_read = 1;
    spin_unlock(&g_CAM_CALLock);
    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]read_s5k5e9yxmipiraw_eeprom_data end!!!\n");

	return 0;

}

u8 S5K5E9_ReadOtp(kal_uint16 address)
{
	u8 readbuff;
	int ret ;

	CAM_CALDB("[S5K5E9_CAL]ENTER address:0x%x \n ",address);
	ret= selective_read_eeprom(address, &readbuff);
	return readbuff;
}

kal_uint8 S5K5E9_Read_Flag(void)
{
	kal_uint8 otp_flag;

	otp_flag = S5K5E9_ReadOtp(S5K5E9_EEPROM_OFFSET);

	if(otp_flag == 0x01) {
		CAM_CALDB("[S5K5E9_CAL] EEPROM Flag is one.\n");
		return KAL_TRUE;
	} else {
		CAM_CALERR("[S5K5E9_CAL] EEPROM is null\n");
		return KAL_FALSE;
	}
}

static kal_uint8 S5K5E9_Read_AWB_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	int i = 0;
	u8 checksum =0;
	u8 datasum = 0;
	u8 temp;
	kal_uint16 AWBaddr = 0x0013;
	kal_uint8 otp_flag;

	otp_flag = S5K5E9_Read_Flag();

	if(buffersize != S5K5E9_AWB_SIZE)
	{
		CAM_CALDB("[S5K3L8_CAL]wrong AWBSIZE:0x%x \n ",buffersize);
		return KAL_FALSE;
	}

	if(otp_flag == 0x01) {
		for(i = 0;i < buffersize;i++)
		{
			iBuffer[i] = S5K5E9_ReadOtp(i + AWBaddr);
			datasum += iBuffer[i];
		}
		temp = S5K5E9_ReadOtp(0x0562);

		checksum = (temp & 0xFF);

		if(checksum == (datasum % 255)){
			CAM_CALDB("[S5K5E9_CAL] AWB checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K5E9_CAL] AWB checksum Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K5E9_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}

static kal_uint8 S5K5E9_Read_LSC_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	int i = 0;
	kal_uint16 LSCaddr = 0x001C;

	u8 checksum =0;
	u8 datasum = 0;
	u8 temp;
	kal_uint8 otp_flag;

	otp_flag = S5K5E9_Read_Flag();
	if(buffersize != S5K5E9_LSC_SIZE)
	{
		CAM_CALDB("[S5K5E9_CAL]wrong LSCSIZE:0x%x\n ",buffersize);
		return KAL_FALSE;
	}

	if(otp_flag == 0x01) {
		for(i = 0;i < buffersize;i++)
		{
			iBuffer[i] = S5K5E9_ReadOtp(i + LSCaddr);
			datasum += iBuffer[i];
		}

		temp = S5K5E9_ReadOtp(0x0D0F);

		checksum = (temp & 0xFF);

		if(checksum == (datasum % 255)){
			CAM_CALDB("[S5K5E9_CAL] LSC checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K5E9_CAL] LSC checksum Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K5E9_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}

int iReadData_5E9(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	if(ui4_length ==1)
	{
		S5K5E9_ReadOtp(ui4_offset);
	}
	else if(ui4_length == S5K5E9_AWB_SIZE)
	{
		S5K5E9_Read_AWB_Otp(ui4_offset, pinputdata, ui4_length);
	}
	else if(ui4_length == S5K5E9_LSC_SIZE)
	{
		S5K5E9_Read_LSC_Otp(ui4_offset, pinputdata, ui4_length);
	}

	CAM_CALDB(" [S5K5E9_CAL]ui4_length = %d,ui4_offset =%d\n ",ui4_length,ui4_offset);
	return 0;
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

static long s5k5e9yxmipiraweeprom_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
	struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	struct stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[s5k5e9yxmipiraw_eeprom] s5k5e9yxmipiraw_eeprom_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;
    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]s5k5e9yxmipiraweeprom_Ioctl_Compat, cmd:0x%x, COMPAT_CAM_CALIOC_G_READ:0x%x, COMPAT_CAM_CALIOC_S_WRITE:0x%x\n", 
	           (unsigned int)cmd, (unsigned int)COMPAT_CAM_CALIOC_G_READ, (unsigned int)COMPAT_CAM_CALIOC_S_WRITE); 
    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        CAM_CALDB("[s5k5e9yxmipiraw_eeprom]COMPAT Read CMD \n");
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
            CAM_CALERR("[s5k5e9yxmipiraw_eeprom] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


static int selective_read_region(u32 offset, BYTE* data, u32 size)
{
    if( !s5k5e9yxmipiraw_eeprom_read )
    {
        CAM_CALDB("[s5k5e9yxmipiraw_eeprom]selective_read_region first read data from eeprom!!!\n");
        if( 0 != read_s5k5e9yxmipiraw_eeprom_data() )
        {
            CAM_CALDB("[s5k5e9yxmipiraw_eeprom]selective_read_region failed!!!\n");
            return 0;
        }
    }
    if(size > sizeof(EEPROM_MTK_TYPE))
    {
        CAM_CALDB("[s5k5e9yxmipiraw_eeprom]selective_read_region read size 0x%x is too big! change to max size:0x%x\n", 
                  size, (u32)sizeof(EEPROM_MTK_TYPE));
        size = sizeof(EEPROM_MTK_TYPE);
    }
    memcpy((u8*)data,s5k5e9yxmipiraw_eeprom_data.Data+offset,size);
	CAM_CALDB("[s5k5e9yxmipiraw_eeprom]selective_read_region offset =%x size %d \n", offset,size);
	//int i = 0;
    //for(i = 0; i < size; i++){
    //    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]selective_read_region s5k5e9yxmipiraw_eeprom_data.Data[%d]=0x%x, data[%d]=0x%x \n", 
    //              offset+i, s5k5e9yxmipiraw_eeprom_data.Data[offset+i], i, data[i]);
	//}
    return size;
}

void update_s5k5e9yx_eeprom_awb_data(void)
{
	kal_uint16 addr = 0x0013;   ///AWB information
	kal_uint32 size = 1;   ///AWB size 8

	/* Add info to proc: update_otp_buffer_data */
	if(read_awb == false){
	   snprintf(fih_s5k5e9yx_wbc,sizeof(fih_s5k5e9yx_wbc),"%s\nWhitebalance data dump for EEPROM s5k5e9yxmipiraw", fih_s5k5e9yx_wbc);

	  if (!get_done || last_size != size || last_offset != addr) {
		  if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_DCC_data, size)) {
			  get_done = 0;
			  last_size = 0;
			  last_offset = 0;
		  }
	  }
  }
}

void update_s5k5e9yx_eeprom_lsc_data(void)
{
	kal_uint16 addr = 0x001C;
	kal_uint32 size = 1868;   ///LSC size 1868
  kal_uint32 ep_table_size = 221;
  int i;
	/* Add info to proc: update_otp_buffer_data */
	if(read_lsc == false){
		 snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\nLens shading correction data dump for EEPROM s5k5e9yxmipiraw", fih_s5k5e9yx_lsc);
		 for(i=0;i<4;i++){
		 if(i == 0) {
		 snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n************Light type: TL84_LIGHT ************", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\nmesh_rolloff_table_size = 221", fih_s5k5e9yx_lsc);
	   }
	   else if(i == 1) {
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n************Light type: A_LIGHT ************", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\nmesh_rolloff_table_size = 221", fih_s5k5e9yx_lsc);
	   }
	   else if(i == 2) {
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n************Light type: D65_LIGHT ************", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\nmesh_rolloff_table_size = 221", fih_s5k5e9yx_lsc);
	   }
	   else if(i == 3) {
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n************Light type: H_LIGHT ************", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\nmesh_rolloff_table_size = 221", fih_s5k5e9yx_lsc);
	   }

	   ///r_gain
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%sr_gain\n", fih_s5k5e9yx_lsc);
	  if (!get_done || last_size != size || last_offset != addr) {
		  if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_DCC_data, ep_table_size)) {
			  get_done = 0;
			  last_size = 0;
			  last_offset = 0;
		  }
	  }
	   /// gr_gain
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%sgr_gain\n", fih_s5k5e9yx_lsc);
	  if (!get_done || last_size != size || last_offset != addr) {
		  if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_DCC_data, ep_table_size)) {
			  get_done = 0;
			  last_size = 0;
			  last_offset = 0;
		  }
	  }
      ///gb_gain
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%sgb_gain\n", fih_s5k5e9yx_lsc);
	  if (!get_done || last_size != size || last_offset != addr) {
		  if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_DCC_data, ep_table_size)) {
			  get_done = 0;
			  last_size = 0;
			  last_offset = 0;
		  }
	  }
      ///b_gain
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%s\n", fih_s5k5e9yx_lsc);
	   snprintf(fih_s5k5e9yx_lsc,sizeof(fih_s5k5e9yx_lsc),"%sb_gain\n", fih_s5k5e9yx_lsc);
	  if (!get_done || last_size != size || last_offset != addr) {
		  if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_DCC_data, ep_table_size)) {
			  get_done = 0;
			  last_size = 0;
			  last_offset = 0;
		  }
	  }
   }/// for end

  }
}

void update_s5k5e9yx_eeprom_kbuffer_data(void)
{
	kal_uint16 addr = S5K5E9_EEPROM_OFFSET;
	kal_uint32 size = DATA_SIZE;
	/* Add info to proc: update_otp_buffer_data */
	if(read_kbuffer == false){
	   snprintf(fih_s5k5e9yx_kbuffer,sizeof(fih_s5k5e9yx_kbuffer),"%s\nKernel buffer data dump:", fih_s5k5e9yx_kbuffer);
	   snprintf(fih_s5k5e9yx_kbuffer,sizeof(fih_s5k5e9yx_kbuffer),"%s\neeprom_name: s5k5e9yxmipiraw", fih_s5k5e9yx_kbuffer);
	   snprintf(fih_s5k5e9yx_kbuffer,sizeof(fih_s5k5e9yx_kbuffer),"%s\nNumber of bytes: %d", fih_s5k5e9yx_kbuffer, size);

	  if (!get_done || last_size != size || last_offset != addr) {
		  if (!_read_s5k5e9yxmipi_eeprom(addr, s5k5e9yxmipi_SPC_data, size)) {
			  get_done = 0;
			  last_size = 0;
			  last_offset = 0;
		  }
	  }
  }
}

bool update_s5k5e9yx_eeprom(void)
{
	int result = 1;
	update_s5k5e9yx_eeprom_kbuffer_data();
	update_s5k5e9yx_eeprom_lsc_data();
	update_s5k5e9yx_eeprom_awb_data();
	return  result;

}

int s5k5e9yxmipi_read_eeprom(u16 offset, u8* data, u32 size)
{
    return selective_read_region(offset, data, size);
}

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
            CAM_CALERR("[s5k5e9yxmipiraw_eeprom]ioctl allocate mem failed\n");
            return -ENOMEM;
        }

		if (copy_from_user((u8 *) pBuff, (u8 *) a_u4Param, sizeof(struct stCAM_CAL_INFO_STRUCT))) {
			/*get input structure address*/
			kfree(pBuff);
                CAM_CALERR("[s5k5e9yxmipiraw_eeprom]ioctl copy from user failed\n");
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
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]ioctl allocate mem failed\n");
        return -ENOMEM;
    }

		if (copy_from_user((u8 *)pu1Params, (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pu1Params);
            CAM_CALERR("[s5k5e9yxmipiraw_eeprom]ioctl copy from user failed\n");
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
            CAM_CALDB("[s5k5e9yxmipiraw_eeprom]Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("[s5k5e9yxmipiraw_eeprom]Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[s5k5e9yxmipiraw_eeprom]Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
#ifdef CAM_CALGETDLT_DEBUG   ///Old TSP Z2
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, ptempbuf->u4Length);
#else
            i4RetValue = iReadData_5E9((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#endif

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("[s5k5e9yxmipiraw_eeprom]Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

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
            CAM_CALERR("[s5k5e9yxmipiraw_eeprom]ioctl copy to user failed\n");
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
    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("[s5k5e9yxmipiraw_eeprom]Opened, return -EBUSY\n");
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
    .compat_ioctl = s5k5e9yxmipiraweeprom_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
//#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALDRV_S5K5E9YXMIPIRAW");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

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
    CAM_CALDB("[s5k5e9yxmipiraw_eeprom]CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB("[s5k5e9yxmipiraw_eeprom]register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB("[s5k5e9yxmipiraw_eeprom]Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]failed to register s5k5e9yxmipiraw_eeprom driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("[s5k5e9yxmipiraw_eeprom]failed to register s5k5e9yxmipiraw_eeprom driver, 2nd time\n");
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

MODULE_DESCRIPTION("CAM_CAL_S5K5E9YXMIPIRAW driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");