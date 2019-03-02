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

#ifndef __S5K5E9YXMIPI_EEPROM_H
#define __S5K5E9YXMIPI_EEPROM_H

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define S5K5E9YXMIPI_EEPROM_DEVICE_ID	    0xA2 /// ID Checksum ///0xA2

#define S5K5E9YXMIPI_EEPROM_INFO_SIZE     18
#define S5K5E9YXMIPI_EEPROM_AWB_SIZE      8
#define S5K5E9YXMIPI_EEPROM_VCM_SIZE      4
#define S5K5E9YXMIPI_EEPROM_LSC_SIZE      1868
#define S5K5E9YXMIPI_EEPROM_PDAF_SIZE     1404

#define S5K5E9YXMIPI_EEPROM_INFO_OFFSET   0x00
#define S5K5E9YXMIPI_EEPROM_AWB_OFFSET    0x12
#define S5K5E9YXMIPI_EEPROM_VCM_OFFSET    0x1A
#define S5K5E9YXMIPI_EEPROM_LSC_OFFSET    0x20
#define S5K5E9YXMIPI_EEPROM_PDAF_OFFSET   0x801

#define MAX_LSC_SIZE 1868+148
#define MAX_PDAF_SIZE 2048
#define MAX_EEPROM_SIZE 4*1024

typedef struct {
    u8     Flag;
	u8     Year;//0x01
	u8     Month;//0x01
	u8     Day;//0x01
	u8     Mid;//0x01
	u8     LensId;//0x01
	u8     VCMId;//0x01
	u8     AfId;//0x01
	u8     ColorTempture;//0x01
	u8     Reserved;//0x01
	u8     CaliVer[4];//0xff000b01
	u8     SerialNum[2];
	u8     Version;//0x01
	u8     AwbAfInfo;//0xF
	u8     UnitAwbR;
	u8     UnitAwbGr;
	u8     UnitAwbGb;
	u8     UnitAwbB;
	u8     GoldenAwbR;
	u8     GoldenAwbGr;
	u8     GoldenAwbGb;
	u8     GoldenAwbB;
	u8     AfInfinite[2];
	u8     AfMacro[2];
	u8     LscSize[2];
	u8     Lsc[MAX_LSC_SIZE];
	u8     PDInfo;//0x7
	u8     Pdaf[MAX_PDAF_SIZE];
}EEPROM_MTK_TYPE;

int s5k5e9yxmipi_read_eeprom(u16 offset, u8* data, u32 size);

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId, u8 number);

#endif
