/*
 * Copyright (C) 2017 MediaTek Inc.
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

#ifndef __IMGSENSOR_PROC_H__
#define __IMGSENSOR_PROC_H__

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "imgsensor_common.h"

#define PROC_CAMERA_INFO "driver/camera_info"
#define IMGSENSOR_STATUS_INFO_LENGTH 128
#define camera_info_size 4096

///OEM Start
#define PROC_FSER_INFO "driver/s5k4h7yx_ser_info"

#define PROC_SKBUFFER_INFO "driver/s5k5e9yx_kbuffer_info"
#define PROC_SLSC_INFO "driver/s5k5e9yx_lsc_info"
#define PROC_SWBC_INFO "driver/s5k5e9yx_wbc_info"

#define camera_lsc_info_size 2048
#define camera_wbc_info_size 1024
///OEM  End

extern char mtk_ccm_name[camera_info_size];
///OEM Start
extern char fih_s5k4h7yx_sernum[20];

extern char fih_s5k5e9yx_kbuffer[camera_info_size];
extern char fih_s5k5e9yx_lsc[camera_lsc_info_size];
extern char fih_s5k5e9yx_wbc[1];
///OEM  End

enum IMGSENSOR_RETURN imgsensor_proc_init(void);

extern struct IMGSENSOR gimgsensor;
#endif

