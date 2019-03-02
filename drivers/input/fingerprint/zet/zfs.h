/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_ZFS_SPI_H
#define LINUX_ZFS_SPI_H

#define FPC_MTK
//#define CS_MANUAL
//#define ZET_REE_SPI_SUPPORT
#ifdef ZET_REE_SPI_SUPPORT
#define ZET_BOOTLOADER
//#define ZET_CAPTURE_SUPPORT
#endif

#define ZET_SPI_DRIVER

struct zfs_platform_data {
	int irq_gpio;
	int reset_gpio;
	int cs_gpio;
	//int external_supply_mv;
	//int txout_boost;
	//int force_hwid;
	//int use_regulator_for_bezel;
};

typedef enum {
	ZFS_MODE_IDLE						= 0,
	ZFS_MODE_WAIT_AND_CAPTURE			= 1,
	ZFS_MODE_SINGLE_CAPTURE				= 2,
	ZFS_MODE_CHECKERBOARD_TEST_NORM		= 3,
	ZFS_MODE_CHECKERBOARD_TEST_INV		= 4,
	ZFS_MODE_BOARD_TEST_ONE				= 5,
	ZFS_MODE_BOARD_TEST_ZERO			= 6,
	ZFS_MODE_WAIT_FINGER_DOWN			= 7,
	ZFS_MODE_WAIT_FINGER_UP				= 8,
	ZFS_MODE_SINGLE_CAPTURE_CAL			= 9,
	ZFS_MODE_CAPTURE_AND_WAIT_FINGER_UP	= 10,
} zfs_capture_mode_t;

typedef enum {
	CHIP_NONE  = 0,
	CHIP_ZFS1096 = 1,
	CHIP_ZFS1097 = 2,	
} zfs_chip_t;

#endif

