/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_ZFS_SPI_COMMON_H
#define LINUX_ZFS_SPI_COMMON_H

#define DEBUG

#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/netlink.h>

#ifndef CONFIG_OF

#else
#include "zfs.h"
#include "zfs_regs.h"
#endif

#ifdef ZET_SPI_DRIVER
#include <linux/spi/spi.h>
#endif

//#define STATIC_SPI_BUF_LENGTH 8192
#define SPI_MAX_LENGTH 1024 //8192
//#define SPI_CS_MANUAL

/* -------------------------------------------------------------------- */
/* zfs1096 driver constants						*/
/* -------------------------------------------------------------------- */
extern const bool target_little_endian;

/* set '0' for dynamic assignment, or '> 0' for static assignment */
#define ZFS1096_MAJOR				0

#define ZFS1096_DEV_NAME                        "zfs1096"

#define ZFS1096_RESET_RETRIES			2
#define ZFS1096_RESET_LOW_US			5
#define ZFS1096_RESET_HIGH1_US			5
#define ZFS1096_RESET_HIGH2_MS			50
#define ZFS1096_BUFFER_MAX_IMAGES		3

#define MAX_FLASH_BUF_SIZE			(0x10000)

/* -------------------------------------------------------------------- */
/* zfs1096 data types							*/
/* -------------------------------------------------------------------- */

typedef enum {
	ZFS_CAPTURE_STATE_IDLE = 0,
	ZFS_CAPTURE_STATE_STARTED,
	ZFS_CAPTURE_STATE_PENDING,
	ZFS_CAPTURE_STATE_WRITE_SETTINGS,
	ZFS_CAPTURE_STATE_WAIT_FOR_FINGER_DOWN,
	ZFS_CAPTURE_STATE_ACQUIRE,
	ZFS_CAPTURE_STATE_FETCH,
	ZFS_CAPTURE_STATE_WAIT_FOR_FINGER_UP,
	ZFS_CAPTURE_STATE_COMPLETED,
	ZFS_CAPTURE_STATE_FAILED,
} zfs_capture_state_t;

typedef enum {
	ZFS1096_IRQ_EXIT   		= 0,
	ZFS1096_IRQ_IGNORE      = 1
} zfs1096_irq_reg_t;

/*
typedef enum {
	ZFS1096_STATUS_REG_BIT_FINGER			= 1 << 0,
	ZFS1096_STATUS_REG_BIT_KEY				= 1 << 1,
	ZFS1096_STATUS_REG_BIT_NAVIGATION		= 1 << 2,
} zfs1096_scan_reg_t;
*/

#define ZFS1096_STATUS_REG_BIT_FINGER 			1
#define ZFS1096_STATUS_REG_BIT_KEY    			2
#define ZFS1096_STATUS_REG_BIT_NAVIGATION		4

#define ZFS1096_REMOVE_REG_BIT(x,y)	( x & ( y ^ 0xFF ) )
#define ZFS1096_REMOVE_REG_BIT_FINGER(x)	( x & 0xFE )
#define ZFS1096_REMOVE_REG_BIT_KEY(x)		( x & 0xFD )
#define ZFS1096_REMOVE_REG_BIT_NAVI(x)		( x & 0xFB )

typedef enum {
	ZFS1096_KEY_BIT0	= 1 << 0,
	ZFS1096_KEY_BIT1	= 1 << 1,
	ZFS1096_KEY_BIT2	= 1 << 2,
	ZFS1096_KEY_BIT3	= 1 << 3,
	ZFS1096_KEY_BIT4	= 1 << 4,
	ZFS1096_KEY_BIT5	= 1 << 5,
	ZFS1096_KEY_BIT6	= 1 << 6,
	ZFS1096_KEY_BIT7	= 1 << 7,
} zfs1096_key_reg_t;

typedef enum {
	ZFS1096_SECTOR_BIT0	= 1 << 0,
	ZFS1096_SECTOR_BIT1	= 1 << 1,
	ZFS1096_SECTOR_BIT2	= 1 << 2,
	ZFS1096_SECTOR_BIT3	= 1 << 3,
	ZFS1096_SECTOR_BIT4	= 1 << 4,
	ZFS1096_SECTOR_BIT5	= 1 << 5,
	ZFS1096_SECTOR_BIT6	= 1 << 6,
	ZFS1096_SECTOR_BIT7	= 1 << 7,
} zfs1096_sector_t;

typedef enum {
	ZFS1096_NONE	= 0,
	ZFS1096_SRAM	= 1,
	ZFS1096_FLASH	= 2,
} zfs1096_flashtype_t;

typedef enum {
	FRAMEWORK_NONE	 = 0,
	FRAMEWORK_IMAGE	 = 1,
	FRAMEWORK_ENROLL = 2,
	FRAMEWORK_VERIFY = 3,
} framework_msg_t;

/* -------------------------------------------------------------------- */
/* zfs1096 data types							*/
/* -------------------------------------------------------------------- */

typedef struct zfs_chip_info {
	zfs_chip_t		       type;
	u8                     revision;
	u8                     pixel_rows;
	u8                     pixel_columns;
	u8                     adc_group_size;
	u16                    spi_max_khz;
	u8					   data_bits;
	u32 				   flash_size;	
} zfs_chip_info_t;

typedef struct zfs_worker_struct {
	struct task_struct 		*thread;
	struct semaphore 		sem_idle;
	wait_queue_head_t 		wq_wait_job;
	int 					req_mode;
	bool 					stop_request;
} zfs_worker_t;

typedef struct zfs_capture_struct {
	zfs_capture_mode_t		current_mode;
	zfs_capture_state_t		state;
	u32						read_offset;
	u32						available_bytes;
	wait_queue_head_t		wq_data_avail;
	int						last_error;
	bool					read_pending_eof;
	bool					deferred_finger_up;
	u8  					zfs_sector;
} zfs_capture_task_t;

typedef struct zfs_input_struct {
	bool enabled;
	u8 key_status;
	u8 navi_state;
} zfs_input_task_t;

typedef struct zfs_setup {
	u8 capture_count;
} zfs_setup_t;

typedef struct zfs_diag {
	const char *chip_id;	/* RO */
	//u8  selftest;		/* RO */
	//u16 spi_register;	/* RW */
	//u8  spi_regsize;	/* RO */
	//u8  spi_data;		/* RW */
	//u16 last_capture_time;	/* RO*/
	//u16 finger_present_status;	/* RO*/
} zfs_diag_t;

typedef struct {
	
#ifdef ZET_SPI_DRIVER	
	struct spi_device       *pdev;//*spi;
#else
	struct platform_device  *pdev;
#endif

	struct class           *class;
	struct device          *device;
	struct cdev            cdev;
	dev_t                  devno;
	zfs_chip_info_t		   chip;
	u32                    cs_gpio;
	u32                    reset_gpio;
	u32                    irq_gpio;
	int                    irq;
	int 				   irq_count; //// FIH BJ C2 MT6755
	wait_queue_head_t      wq_irq_return;
	bool                   interrupt_done;
	struct semaphore       mutex;
	u8                     *huge_buffer;
	size_t                 huge_buffer_size;
	zfs_worker_t       	   worker;
	zfs_capture_task_t 	   capture;
	zfs_setup_t        	   setup;
	zfs_diag_t         	   diag;
/*	bool                   soft_reset_enabled;
	struct regulator       *vcc_spi;
	struct regulator       *vdd_ana;
	struct regulator       *vdd_io;
	struct regulator       *vdd_tx;
	bool                   power_enabled;
	int                    vddtx_mv;
	bool                   txout_boost;
	u16                    force_hwid;
	bool                   use_regulator_for_bezel;*/
	u16                    spi_freq_khz;
	struct input_dev       *input_dev;
	zfs_input_task_t   	   input;
	u8					   scan_mode;
	int 				   key_event[8];

	struct workqueue_struct *zfs_int_workqueue;
	struct work_struct     zfs_int_work;
	struct semaphore       sem_frame;
	u8                     *flash_buffer;
	bool  				   flash_update;
	u8					   flash_type;
	
	u8  				   zfs1096_fw_info[30]; //17+1
	//u8                     *zfs1096_fw_info;
	u8					   zfs1096_image_header[4];	
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#else
	struct notifier_block notifier;
#endif	
	
} zfs_data_t; //zfs_spi_data_t; 

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */

#define UNUSED(x) (void)(x)

extern size_t zfs_calc_huge_buffer_minsize(zfs_data_t *zfs_dev);

extern int zfs_manage_huge_buffer(zfs_data_t *zfs_dev, size_t new_size);

extern int zfs_check_hw_id(zfs_data_t *zfs_dev);

extern int zfs_setup_defaults(zfs_data_t *zfs_dev);

extern bool zfs_check_in_range_u64(u64 val, u64 min, u64 max);

extern const char *zfs_hw_id_text(zfs_data_t *zfs_dev);

#ifdef ZET_REE_SPI_SUPPORT

extern int zfs_reg_write(zfs_data_t *zfs_dev, int tx_len, int rx_len, u8 *tx_buf, u8 *rx_buf);

extern int zfs_reg_read(zfs_data_t *zfs_dev, int tx_len, int rx_len, u8 *tx_buf, u8 *rx_buf);

extern int zfs_reg_read_sector(zfs_data_t *zfs_dev, int tx_len, int rx_len, u8 *tx_buf, u8 *rx_buf);

extern int zfs1096_mcu_reset(zfs_data_t *zfs_dev); //move to SPI mode

extern int zfs1096_download_task(zfs_data_t *zfs_dev); //move to SPI mode

#endif /* ZET_REE_SPI_SUPPORT */

#ifdef ZET_CAPTURE_SUPPORT

extern int zfs1096_clear_int(zfs_data_t *zfs_dev, int code);

//extern int zfs1096_wait_for_irq(zfs_data_t *zfs_dev, u8 scan_cmd, u8 scan_mode, int timeout);
extern int zfs1096_wait_for_irq(zfs_data_t *zfs_dev, u8 scan_cmd, int timeout);

//extern int zfs1096_wait_for_key_irq(zfs_data_t *zfs_dev, u8 scan_cmd, u8 scan_mode);
extern int zfs1096_wait_for_key_irq(zfs_data_t *zfs_dev, u8 scan_cmd);

extern int zfs1096_read_irq(zfs_data_t *zfs_dev, bool clear_int, u8 _cmd, u8 *_buf);

extern int zfs1096_read_key_irq(zfs_data_t *zfs_dev, int timeout, bool timeout_clear_int, u8 *key_buf);

//extern int zfs1096_mcu_reset(zfs_data_t *zfs_dev); //move to SPI mode

//extern int zfs1096_download_task(zfs_data_t *zfs_dev); //move to SPI mode

extern int zfs1096_read_hw_info(zfs_data_t *zfs_dev);

#endif /* ZET_CAPTURE_SUPPORT */

#endif /* LINUX_ZFS_SPI_COMMON_H */

