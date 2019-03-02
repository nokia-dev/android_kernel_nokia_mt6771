/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/input.h>
#include <linux/delay.h>

#ifndef CONFIG_OF
//#include <linux/spi/zfs_spi_common.h>
//#include <linux/spi/zfs_spi_input.h>
#else
#include "zfs_common.h"
#include "zfs_input.h"
#endif


/* -------------------------------------------------------------------- */
/* driver constants							*/
/* -------------------------------------------------------------------- */

#define ZFS_NAVI_LEFT			0x0A
#define ZFS_NAVI_RIGHT			0x0B
#define ZFS_NAVI_UP				0x0C
#define ZFS_NAVI_DOWN			0x0D
#define ZFS_NAVI_SINGLE_CLICK	0x0E
#define ZFS_NAVI_DOUBLE_CLICK	0x0F
#define ZFS_NAVI_LONG_PRESS		0x09


#define ZFS_KEY_UP 			KEY_UP			/* 103 */
#define ZFS_KEY_PAGEUP 		KEY_PAGEUP		/* 104 */
#define ZFS_KEY_LEFT 		KEY_LEFT		/* 105 */
#define ZFS_KEY_RIGHT 		KEY_RIGHT		/* 106 */
#define ZFS_KEY_END 		KEY_END			/* 107 */
#define ZFS_KEY_DOWN 		KEY_DOWN		/* 108 */
#define ZFS_KEY_PAGEDOWN 	KEY_PAGEDOWN	/* 109 */
#define ZFS_KEY_BACK		KEY_BACK  		/* 158 */
#define ZFS_KEY_MENU		KEY_MENU  		/* 139 */
#define ZFS_KEY_HOME		KEY_HOMEPAGE  	/* 172 */
#define ZFS_KEY_WAKEUP		KEY_WAKEUP		/* 143 */

#define ZFS_TOUCH_KEY_EQUAL 	 2
#define ZFS_TOUCH_KEY_TURN_ON  	 1
#define ZFS_TOUCH_KEY_TURN_OFF   0
//#define GET_KEY_STATUS(x,y) (x & (1 << y)) >> y
#define CHECK_KEY(old,new,shift) ( \
	(((old & (1 << shift)) >> shift)!=((new & (1 << shift)) >> shift))?((new & (1 << shift)) >> shift): \
	ZFS_TOUCH_KEY_EQUAL)

static int key_event[] = { 0, 0, 0, ZFS_KEY_HOME, ZFS_KEY_BACK, ZFS_KEY_BACK, ZFS_KEY_MENU, ZFS_KEY_HOME}; //bit0~7

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
int zfs_input_init(zfs_data_t *zfs_dev)
{
	int error = 0;
	int i;

	dev_dbg(&zfs_dev->pdev->dev, "%s\n", __func__);

	zfs_dev->input_dev = input_allocate_device();

	if (!zfs_dev->input_dev) {
		dev_err(&zfs_dev->pdev->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		zfs_dev->input_dev->name = ZFS1096_DEV_NAME;

		__set_bit(EV_KEY, zfs_dev->input_dev->evbit);
		__set_bit(ZFS_KEY_UP, zfs_dev->input_dev->keybit);
		__set_bit(ZFS_KEY_DOWN, zfs_dev->input_dev->keybit);
		__set_bit(ZFS_KEY_LEFT, zfs_dev->input_dev->keybit);
		__set_bit(ZFS_KEY_RIGHT, zfs_dev->input_dev->keybit);

		error = input_register_device(zfs_dev->input_dev);
	}

	if (error) {
		dev_err(&zfs_dev->pdev->dev, "Input_register_device failed.\n");
		input_free_device(zfs_dev->input_dev);
		zfs_dev->input_dev = NULL;
	}
	
	zfs_dev->input.key_status = 0;
	
	for(i=0 ;i<8; i++)
	{
		zfs_dev->key_event[i] = key_event[i];
	}

	return error;
}


/* -------------------------------------------------------------------- */
void zfs_input_destroy(zfs_data_t *zfs_dev)
{
	dev_dbg(&zfs_dev->pdev->dev, "%s\n", __func__);

	if (zfs_dev->input_dev != NULL)
		input_free_device(zfs_dev->input_dev);
}


/* -------------------------------------------------------------------- */
int zfs_input_enable(zfs_data_t *zfs_dev, bool enabled)
{
	dev_dbg(&zfs_dev->pdev->dev, "%s\n", __func__);

	zfs_dev->input.enabled = enabled;

	return 0;
} 

/* -------------------------------------------------------------------- */
int zfs_input_set_event(zfs_data_t *zfs_dev, int key_code, int on_off)
{
	int error = 0;

	dev_dbg(&zfs_dev->pdev->dev, "%s with key code : %02x  %02x\n", __func__,key_code,ZFS_KEY_RIGHT);


	switch(key_code)
	{
		case ZFS_NAVI_LEFT:
			dev_dbg(&zfs_dev->pdev->dev, "%s ZFS_NAVI_LEFT \n", __func__);
			key_code = ZFS_KEY_LEFT;
			break;
		case ZFS_NAVI_RIGHT:
			dev_dbg(&zfs_dev->pdev->dev, "%s ZFS_NAVI_RIGHT \n", __func__);
			key_code = ZFS_KEY_RIGHT;
			break;
		case ZFS_NAVI_UP:
			dev_dbg(&zfs_dev->pdev->dev, "%s ZFS_NAVI_UP \n", __func__);
			key_code = ZFS_KEY_UP;
			break;
		case ZFS_NAVI_DOWN:
			dev_dbg(&zfs_dev->pdev->dev, "%s ZFS_NAVI_DOWN \n", __func__);
			key_code = ZFS_KEY_DOWN;
			break;
		default:
			dev_dbg(&zfs_dev->pdev->dev, "%s unknown cmd code : %02x \n", __func__,key_code);
			break;
	}

	input_report_key(zfs_dev->input_dev, key_code, 1);
	input_sync(zfs_dev->input_dev);
	input_report_key(zfs_dev->input_dev, key_code, 0);
	input_sync(zfs_dev->input_dev);

	return error;
}

/* -------------------------------------------------------------------- */
extern void zfs_input_snyc(zfs_data_t *zfs_dev)
{
	input_sync(zfs_dev->input_dev);
}

#ifdef ZET_CAPTURE_SUPPORT

/* -------------------------------------------------------------------- */
int zfs1096_input_task(zfs_data_t *zfs_dev)
{
	int error = 0;
	u8 key_status[2];

	dev_dbg(&zfs_dev->pdev->dev, "%s\n", __func__);
	
	//error = zfs1096_wait_for_key_irq(zfs_dev, ZFS1096_REG_READ_KEY_INTERRUPT, ZFS1096_STATUS_REG_BIT_KEY);
	error = zfs1096_wait_for_key_irq(zfs_dev, ZFS1096_REG_READ_KEY_INTERRUPT);
	
	if( error < 0 )
	{
		dev_dbg(&zfs_dev->pdev->dev, "[ZET] %s zfs1096_wait_for_key_irq failed.\n", __func__);
		return error;
	}	

	while (!zfs_dev->worker.stop_request /*&& !error*/) {
		
		error = zfs1096_read_key_irq(zfs_dev, 50, false, key_status);
		
		if (error == 0) {
			zfs1096_input_report(zfs_dev, key_status);
		}		
		
	}
	
	//zfs_input_enable(zfs_dev, false);
	return error;
}

int zfs1096_input_report(zfs_data_t *zfs_dev, u8 *key_status)
{
	int error = 0;
	int i;
	int key_state = 0;
	
	if(!zfs_dev->input.enabled)
	{
		zfs_dev->input.key_status = key_status[0];	
		zfs_dev->input.navi_state = key_status[1];	
		return error;
	}		
	
	//KEY
	for(i=0; i<8 /*sizeof(key_event)*/; i++)
	{		
		if(zfs_dev->key_event[i] != 0)
		{
			key_state = CHECK_KEY(zfs_dev->input.key_status,key_status[0],i);
			switch(key_state)
			{
				case ZFS_TOUCH_KEY_EQUAL:
					break;
				case ZFS_TOUCH_KEY_TURN_OFF:
					input_report_key(zfs_dev->input_dev,
								zfs_dev->key_event[i], ZFS_TOUCH_KEY_TURN_OFF);
					break;
				case ZFS_TOUCH_KEY_TURN_ON:
					input_report_key(zfs_dev->input_dev,
								zfs_dev->key_event[i], ZFS_TOUCH_KEY_TURN_ON);					
					break;
				default:
					break;
			}
		}
		
		printk("[ZET] %s pre: %02x now: %02x i: %d res: %02x \n", __func__, zfs_dev->input.key_status, key_status[0], i, key_state);
	}
	
	input_sync(zfs_dev->input_dev);
	
	//NAVI
	switch(key_status[1])
	{
		case ZFS_NAVI_LEFT:
			input_report_key(zfs_dev->input_dev,
				ZFS_KEY_LEFT, ZFS_TOUCH_KEY_TURN_ON);
				
			input_sync(zfs_dev->input_dev);
			
			input_report_key(zfs_dev->input_dev,
				ZFS_KEY_LEFT, ZFS_TOUCH_KEY_TURN_OFF);
				
			input_sync(zfs_dev->input_dev);
				
			break;
		case ZFS_NAVI_RIGHT:
			input_report_key(zfs_dev->input_dev,
				ZFS_KEY_RIGHT, ZFS_TOUCH_KEY_TURN_ON);
				
			input_sync(zfs_dev->input_dev);
			
			input_report_key(zfs_dev->input_dev,
				ZFS_KEY_RIGHT, ZFS_TOUCH_KEY_TURN_OFF);
				
			input_sync(zfs_dev->input_dev);
			
			break;
		default:
			break;
	}
	
	zfs_dev->input.key_status = key_status[0];	
	zfs_dev->input.navi_state = key_status[1];	
	
	return error;
}

#endif /* ZET_CAPTURE_SUPPORT */


