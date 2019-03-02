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
#include <linux/time.h>

#ifndef CONFIG_OF

#else
#include "zfs_common.h"
#include "zfs_capture.h"
#include "zfs_input.h"
#endif

/*MTK header*/
#ifdef FPC_MTK
//#ifdef ZET_REE_SPI_SUPPORT
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif
//#endif
#endif

#ifdef STATIC_SPI_BUF_LENGTH
static u8 spi_rx_buf[STATIC_SPI_BUF_LENGTH] = {0};
#endif

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
int zfs_init_capture(zfs_data_t *zfs_dev)
{
	zfs_dev->capture.state = ZFS_CAPTURE_STATE_IDLE;
	zfs_dev->capture.current_mode = ZFS_MODE_IDLE;
	
	zfs_dev->capture.zfs_sector = 0;
	zfs_dev->capture.read_offset = 0;
	zfs_dev->capture.available_bytes = 0;
	zfs_dev->capture.deferred_finger_up = false;
	zfs_dev->capture.last_error = 0;
	zfs_dev->capture.read_pending_eof = false;

	init_waitqueue_head(&zfs_dev->capture.wq_data_avail);

	return 0;
}

/* -------------------------------------------------------------------- */
bool zfs_capture_check_ready(zfs_data_t *zfs_dev)
{
	zfs_capture_state_t state = zfs_dev->capture.state;

	return (state == ZFS_CAPTURE_STATE_IDLE) ||
		(state == ZFS_CAPTURE_STATE_COMPLETED) ||
		(state == ZFS_CAPTURE_STATE_FAILED);
}

#ifdef ZET_CAPTURE_SUPPORT

/* -------------------------------------------------------------------- */
int zfs1096_capture_task(zfs_data_t *zfs_dev)
{
	// zfs_capture_mode_t mode = zfs_dev->capture.current_mode;
	int error = 0;
	int result = 0;
	struct spi_message msg;
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	u8 cmd1[2] = {0xC0,0x00};
	u8 cmd2[2] = {0xC4,0x00};
	int iBlockLen = 3844;//31*62*2;
	int offset = 0;
	int i = 0;
#ifndef STATIC_SPI_BUF_LENGTH	
	u8 *rx_buf = NULL;
#endif

#ifdef SPI_MAX_LENGTH 
	// recieve multi times
	int rest_byte;
	int recv_byte;
	int count = 0;
	int rx_buf_once = SPI_MAX_LENGTH;
#endif

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = NULL,
		.len    = 2,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
	struct spi_transfer t2 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd2,
#ifdef STATIC_SPI_BUF_LENGTH
		.rx_buf = spi_rx_buf,
#else		
		.rx_buf = rx_buf,
#endif
		.len    = iBlockLen+1,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};	
	
	zfs_dev->capture.state = ZFS_CAPTURE_STATE_FETCH;

#ifdef FPC_MTK	
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER; //FIFO_TRANSFER;
#endif
	
	// do
	// {
		
		down(&zfs_dev->sem_frame);		
		
		memset(zfs_dev->huge_buffer,0x0,zfs_dev->huge_buffer_size);
		
		zfs_dev->capture.available_bytes = 0;
		zfs_dev->capture.read_pending_eof = false;
		zfs_dev->capture.read_offset = 0;
		zfs_dev->capture.zfs_sector = 0;
		offset = 0;
		
		//result = zfs1096_wait_for_irq(zfs_dev, ZFS1096_REG_READ_STATUS, ZFS1096_STATUS_REG_BIT_FINGER, 50);
		result = zfs1096_wait_for_irq(zfs_dev, ZFS1096_REG_READ_STATUS, 50);
		
		if( result != 0 )
		{
			error = -result;
			goto out_error;
		}			
		
		if(zfs_dev->worker.stop_request)
		{
			error = -EINTR;
			goto out_error;
		}
		
#ifndef STATIC_SPI_BUF_LENGTH
		rx_buf = kzalloc(iBlockLen+1, GFP_KERNEL);
#endif			
		
		for(i=0; i<4; i++)
		{
			cmd1[1] = i & 0xFF;
			
			spi_message_init(&msg);
			spi_message_add_tail(&t1,  &msg);  // 0xC0,0x00

			error = spi_sync(zfs_dev->pdev, &msg);

			if (error<0)
			{
				//zfs_dev->capture.last_error = error;
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd1[0], cmd1[1]);
				//break;
				goto out_error;
			}
			
			/*wait_event_interruptible(
				zfs_dev->wq_irq_return,
				zfs_dev->interrupt_done);*/
			result = wait_event_interruptible_timeout(
					zfs_dev->wq_irq_return,
					zfs_dev->interrupt_done, 50);				
				
			zfs_dev->interrupt_done = false;			
			
			if( result <= 0)
			{
				if( result == 0 )
					error = ETIMEDOUT;
				else
					error = -result;
				goto out_error;
			}
			
			if(zfs_dev->worker.stop_request)
			{
				error = -EINTR;
				goto out_error;
			}			
				
			offset = iBlockLen * i;

#ifdef STATIC_SPI_BUF_LENGTH
			memset(spi_rx_buf,0x0,STATIC_SPI_BUF_LENGTH);
#else
			memset(rx_buf,0x0,iBlockLen+1);
#endif
			
#ifndef SPI_MAX_LENGTH // recieve once
			
			spi_message_init(&msg);
			spi_message_add_tail(&t2,  &msg);

			error = spi_sync(zfs_dev->pdev, &msg);
			
			if (error<0)
			{
				//zfs_dev->capture.last_error = error;
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd2[0], cmd2[1]);	
				//break;
				goto out_error;
			}
			
#else // recieve multi times

			rest_byte = iBlockLen+1;
			count=0;

			while(rest_byte > 0)
			{
				if(rest_byte >= rx_buf_once)
					recv_byte = rx_buf_once;
				else
					recv_byte = rest_byte;
				
				t2.len = recv_byte;
				
#ifdef STATIC_SPI_BUF_LENGTH
				t2.rx_buf = spi_rx_buf+count*rx_buf_once;
#else				
				t2.rx_buf = rx_buf+count*rx_buf_once;
#endif
				
				spi_message_init(&msg);
				spi_message_add_tail(&t2,  &msg);

				error = spi_sync(zfs_dev->pdev, &msg);	
				
				if (error<0)
				{
					dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd2[0], cmd2[1]);	
					//break;
					goto out_error;
				}
				
				count++;
				rest_byte -= recv_byte;
			}	
			
			zfs_dev->capture.zfs_sector |= (1 << i);

#endif

#ifdef STATIC_SPI_BUF_LENGTH
			memcpy(zfs_dev->huge_buffer+offset, spi_rx_buf+1, iBlockLen);
#else
			memcpy(zfs_dev->huge_buffer+offset, rx_buf+1, iBlockLen);
#endif
			zfs_dev->capture.available_bytes += iBlockLen;
			
			zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	
		}
		
		zfs_dev->capture.read_pending_eof = true;			
		
		wake_up_interruptible(&zfs_dev->capture.wq_data_avail);
				
	// }while( 0 == zfs1096_wait_for_irq(zfs_dev, ZFS1096_REG_READ_STATUS, ZFS1096_STATUS_REG_BIT_FINGER, 1000) );
	
	// if(error < 0)
		// goto out_error;
	
out_error:
	up(&zfs_dev->sem_frame);
	
	zfs_dev->capture.last_error = error;
	
#ifndef STATIC_SPI_BUF_LENGTH	
	if(rx_buf)
		kfree(rx_buf);
#endif

	if(error) {
		zfs_dev->capture.state = ZFS_CAPTURE_STATE_FAILED;
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s %s %d\n", __func__,
			(error == -EINTR) ? "TERMINATED" : "FAILED", error);
			
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	
	} else {
		zfs_dev->capture.state = ZFS_CAPTURE_STATE_COMPLETED;
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s OK\n", __func__);
	}	
	
	return error;

}

/* -------------------------------------------------------------------- */
int zfs1096_capture_finger_key_navi_task(zfs_data_t *zfs_dev)
{
	// zfs_capture_mode_t mode = zfs_dev->capture.current_mode;
	int error = 0;
	int result = 0;
	struct spi_message msg;
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	u8 cmd1[2] = {0xC1,0x00};
	u8 cmd2[2] = {0xC5,0x00};
	u8 key_buf[2] = {0x00, 0x00};
	int iHeader = 3;
	int iBlockLen = 3844;//31*62*2;
	int offset = 0;
	int i = 0;
#ifndef STATIC_SPI_BUF_LENGTH	
	u8 *rx_buf = NULL;
#endif

#ifdef SPI_MAX_LENGTH 
	// recieve multi times
	int rest_byte;
	int recv_byte;
	int count = 0;
	int rx_buf_once = SPI_MAX_LENGTH;
#endif

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = NULL,
		.len    = 2,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
	struct spi_transfer t2 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd2,
#ifdef STATIC_SPI_BUF_LENGTH
		.rx_buf = spi_rx_buf,
#else		
		.rx_buf = rx_buf,
#endif
		.len    = iBlockLen+iHeader+1,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};	
	
	zfs_dev->capture.state = ZFS_CAPTURE_STATE_FETCH;
	
#ifdef FPC_MTK	
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER; //FIFO_TRANSFER;
#endif
	
	// do
	// {
		
		down(&zfs_dev->sem_frame);		
		
		memset(zfs_dev->huge_buffer,0x0,zfs_dev->huge_buffer_size);
		memset(zfs_dev->zfs1096_image_header,0x0,4);
		
		zfs_dev->capture.available_bytes = 0;
		zfs_dev->capture.read_pending_eof = false;
		zfs_dev->capture.read_offset = 0;
		zfs_dev->capture.zfs_sector = 0;
		offset = 0;
		
#ifndef STATIC_SPI_BUF_LENGTH
		rx_buf = kzalloc(iBlockLen+iHeader+1, GFP_KERNEL);
#endif		
		
		for(i=0; i<4; i++)
		{
			cmd1[1] = i & 0xFF;
			memset(zfs_dev->zfs1096_image_header,0x0,4);
			zfs_dev->zfs1096_image_header[0] = i+1;
			
			spi_message_init(&msg);
			spi_message_add_tail(&t1,  &msg);  // 0xC1,0x00

			error = spi_sync(zfs_dev->pdev, &msg);

			if (error<0)
			{
				// zfs_dev->capture.last_error = error;
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd1[0], cmd1[1]);
				//break;
				goto out_error;
			}
			
			/*wait_event_interruptible(
				zfs_dev->wq_irq_return,
				zfs_dev->interrupt_done);*/
			result = wait_event_interruptible_timeout(
					zfs_dev->wq_irq_return,
					zfs_dev->interrupt_done, 50);				
				
			zfs_dev->interrupt_done = false;			
			
			if( result <= 0)
			{
				if( result == 0 )
					error = ETIMEDOUT;
				else
					error = -result;
				goto out_error;
			}
			
			if(zfs_dev->worker.stop_request)
			{
				error = -EINTR;
				goto out_error;
			}			
				
			offset = iBlockLen * i;

#ifdef STATIC_SPI_BUF_LENGTH
			memset(spi_rx_buf,0x0,STATIC_SPI_BUF_LENGTH);
#else
			memset(rx_buf,0x0,iBlockLen+iHeader+1);
#endif

			/* Header */

			t2.len = iHeader + 1;
			t2.rx_buf = rx_buf;
			
			spi_message_init(&msg);
			spi_message_add_tail(&t2,  &msg);

			error = spi_sync(zfs_dev->pdev, &msg);
			
			if (error<0)
			{
				//zfs_dev->capture.last_error = error;
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd2[0], cmd2[1]);	
				//break;
				goto out_error;
			}
						
			memcpy(zfs_dev->zfs1096_image_header+1, rx_buf+1, 3);
			memcpy(key_buf, rx_buf+2, 2);
			zfs1096_input_report(zfs_dev, key_buf);
			
			if( (rx_buf[1] & 0x01) == 0x0 ) // finger not stable
			{
				error = -EINVAL;
				goto out_error;
			}

			/* image */
			
#ifndef SPI_MAX_LENGTH // recieve once
			
			t2.len = iBlockLen;
			t2.rx_buf = rx_buf + iHeader + 1;
			
			spi_message_init(&msg);
			spi_message_add_tail(&t2,  &msg);

			error = spi_sync(zfs_dev->pdev, &msg);
			
			if (error<0)
			{
				// zfs_dev->capture.last_error = error;
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd2[0], cmd2[1]);	
				// break;
				goto out_error;
			}
			
#else // recieve multi times

			rest_byte = iBlockLen;
			count=0;

			while(rest_byte > 0)
			{
				if(rest_byte >= rx_buf_once)
					recv_byte = rx_buf_once;
				else
					recv_byte = rest_byte;
				
				t2.len = recv_byte;
				
#ifdef STATIC_SPI_BUF_LENGTH
				t2.rx_buf = spi_rx_buf+count*rx_buf_once;
#else				
				t2.rx_buf = rx_buf + iHeader + 1 + count*rx_buf_once;
#endif
				
				spi_message_init(&msg);
				spi_message_add_tail(&t2,  &msg);

				error = spi_sync(zfs_dev->pdev, &msg);	
				
				if (error<0)
				{
					dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, cmd2[0], cmd2[1]);	
					//break;
					goto out_error;
				}
				
				count++;
				rest_byte -= recv_byte;
			}	

#endif

			zfs_dev->capture.zfs_sector |= (1 << i);
			
#ifdef STATIC_SPI_BUF_LENGTH
			memcpy(zfs_dev->huge_buffer+offset, spi_rx_buf+iHeader+1, iBlockLen);
#else
			memcpy(zfs_dev->huge_buffer+offset, rx_buf+iHeader+1, iBlockLen);
#endif
			zfs_dev->capture.available_bytes += iBlockLen;
			
			zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	
		}
		
		zfs_dev->capture.read_pending_eof = true;			
		
		wake_up_interruptible(&zfs_dev->capture.wq_data_avail);
				
	// }while( 0 == zfs1096_wait_for_irq(zfs_dev, ZFS1096_REG_READ_STATUS, ZFS1096_STATUS_REG_BIT_FINGER, 1000) );
	
	// if(error < 0)
		// goto out_error;
	
out_error:
	up(&zfs_dev->sem_frame);
	
	zfs_dev->capture.last_error = error;
	
#ifndef STATIC_SPI_BUF_LENGTH	
	if(rx_buf)
		kfree(rx_buf);
#endif

	if(error) {
		zfs_dev->capture.state = ZFS_CAPTURE_STATE_FAILED;
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s %s %d\n", __func__,
			(error == -EINTR) ? "TERMINATED" : "FAILED", error);
			
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	
	} else {
		zfs_dev->capture.state = ZFS_CAPTURE_STATE_COMPLETED;
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s OK\n", __func__);
	}	
	
	return error;

}

#endif /* ZET_CAPTURE_SUPPORT */