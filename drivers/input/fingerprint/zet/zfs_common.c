/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/clk.h>

#ifndef CONFIG_OF

#else
#include "zfs_common.h"
#include "zfs_regs.h"
#endif

#ifdef FPC_MTK
#include <linux/clk.h>
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

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	const bool target_little_endian = true;
#else
	#warning BE target not tested!
	const bool target_little_endian = false;
#endif

/* -------------------------------------------------------------------- */
/* data types							*/
/* -------------------------------------------------------------------- */
struct chip_struct {
	zfs_chip_t type;
	u16 hwid;
	u8  revision;
	u8  pixel_rows;
	u8  pixel_columns;
	u8  adc_group_size;
	u16 spi_max_khz;
	u8  data_bits;
	u32 flash_size;
};

/* -------------------------------------------------------------------- */
/* driver constants						*/
/* -------------------------------------------------------------------- */
#define ZFS1096_ROWS		124u
#define ZFS1096_COLUMNS		62u

#define ZFS10xx_ADC_GROUP_SIZE	4u

static const char * const chip_text[] = {
	"N/A", /* CHIP_NONE */
	"zfs1096", /* CHIP_ZFS1096 */
	"zfs1097", /* CHIP_ZFS1097 */	
};

static const struct chip_struct chip_data[] = {
	{CHIP_ZFS1097, 0x020a, 0, 89, 62, ZFS10xx_ADC_GROUP_SIZE, 8000,  16,  28704}, // data size = 28k +32
	{CHIP_ZFS1096, 0x020b, 0, ZFS1096_ROWS, ZFS1096_COLUMNS, ZFS10xx_ADC_GROUP_SIZE, 8000,  16,  0x8000},
	{   CHIP_NONE, 0, 0, 0, 0, 0, 0, 0}
};

/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
size_t zfs_calc_huge_buffer_minsize(zfs_data_t *zfs_dev)
{
	const size_t buff_min = ZFS1096_ROWS * ZFS1096_COLUMNS * 2;
	return buff_min;
}

/* -------------------------------------------------------------------- */
int zfs_manage_huge_buffer(zfs_data_t *zfs_dev, size_t new_size)
{
	int error = 0;
	int buffer_order_new, buffer_order_curr;

	buffer_order_curr = get_order(zfs_dev->huge_buffer_size);
	buffer_order_new  = get_order(new_size);

	if (new_size == 0) {
		if (zfs_dev->huge_buffer) {
			free_pages((unsigned long)zfs_dev->huge_buffer,
							buffer_order_curr);

			zfs_dev->huge_buffer = NULL;
		}
		zfs_dev->huge_buffer_size = 0;
		error = 0;

	} else {
		if (zfs_dev->huge_buffer &&
			(buffer_order_curr != buffer_order_new)) {

			free_pages((unsigned long)zfs_dev->huge_buffer,
							buffer_order_curr);

			zfs_dev->huge_buffer = NULL;
		}

		if (zfs_dev->huge_buffer == NULL) {
			zfs_dev->huge_buffer =
				(u8 *)__get_free_pages(GFP_KERNEL,
							buffer_order_new);

			zfs_dev->huge_buffer_size = (zfs_dev->huge_buffer) ?
				(size_t)PAGE_SIZE << buffer_order_new : 0;

			error = (zfs_dev->huge_buffer_size == 0) ? -ENOMEM : 0;
		}
	}

	if (error) {
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s, failed %d\n",
							__func__, error);
	} else {
		dev_info(&zfs_dev->pdev->dev, "[ZET] %s, size=%d bytes\n",
					__func__, (int)zfs_dev->huge_buffer_size);
	}

	return error;
}

/* -------------------------------------------------------------------- */
int zfs_check_hw_id(zfs_data_t *zfs_dev)
{
	int error = 0;
	u16 hardware_id;
	int counter = 0;
	bool match = false;

	// zfs_dev->force_hwid = 0x020a;
	
	//zfs1096_read_hw_info(zfs_dev);

	hardware_id = 0x020a;
	//hardware_id |= 0x8000;  /* Mark it as none bezel */
	//hardware_id &= 0xFFF0;  /* Ignore manufacture */

	while (!match && chip_data[counter].type != CHIP_NONE) {
		if (chip_data[counter].hwid == hardware_id)
			match = true;
		else
			counter++;
	}

	pr_err("[ZET] match hardware_id is %x\n", hardware_id);

	if (match) {
		zfs_dev->chip.type     = chip_data[counter].type;
		zfs_dev->chip.revision = chip_data[counter].revision;

		zfs_dev->chip.pixel_rows     = chip_data[counter].pixel_rows;
		zfs_dev->chip.pixel_columns  = chip_data[counter].pixel_columns;
		zfs_dev->chip.adc_group_size = chip_data[counter].adc_group_size;
		zfs_dev->chip.spi_max_khz    = chip_data[counter].spi_max_khz;
		zfs_dev->chip.data_bits      = chip_data[counter].data_bits;
		zfs_dev->chip.flash_size      = chip_data[counter].flash_size;

		dev_err(&zfs_dev->pdev->dev,
				"[ZET] Hardware id: 0x%x (%s, rev.%d)\n", hardware_id,
						chip_text[zfs_dev->chip.type],
						zfs_dev->chip.revision);
	} else {
		dev_err(&zfs_dev->pdev->dev,
			"[ZET] Hardware id mismatch: got 0x%x\n", hardware_id);

		zfs_dev->chip.type = CHIP_NONE;
		zfs_dev->chip.revision = 0;

		return -EIO;
	}

	return error;
}

/* -------------------------------------------------------------------- */
int zfs_setup_defaults(zfs_data_t *zfs_dev)
{
	int error = 0;
	return error;
}

/* -------------------------------------------------------------------- */
bool zfs_check_in_range_u64(u64 val, u64 min, u64 max)
{
	return (val >= min) && (val <= max);
}

/* -------------------------------------------------------------------- */
const char *zfs_hw_id_text(zfs_data_t *zfs_dev)
{
	pr_err("zfs_hw_id_text revision is %s\n", chip_text[zfs_dev->chip.type]);
	return chip_text[zfs_dev->chip.type];
}

#ifdef ZET_REE_SPI_SUPPORT

/* -------------------------------------------------------------------- */
int zfs_reg_write(zfs_data_t *zfs_dev, int tx_len, int rx_len, u8 *tx_buf, u8 *rx_buf)
{
	int error = 0;
	struct spi_message msg;
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len    = 0,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
#ifdef FPC_MTK	
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	t1.tx_buf = tx_buf;
	t1.len = tx_len;
	
	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);		
	}	

	return error;
}

/* -------------------------------------------------------------------- */
int zfs_reg_read(zfs_data_t *zfs_dev, int tx_len, int rx_len, u8 *tx_buf, u8 *rx_buf)
{
	int error = 0;
	struct spi_message msg;
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len    = 0,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	t1.tx_buf = tx_buf;
	t1.rx_buf = rx_buf;
	t1.len = rx_len;
	
	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);		
	}	

	return error;
}

/* -------------------------------------------------------------------- */
int zfs_reg_read_sector(zfs_data_t *zfs_dev, int tx_len, int rx_len, u8 *tx_buf, u8 *rx_buf)
{
	int error = 0;
	struct spi_message msg;
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	int iHeader = 3;
	int offset = 0;
	int iBlockLen = 0;//3844;//31*62*2;
	int sector = 0;
	int rest_byte;
	int recv_byte;
	int count = 0;
	int rx_buf_once = SPI_MAX_LENGTH;	
	int img_size = 0;
	int i;

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len    = 0,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	if(tx_buf[0] != ZFS1096_REG_READ_FINGER_STABLE && tx_buf[0] != ZFS1096_REG_READ_KEY_STABLE)
	{
		zfs_dev->capture.state = ZFS_CAPTURE_STATE_FAILED;
		zfs_dev->capture.read_pending_eof = false;
		zfs_dev->capture.available_bytes = 0;			
		
		error = -EFAULT; /* Bad address */
		goto out_error;
	}
	
	zfs_dev->interrupt_done = false;			
	
	iBlockLen = rx_len - (iHeader + 1);
	
	sector = tx_buf[1];
	offset = iBlockLen * sector;	
	tx_buf[1] = 0;

	t1.tx_buf = tx_buf;
	t1.rx_buf = rx_buf;
	t1.len = iHeader + 1;
	
	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) header failed.\n", __func__);		
		zfs_dev->capture.state = ZFS_CAPTURE_STATE_FAILED;
		zfs_dev->capture.read_pending_eof = false;
		zfs_dev->capture.available_bytes = 0;	
		goto out_error;
	}	
	
	if(tx_buf[0] == ZFS1096_REG_READ_FINGER_STABLE)
	{
		rx_buf[4] = 0; 
		
		if( (rx_buf[1] & 0x01) == 0x0 ) // finger not stable
		{
			zfs_dev->capture.state = ZFS_CAPTURE_STATE_FAILED;
			zfs_dev->capture.read_pending_eof = false;
			zfs_dev->capture.available_bytes = 0;			
			
			error = -EINVAL; /* Invalid argument */
			goto out_error;
		}		

		rest_byte = iBlockLen;
		count=0;

		while(rest_byte > 0)
		{
			if(rest_byte >= rx_buf_once)
				recv_byte = rx_buf_once;
			else
				recv_byte = rest_byte;
			
			t1.len = recv_byte;				
		
			t1.rx_buf = zfs_dev->huge_buffer + offset + count*rx_buf_once;
			
			spi_message_init(&msg);
			spi_message_add_tail(&t1,  &msg);

			error = spi_sync(zfs_dev->pdev, &msg);	
			
			if (error<0)
			{
				zfs_dev->capture.state = ZFS_CAPTURE_STATE_FAILED;
				zfs_dev->capture.read_pending_eof = false;
				zfs_dev->capture.available_bytes = 0;			
				
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) image failed %d.\n", __func__, sector);		
				//break;
				goto out_error;
			}
			
			count++;
			rest_byte -= recv_byte;
		}	
		
		rx_buf[4] = 1; 
		
		zfs_dev->capture.zfs_sector |= (1 << sector);
		
		for(i=0; i<8; i++)
		{
			if(zfs_dev->capture.zfs_sector & (1 << i))
				zfs_dev->capture.available_bytes += iBlockLen;
		}
		
		img_size = zfs_calc_huge_buffer_minsize(zfs_dev);
		
		if(zfs_dev->capture.available_bytes >= img_size)
		{
			zfs_dev->capture.state = ZFS_CAPTURE_STATE_COMPLETED;
			zfs_dev->capture.read_pending_eof = true;
			zfs_dev->capture.available_bytes = img_size;			
		}else
		{
			zfs_dev->capture.read_pending_eof = false;	
		}		
		
	}
	
out_error:
	return error;
}

/* -------------------------------------------------------------------- */
int zfs1096_mcu_reset(zfs_data_t *zfs_dev)
{
	int error = 0;
	struct spi_message msg;
	u8 *read_buffer = NULL;
	u8 cmd1[ZFS1096_REG_SIZE(ZFS1096_REG_MCU_RESET)];
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	
	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = &read_buffer,
		.len    = ZFS1096_REG_SIZE(ZFS1096_REG_MCU_RESET),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
	cmd1[0] = ZFS1096_REG_MCU_RESET;
	cmd1[1] = 0x85;	
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s failed.\n", __func__);	
	}
	
	return error;
}				

/* -------------------------------------------------------------------- */
int zfs1096_download_task(zfs_data_t *zfs_dev)
{
	// zfs_capture_mode_t mode = zfs_dev->capture.current_mode;
	int error = 0;
	struct spi_message msg;
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	int iPageLen = 256;
	int iFlashRestLen = 0;
	int iFlashWriteLen = 0;	
	int iFlashPageIndex = 0;
	int i = 0;
	u8 *tx_buf = NULL;
	u8 *rx_buf = NULL;
	int status_count;
	int status_count_max = 1000;

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len    = iPageLen+4,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
#ifdef FPC_MTK	
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER; //FIFO_TRANSFER;
#endif

	if(zfs_dev->flash_buffer == NULL)
	{
		goto out_error;
	}
	
	zfs1096_mcu_reset(zfs_dev);
	
	tx_buf = kzalloc(iPageLen+4, GFP_KERNEL);	
	//memset(tx_buf,0x00,iPageLen+4);	
	rx_buf = kzalloc(iPageLen+4, GFP_KERNEL);	
	
	t1.tx_buf = tx_buf;
	t1.rx_buf = rx_buf;	
	
	if(zfs_dev->flash_type == ZFS1096_NONE)
	{		
		//assume flash mode
		tx_buf[0] = 0xE4;
		tx_buf[1] = 0x96; //pwd
		tx_buf[2] = 0x00;
		tx_buf[3] = 0x00;
		//rx_buf = NULL;
		
		t1.len = 2;
		
		//t1.tx_buf = tx_buf;
		//t1.rx_buf = rx_buf;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0xE4,0x96
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, tx_buf[0], tx_buf[1]);
			goto exit_and_reset;
		}			
		
		//flash testing 
		tx_buf[0] = 0x9F;
		tx_buf[1] = 0x00;
		tx_buf[2] = 0x00;
		tx_buf[3] = 0x00;
		//rx_buf = NULL;
		
		t1.len = 2;
		
		//t1.tx_buf = tx_buf;
		//t1.rx_buf = rx_buf;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0x9F 
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, tx_buf[0], tx_buf[1]);
			goto exit_and_reset;
		}			
		
		if(rx_buf[1] == 0x51)
		{
			zfs_dev->flash_type = ZFS1096_FLASH;
		}else
		{
			zfs_dev->flash_type = ZFS1096_SRAM;
			
			//sram mode
			tx_buf[0] = 0xE4;
			tx_buf[1] = 0x55; //pwd
			tx_buf[2] = 0x00;
			tx_buf[3] = 0x00;
			//rx_buf = NULL;
			
			t1.len = 2;
			
			t1.tx_buf = tx_buf;
			t1.rx_buf = rx_buf;
			
			spi_message_init(&msg);
			spi_message_add_tail(&t1,  &msg);  // 0xE4,0x55
			
			error = spi_sync(zfs_dev->pdev, &msg);

			if (error<0)
			{
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, tx_buf[0], tx_buf[1]);
				goto exit_and_reset;
			}						
		}			
		
	}else
	{
		if(zfs_dev->flash_type == ZFS1096_SRAM)
		{
			tx_buf[0] = 0xE4;
			tx_buf[1] = 0x55; //pwd
			tx_buf[2] = 0x00;
			tx_buf[3] = 0x00;
			
		}else
		{
			tx_buf[0] = 0xE4;
			tx_buf[1] = 0x96; //pwd
			tx_buf[2] = 0x00;
			tx_buf[3] = 0x00;
		}
		
		t1.len = 2;
		
		t1.tx_buf = tx_buf;
		t1.rx_buf = rx_buf;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0xE4, ..
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. %02X %02X\n", __func__, tx_buf[0], tx_buf[1]);
			goto exit_and_reset;
		}	
	}
	
	if(zfs_dev->flash_type == ZFS1096_SRAM)
		goto download;
	
	//Erase 0x20 0x00~0x20 0x70
	for(i=0; i<7; i++) // 8 or 7 sectors
	{
		tx_buf[0] = 0x06;
		tx_buf[1] = 0x00;
		tx_buf[2] = 0x00;
		tx_buf[3] = 0x00;
		//rx_buf = NULL;
		
		t1.len = 1;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0x06
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. sector = %d %02X %02X\n",
				__func__, i, tx_buf[0], tx_buf[1]);
			goto exit_and_reset;
		}	
		
		/////////////////
		tx_buf[0] = 0x20;
		tx_buf[1] = 0x00;
		tx_buf[2] = i*16;
		tx_buf[3] = 0x00;
		//rx_buf = NULL;
		
		t1.len = 4;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0x02 0x00 0x00~0x06(7) 0x00
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s Erase timeout failed. sector = %d\n", 
				__func__, i);
			goto exit_and_reset;
		}			
		
		if(0)
		{
			mdelay(175);
		}else
		{
			status_count = 0;
			do
			{
				if(status_count>status_count_max)
				{
					dev_err(&zfs_dev->pdev->dev, "[ZET] %s erase timeout. sector = %d %02X %02X %02X %02X\n",
						__func__, i, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
					goto exit_and_reset;
				}else
				{
					status_count++;
				}

				tx_buf[0] = 0x05;
				tx_buf[1] = 0x00;
				tx_buf[2] = 0x00;
				tx_buf[3] = 0x00;
				
				t1.len = 2;
				
				spi_message_init(&msg);
				spi_message_add_tail(&t1,  &msg);  // 0x05
				
				error = spi_sync(zfs_dev->pdev, &msg);

				if (error<0)
				{
					dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. sector = %d %02X %02X\n",
						__func__, i, tx_buf[0], tx_buf[1]);
					goto exit_and_reset;
				}	
				
			}while(rx_buf[1] & 0x01); //WIP bit

		}
		
	}
	
download:	
	
	//iFlashRestLen = zfs_dev->chip.flash_size;
	
	if(zfs_dev->flash_type == ZFS1096_SRAM)
	{
		iFlashRestLen = 4096 * 7 + 32; //zfs_dev->chip.flash_size; //28k+32bytes
	}
	else
	{
		iFlashRestLen = 4096 * 7; //zfs_dev->chip.flash_size - 32; //sector0~7
	}

	pr_err("[ZET] zfs1096_download_task - iFlashRestLen:%d \n",iFlashRestLen);
	while(iFlashRestLen>0)
	{
		
		tx_buf[0] = 0x06;
		tx_buf[1] = 0x00;
		tx_buf[2] = 0x00;
		tx_buf[3] = 0x00;
		//rx_buf = NULL;
		
		t1.len = 1;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0x06
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. page = %d %02X %02X\n", 
				__func__, iFlashPageIndex, tx_buf[0], tx_buf[1]);
			goto exit_and_reset;
		}	
		
		///////////////////
		if(iFlashRestLen>=iPageLen)
			iFlashWriteLen = iPageLen;
		else
			iFlashWriteLen = iFlashRestLen;		
		memcpy((tx_buf + 4), zfs_dev->flash_buffer+(iFlashPageIndex*iPageLen), iFlashWriteLen);
		
		tx_buf[0] = 0x02;
		tx_buf[1] = 0x00;
		tx_buf[2] = iFlashPageIndex & 0xFF;
		tx_buf[3] = 0x00;
		//rx_buf = NULL;
		
		t1.len = iPageLen+4;
		
		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);  // 0x06
		
		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. page = %d %02X %02X\n", 
				__func__, iFlashPageIndex, tx_buf[0], tx_buf[1]);
			goto exit_and_reset;
		}
		
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s write page %02X %02X %02X %02X\n", 
			__func__, tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
		
		if(zfs_dev->flash_type == ZFS1096_FLASH)
		{
			if(0)
			{
				mdelay(75);
			}else
			{
				status_count = 0;
				do
				{
					if(status_count>status_count_max)
					{
						dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. sector = %d %02X %02X\n",
							__func__, i, tx_buf[0], tx_buf[1]);
						goto exit_and_reset;
					}else
					{
						status_count++;
					}

					tx_buf[0] = 0x05;
					tx_buf[1] = 0x00;
					tx_buf[2] = 0x00;
					tx_buf[3] = 0x00;
					
					t1.len = 1;
					
					spi_message_init(&msg);
					spi_message_add_tail(&t1,  &msg);  // 0x06
					
					error = spi_sync(zfs_dev->pdev, &msg);

					if (error<0)
					{
						dev_err(&zfs_dev->pdev->dev, "[ZET] %s spi_sync failed. sector = %d %02X %02X\n",
							__func__, i, tx_buf[0], tx_buf[1]);
						goto exit_and_reset;
					}	
					
				}while(rx_buf[1] & 0x01); //WIP bit

			}	
		}	
		
		iFlashPageIndex++;
		iFlashRestLen -= iFlashWriteLen;
		
	}		

	zfs_dev->flash_update = true;

exit_and_reset:		
	
	zfs1096_mcu_reset(zfs_dev);
	
	mdelay(ZFS1096_RESET_HIGH2_MS);
	
out_error:
	
	if(tx_buf)
		kfree(tx_buf);
	
	if(rx_buf)
		kfree(rx_buf);
	
	if(zfs_dev->flash_buffer)
	{
		kfree(zfs_dev->flash_buffer);
		zfs_dev->flash_buffer = NULL;
	}
	
	return error;

}

#endif /* ZET_REE_SPI_SUPPORT */

#ifdef ZET_CAPTURE_SUPPORT

/* -------------------------------------------------------------------- */
int zfs1096_clear_int(zfs_data_t *zfs_dev, int code)
{
	int error = 0;
	struct spi_message msg;
	u8 read_buffer[ZFS1096_REG_MAX_SIZE];
	//u8 cmd1[4] = {ZFS1096_REG_READ_INTERRUPT_WITH_CLEAR,0x00,0x00,0x00};
	u8 cmd1[ZFS1096_REG_SIZE(ZFS1096_REG_READ_INTERRUPT_WITH_CLEAR)];
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	
	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = &read_buffer,
		.len    = ZFS1096_REG_SIZE(ZFS1096_REG_READ_INTERRUPT_WITH_CLEAR),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
	cmd1[0] = ZFS1096_REG_READ_INTERRUPT_WITH_CLEAR;
	cmd1[1] = code;	
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(%d) failed.\n", __func__, code);	
	}else
	{
		zfs_dev->interrupt_done = false;
	}
	
	return error;
}				

/* -------------------------------------------------------------------- */
int zfs1096_wait_for_irq(zfs_data_t *zfs_dev,/* fpc1020_data_t *fpc1020 */ u8 scan_cmd, int timeout)
{
	int error = 0;
	struct spi_message msg;
	u8 read_buffer[ZFS1096_REG_MAX_SIZE];
	u8 cmd1[ZFS1096_REG_MAX_SIZE];
	u8 cmd2[ZFS1096_REG_SIZE(ZFS1096_REG_READ_STABLE)];
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	int result = 0;

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = NULL,
		.len    = ZFS1096_REG_SIZE(scan_cmd),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
	struct spi_transfer t2 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd2,
		.rx_buf = &read_buffer,
		.len    = ZFS1096_REG_SIZE(ZFS1096_REG_READ_STABLE),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};			
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	memset(read_buffer,0x0,ZFS1096_REG_MAX_SIZE);
	memset(cmd1,0x0,ZFS1096_REG_MAX_SIZE);
	memset(cmd2,0x0,ZFS1096_REG_SIZE(ZFS1096_REG_READ_STABLE));

	cmd1[0] = scan_cmd;
	cmd1[1] = zfs_dev->scan_mode;//scan_mode & 0xFF;
	
	cmd2[0] = ZFS1096_REG_READ_STABLE;	
	
	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);		
		return error;
	}
	
	error = result;
	
	if( scan_cmd == ZFS1096_REG_READ_STATUS )
	{
		if (!timeout) {
			result = wait_event_interruptible(
					zfs_dev->wq_irq_return,
					zfs_dev->interrupt_done);
		} else {
			result = wait_event_interruptible_timeout(
					zfs_dev->wq_irq_return,
					zfs_dev->interrupt_done, timeout);
		}		
		
		if (result < 0) {
			dev_err(&zfs_dev->pdev->dev,
				 "[ZET] wait_event_interruptible interrupted by signal.\n");
				 
			zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	 
				 
			return result;
		}

		if (result || !timeout) {
			zfs_dev->interrupt_done = false;		

			spi_message_init(&msg);
			spi_message_add_tail(&t2,  &msg);

			error = spi_sync(zfs_dev->pdev, &msg);

			if (error<0)
			{
				dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t2) failed.\n", __func__);	
				return error;
			}		
				
			zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	
			
			if(read_buffer[1] != 1)
				return -ETIMEDOUT;  //not stable
			else
				return 0;  //stable

		}
		
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	
		
		return -ETIMEDOUT;
	}

	return error;
}		

/* -------------------------------------------------------------------- */
//int zfs1096_wait_for_key_irq(zfs_data_t *zfs_dev, u8 scan_cmd, u8 scan_mode)
int zfs1096_wait_for_key_irq(zfs_data_t *zfs_dev, u8 scan_cmd)
{
	int error = 0;
	struct spi_message msg;
	// u8 read_buffer[ZFS1096_REG_MAX_SIZE];
	u8 cmd1[4];
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	//int result = 0;

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = NULL,
		.len    = ZFS1096_REG_SIZE(scan_cmd),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};	
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	// memset(read_buffer,0x0,ZFS1096_REG_MAX_SIZE);
	memset(cmd1,0x0,4);
	
	cmd1[0] = scan_cmd;
	cmd1[1] = ZFS1096_REMOVE_REG_BIT(zfs_dev->scan_mode, ZFS1096_STATUS_REG_BIT_FINGER);//scan_mode && 0xFF;
	
	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);
	
	dev_err(&zfs_dev->pdev->dev, "[ZET] %s %02x %02x.\n", __func__, cmd1[0], cmd1[1]);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);		
	}

	return error;
}		

/* -------------------------------------------------------------------- */
int zfs1096_read_irq(zfs_data_t *zfs_dev, bool clear_int, u8 _cmd, u8 *_buf)
{
	int error = 0;
	struct spi_message msg;
	u8 tx_cmd[4] = {0};

#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	
	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &tx_cmd,
		.rx_buf = _buf,
		.len    = ZFS1096_REG_SIZE(_cmd),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};			
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	tx_cmd[0] = _cmd;

	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);		
		return error;
	}
	
	if(clear_int)
	{
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);
	}
	
	return error;
}

/* -------------------------------------------------------------------- */
int zfs1096_read_key_irq(zfs_data_t *zfs_dev, int timeout, bool timeout_clear_int, u8 *key_buf)
{
	int error = 0;
	struct spi_message msg;
	u8 read_buffer[4];
	u8 cmd1[4];	
#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	int result = 0;

	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd1,
		.rx_buf = &read_buffer,
		.len    = ZFS1096_REG_SIZE(ZFS1096_REG_READ_KEY_STABLE),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	memset(read_buffer,0x0,4);
	memset(cmd1,0x0,4);
	
	cmd1[0] = ZFS1096_REG_READ_KEY_STABLE;
	
	if (!timeout) {
		result = wait_event_interruptible(
				zfs_dev->wq_irq_return,
				zfs_dev->interrupt_done);
	} else {
		result = wait_event_interruptible_timeout(
				zfs_dev->wq_irq_return,
				zfs_dev->interrupt_done, timeout);
	}		
	
	if (result < 0) {
		dev_err(&zfs_dev->pdev->dev,
			 "[ZET] %s wait_event_interruptible interrupted by signal.\n", __func__);
			 
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_IGNORE);	 
			 
		return result;
	}

	if (result || !timeout) {
		zfs_dev->interrupt_done = false;		

		spi_message_init(&msg);
		spi_message_add_tail(&t1,  &msg);

		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);	
			return error;
		}		
		
		memcpy(key_buf, read_buffer+2, 2);
		
		return 0;  //sucess
	}
	
	if(timeout_clear_int)
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_IGNORE);	
	
	return -ETIMEDOUT;

}		

/* -------------------------------------------------------------------- */
int zfs1096_read_hw_info(zfs_data_t *zfs_dev)
{
	int error = 0;
	struct spi_message msg;
	u8 tx_cmd[2] = { 0xB2, 0xF9 };
	u8 cmd2[4];
	int timeout = 50;
	int i;
	int result = 0;

#ifdef FPC_MTK		
	struct mt_chip_conf *spi_par;
#endif
	
	struct spi_transfer t1 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &tx_cmd,
		.rx_buf = NULL,
		.len    = ZFS1096_REG_SIZE(ZFS1096_REG_READ_HW_INFO),
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};			
	
	struct spi_transfer t2 = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = (u32)zfs_dev->spi_freq_khz * 1000u,
		.tx_buf = &cmd2,
		.rx_buf = zfs_dev->zfs1096_fw_info,
		.len    = 18,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};		
	
#ifdef FPC_MTK		
	spi_par	= (struct mt_chip_conf *)zfs_dev->pdev->controller_data;
	spi_par->com_mod = DMA_TRANSFER;//FIFO_TRANSFER;
#endif

	spi_message_init(&msg);
	spi_message_add_tail(&t1,  &msg);

	error = spi_sync(zfs_dev->pdev, &msg);

	if (error<0)
	{
		dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t1) failed.\n", __func__);		
		return error;
	}
	
	error = result;
	
	if (!timeout) {
		result = wait_event_interruptible(
				zfs_dev->wq_irq_return,
				zfs_dev->interrupt_done);
	} else {
		result = wait_event_interruptible_timeout(
				zfs_dev->wq_irq_return,
				zfs_dev->interrupt_done, timeout);
	}		
		
	memset(zfs_dev->zfs1096_fw_info,0x0,sizeof(zfs_dev->zfs1096_fw_info));	
	memset(cmd2,0x0,sizeof(cmd2));
	
	cmd2[0] = ZFS1096_REG_READ_KEY_STABLE;
	
	if (result < 0) {
		dev_err(&zfs_dev->pdev->dev,
			 "[ZET] %s wait_event_interruptible interrupted by signal.\n", __func__);
			 
		printk("[ZET] 0xB2 : ");
			 
		for(i=0; i<18; i++)
		{
			zfs_dev->zfs1096_fw_info[i] = 0xF0;
			printk("0x%02x ", zfs_dev->zfs1096_fw_info[i]);
		}
		
		printk("\n");
			 
		zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);	 
			 
		return result;
	}

	if (result || !timeout) {
		zfs_dev->interrupt_done = false;		

		spi_message_init(&msg);
		spi_message_add_tail(&t2,  &msg);

		error = spi_sync(zfs_dev->pdev, &msg);

		if (error<0)
		{
			dev_err(&zfs_dev->pdev->dev, "[ZET] %s(t2) failed.\n", __func__);	
			
			printk("[ZET] 0xB2 : ");
			
			for(i=0; i<18; i++)
			{
				zfs_dev->zfs1096_fw_info[i] = 0x80;
				printk("0x%02x ", zfs_dev->zfs1096_fw_info[i]);
			}			
			
			printk("\n");
			
			zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);
			
			return error;
		}		
		
	}
	
	printk("[ZET] 0xB2 : ");
	
	for(i=0; i<18; i++)
	{
		printk("0x%02x ", zfs_dev->zfs1096_fw_info[i]);
	}			
	
	printk("\n");	
	
	zfs1096_clear_int(zfs_dev, ZFS1096_IRQ_EXIT);
	
	return error;
}

#endif /* ZET_CAPTURE_SUPPORT */