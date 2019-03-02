/*
 * Copyright (C) 2013-2016, Shenzhen Huiding Technology Co., Ltd.
 * All Rights Reserved.
 */
#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/* MTK header */
#ifndef CONFIG_SPI_MT65XX
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif
#include "mtk_gpio.h"

/* there is no this file on standardized GPIO platform */
#ifdef CONFIG_MTK_GPIO
#include "mach/gpio_const.h"
#endif

#include "gf_spi_tee.h"

#ifndef CONFIG_SPI_MT65XX
int  gf_ioctl_spi_init_cfg_cmd(struct mt_chip_conf *mcc, unsigned long arg){
    return 0;
}
#endif

int gf_ioctl_transfer_raw_cmd(struct gf_device *gf_dev, unsigned long arg,unsigned int bufsiz){
    return 0;
}

