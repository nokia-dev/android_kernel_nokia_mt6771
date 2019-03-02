/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef BUILD_LK
#include <string.h>
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <platform/upmu_common.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <asm-generic/gpio.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#endif

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#endif

#include "lcm_drv.h"
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output);
#ifndef BUILD_LK
static unsigned int LCD_RST_PIN;
static unsigned int LCD_PWR_EN_PIN;
static unsigned int LCD_PWR2_EN_PIN;
static unsigned int LCD_STB_EN_PIN;
void lcm_request_gpio_control(struct device *dev)
{
	LCD_RST_PIN = of_get_named_gpio(dev->of_node, "lcd_rst_pin", 0);
	gpio_request(LCD_RST_PIN, "LCD_RST_PIN");

	LCD_PWR_EN_PIN = of_get_named_gpio(dev->of_node, "lcd_pwr_en_pin", 0);
	gpio_request(LCD_PWR_EN_PIN, "LCD_PWR_EN_PIN");

	LCD_PWR2_EN_PIN = of_get_named_gpio(dev->of_node, "lcd_pwr2_en_pin", 0);
	gpio_request(LCD_PWR2_EN_PIN, "LCD_PWR2_EN_PIN");

	LCD_STB_EN_PIN = of_get_named_gpio(dev->of_node, "lcd_stb_pin", 0);
	gpio_request(LCD_STB_EN_PIN, "LCD_STB_EN_PIN");
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	lcm_request_gpio_control(dev);

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
	 .compatible = "jdi,jd9365_pbt101wxb40i072_wxga_boe",
	 .data = 0,
	 }, {
	     /* sentinel */
	     }
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		   .name = "jd9365_pbt101wxb40i072_wxga_boe",
		   .owner = THIS_MODULE,
		   .of_match_table = lcm_platform_of_match,
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: register lcm init driver done\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM: failed to register this driver!\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
}

late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("LCM display subsystem driver");
MODULE_LICENSE("GPL");
#endif

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE								0
#define FRAME_WIDTH                                     (800)
#define FRAME_HEIGHT                                    (1280)

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)      lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define REGFLAG_DELAY                               0xFC
#define REGFLAG_END_OF_TABLE                        0xFD	/* END OF REGISTERS MARKER */

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */
static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	if (GPIO == 0xFFFFFFFF) {
#ifdef BUILD_LK
		printf("[LK/LCM] GPIO_LCD_PWR_EN =   0x%x\n", GPIO_LCD_PWR_EN);
		printf("[LK/LCM] GPIO_LCD_BIAS_ENP =   0x%x\n", GPIO_LCD_BIAS_ENP);
		printf("[LK/LCM] GPIO_LCM_RST =   0x%x\n", GPIO_LCD_RST);
#else
		pr_debug("[Kernel/LCM] LCD_PWR_EN_PIN =   0x%x\n", LCD_PWR_EN_PIN);
		pr_debug("[Kernel/LCM] LCD_PWR2_EN_PIN =   0x%x\n", LCD_PWR2_EN_PIN);
		pr_debug("[Kernel/LCM] LCD_RST_PIN =   0x%x\n", LCD_RST_PIN);
		pr_debug("[Kernel/LCM] LCD_STB_EN_PIN =   0x%x\n", LCD_STB_EN_PIN);
#endif
		return;
	}
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

#ifdef BUILD_LK
	printf("[LK/LCM] %s() enter\n", __func__);
#else
	pr_notice("[Kernel/LCM] %s() enter\n", __func__);
#endif
	/*  Page0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);

	/*  PASSWORD */
	data_array[0] = 0x93E11500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65E21500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xF8E31500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Lane select by internal reg  4 lanes */
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x032D1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x03801500; /* 0X03=4-LANE; 0x02=3-LANE */
	dsi_set_cmdq(data_array, 1, 1);

	/* --- Sequence Ctrl  ----	 */
	data_array[0] = 0x02701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);

	/* --- Page1  ---- */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set VCOM */
	data_array[0] = 0x00001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x62011500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set VCOM_Reverse */
	data_array[0] = 0x00031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6D041500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set Gamma Power, VGMP,VGMN,VGSP,VGSN */
	data_array[0] = 0x00171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xBF181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xBF1B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x001C1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set Gate Power */
	data_array[0] = 0x3E1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x28211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0E221500;
	dsi_set_cmdq(data_array, 1, 1);

	/* SETPANEL */
	data_array[0] = 0x09371500;
	dsi_set_cmdq(data_array, 1, 1);

	/* SET RGBCYC */
	data_array[0] = 0x04381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x08391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x123A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x783C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xFF3E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7F3F1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Set TCON */
	data_array[0] = 0x06401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0xA0411500;
	dsi_set_cmdq(data_array, 1, 1);

	/* --- power voltage  ---- */
	data_array[0] = 0x01551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x69571500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x0A591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x295A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x155B1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* --- Gamma  ---- */
	data_array[0] = 0x7C5D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x655E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x555F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49601500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44611500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x35621500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3A631500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23641500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D651500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3C661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5D681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x566A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x486B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x456C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x386D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x256E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7C701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x65711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x55721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x35751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3A761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x23771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3C791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x3D7A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5D7B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4D7C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x567D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x487E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x457F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x38801500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x25811500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00821500;
	dsi_set_cmdq(data_array, 1, 1);

	/* Page2, for GIP */
	data_array[0] = 0x02E01500;
	dsi_set_cmdq(data_array, 1, 1);

	/* GIP_L Pin mapping */
	data_array[0] = 0x1E001500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E011500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41021500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41031500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x43041500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x43051500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F061500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F071500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F081500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F091500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E0A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E0B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F0C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x470D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x470E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x450F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45101500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B111500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B121500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49131500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49141500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F151500;
	dsi_set_cmdq(data_array, 1, 1);

	/* GIP_R Pin mapping */
	data_array[0] = 0x1E161500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E171500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40181500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40191500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x421A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x421B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F1C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F1D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F1E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1f1F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E201500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E211500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1f221500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46231500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46241500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44251500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x44261500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A271500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A281500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48291500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x482A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1f2B1500;
	dsi_set_cmdq(data_array, 1, 1);

	/* GIP_L_GS Pin mapping */
	data_array[0] = 0x1F2C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F2D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x422E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x422F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40301500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x40311500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E321500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E331500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F341500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F351500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E361500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E371500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F381500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x48391500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x483A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A3B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4A3C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x443D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x443E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x463F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x46401500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F411500;
	dsi_set_cmdq(data_array, 1, 1);

	/* GIP_R_GS Pin mapping */
	data_array[0] = 0x1F421500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F431500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x43441500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x43451500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41461500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x41471500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E481500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E491500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E4A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E4C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1E4D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F4E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x494F1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x49501500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B511500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x4B521500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45531500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x45541500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47551500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x47561500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F571500;
	dsi_set_cmdq(data_array, 1, 1);

	/*GIP Timing */
	data_array[0] = 0x10581500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00591500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x005A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x305B1500; /*STV_S0 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025C1500; /*STV_S0 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x405D1500; /*STV_W / S1 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x015E1500; /*STV_S2 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x025F1500; /*STV_S3 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x30601500; /*ETV_W / S1 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x01611500; /*ETV_S2 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02621500; /*ETV_S3 */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A631500; /*SETV_ON */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A641500; /*SETV_OFF */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x05651500; /*ETV */
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x12661500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x74671500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x04681500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A691500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x6A6A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x086B1500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x006C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x066D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x006E1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x886F1500;
	dsi_set_cmdq(data_array, 1, 1);
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00701500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00711500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x06721500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B731500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00741500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x07751500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00761500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x5D771500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x17781500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x1F791500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007A1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x007C1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x037D1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x7B7E1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*Page4 */
	data_array[0] = 0x04E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2B2B1500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x442E1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*Page1 */
	data_array[0] = 0x01E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x010E1500;
	dsi_set_cmdq(data_array, 1, 1);

	/*Page3 */
	data_array[0] = 0x03E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x2F981500;
	dsi_set_cmdq(data_array, 1, 1);

	/*Page0 */
	data_array[0] = 0x00E01500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02E61500;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x02E71500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(5);

	data_array[0] = 0x00351500;
	dsi_set_cmdq(data_array, 1, 1);
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->density = 213;
#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
#endif

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 24;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 18;
	params->dsi.horizontal_backporch = 18;
	params->dsi.horizontal_frontporch = 18;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	/* video mode timing */
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.PLL_CLOCK = 235;
}

static void lcm_init_lcm(void)
{
#ifndef BUILD_LK
	pr_notice("[Kernel/LCM] lcm_init() enter\n");

	lcm_set_gpio_output(LCD_PWR_EN_PIN, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(LCD_PWR2_EN_PIN, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(LCD_STB_EN_PIN, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);
#endif
}

void lcm_suspend(void)
{
#ifndef BUILD_LK
	pr_notice("[Kernel/LCM] lcm_suspend() enter\n");

	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ZERO);
	MDELAY(5);

	lcm_set_gpio_output(LCD_STB_EN_PIN, GPIO_OUT_ZERO);
	MDELAY(10);

	lcm_set_gpio_output(LCD_PWR_EN_PIN, GPIO_OUT_ZERO);
	MDELAY(10);

	lcm_set_gpio_output(LCD_PWR2_EN_PIN, GPIO_OUT_ZERO);
#endif
}

void lcm_resume(void)
{
#ifndef BUILD_LK
	pr_notice("[Kernel/LCM] lcm_resume() enter\n");

	lcm_set_gpio_output(LCD_PWR_EN_PIN, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(LCD_PWR2_EN_PIN, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(LCD_STB_EN_PIN, GPIO_OUT_ONE);
	MDELAY(10);

	lcm_set_gpio_output(LCD_RST_PIN, GPIO_OUT_ONE);

	init_lcm_registers();
#endif
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

LCM_DRIVER jd9365_pbt101wxb40i072_wxga_boe_lcm_drv = {
	.name = "jd9365_pbt101wxb40i072_wxga_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
};
