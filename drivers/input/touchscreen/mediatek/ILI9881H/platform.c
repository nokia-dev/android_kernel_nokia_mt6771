/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include "common.h"
#include "core/config.h"
#include "core/i2c.h"
#include "core/spi.h"
#include "core/firmware.h"
#include "core/finger_report.h"
#include "core/flash.h"
#include "core/protocol.h"
#include "platform.h"
#include "core/parser.h"
#include "core/mp_test.h"
#include "core/gesture.h"
#include <../NT36xxx/fih_touch.h>
#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"

#if (TP_PLATFORM == PT_MTK)
#define DTS_OF_NAME		"mediatek,cap_touch"
#include "tpd.h"
extern struct tpd_device *tpd;
#define MTK_RST_GPIO GTP_RST_PORT
#define MTK_INT_GPIO GTP_INT_PORT
#else
#define DTS_OF_NAME		"tchip,ilitek"
#endif /* PT_MTK */

#define DEVICE_ID	"ILITEK_TDDI"

#ifdef USE_KTHREAD
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#endif

/* Debug level */
//uint32_t ipio_debug_level = DEBUG_ALL;
uint32_t ipio_debug_level = 0;  //20180816 add
EXPORT_SYMBOL(ipio_debug_level);

extern struct fih_touch_cb touch_cb;
extern int tp_probe_success;
extern void fih_ili_selftest_open(void);
extern int fih_touch_proc_init(void);
extern int fih_ili_selftest_read(void);
extern void fih_ili_tpfwver_read(char *);
extern char mtkfb_lcm_name[256];
//extern void touch_tpfwimver_read(char *fw_ver);
//extern void touch_fwupgrade(int);
//extern void touch_fwupgrade_read(char *);
extern void fih_ili_vendor_read(char *);

struct ilitek_platform_data *ipd = NULL;

void ilitek_platform_disable_irq(void)
{
	unsigned long nIrqFlag;

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			disable_irq_nosync(ipd->isr_gpio);
			ipd->isEnableIRQ = false;
			ipio_debug(DEBUG_IRQ, "Disable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already disabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_disable_irq);

void ilitek_platform_enable_irq(void)
{
	unsigned long nIrqFlag;

	spin_lock_irqsave(&ipd->plat_spinlock, nIrqFlag);

	if (!ipd->isEnableIRQ) {
		if (ipd->isr_gpio) {
			enable_irq(ipd->isr_gpio);
			ipd->isEnableIRQ = true;
			ipio_debug(DEBUG_IRQ, "Enable IRQ: %d\n", ipd->isEnableIRQ);
		} else
			ipio_err("The number of gpio to irq is incorrect\n");
	} else
		ipio_debug(DEBUG_IRQ, "IRQ was already enabled\n");

	spin_unlock_irqrestore(&ipd->plat_spinlock, nIrqFlag);
}
EXPORT_SYMBOL(ilitek_platform_enable_irq);

int ilitek_platform_tp_hw_reset(bool isEnable)
{
	int ret = 0;
	ipio_info("HW Reset: %d\n", isEnable);

	ilitek_platform_disable_irq();

	if (isEnable) {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 1);
		mdelay(ipd->delay_time_high);
		tpd_gpio_output(ipd->reset_gpio, 0);
		mdelay(ipd->delay_time_low);
		tpd_gpio_output(ipd->reset_gpio, 1);
		mdelay(ipd->edge_delay);
#else
		gpio_direction_output(ipd->reset_gpio, 1);
		mdelay(ipd->delay_time_high);
		gpio_set_value(ipd->reset_gpio, 0);
		mdelay(ipd->delay_time_low);
		gpio_set_value(ipd->reset_gpio, 1);
		mdelay(ipd->edge_delay);
#endif /* PT_MTK */
	} else {
#if (TP_PLATFORM == PT_MTK)
		tpd_gpio_output(ipd->reset_gpio, 0);
#else
		gpio_set_value(ipd->reset_gpio, 0);
#endif /* PT_MTK */
	}

#ifdef HOST_DOWNLOAD
	core_config_ice_mode_enable();
	ret = core_firmware_upgrade(UPDATE_FW_PATH, true);
	if(ret < 0)
		ipio_err("host download failed!\n");
#endif
	mdelay(10);
	ilitek_platform_enable_irq();
	return ret;
}
EXPORT_SYMBOL(ilitek_platform_tp_hw_reset);

#ifdef REGULATOR_POWER_ON
void ilitek_regulator_power_on(bool status)
{
	int res = 0;

	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (ipd->vdd) {
			res = regulator_enable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_enable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	} else {
		if (ipd->vdd) {
			res = regulator_disable(ipd->vdd);
			if (res < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			res = regulator_disable(ipd->vdd_i2c);
			if (res < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	}
	core_config->icemodeenable = false;
	mdelay(5);
}
EXPORT_SYMBOL(ilitek_regulator_power_on);
#endif /* REGULATOR_POWER_ON */

#ifdef BATTERY_CHECK
static void read_power_status(uint8_t *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug(DEBUG_BATTERY, "Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
}

static void ilitek_platform_vpower_notify(struct work_struct *pWork)
{
	uint8_t charge_status[20] = { 0 };
	static int charge_mode = 0;

	ipio_debug(DEBUG_BATTERY, "isEnableCheckPower = %d\n", ipd->isEnablePollCheckPower);
	read_power_status(charge_status);
	ipio_debug(DEBUG_BATTERY, "Batter Status: %s\n", charge_status);

	if (strstr(charge_status, "Charging") != NULL || strstr(charge_status, "Full") != NULL
	    || strstr(charge_status, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ipio_debug(DEBUG_BATTERY, "Charging mode\n");
			core_config_plug_ctrl(false);
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug(DEBUG_BATTERY, "Not charging mode\n");
			core_config_plug_ctrl(true);
			charge_mode = 2;
		}
	}

	if (ipd->isEnablePollCheckPower)
		queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work, ipd->work_delay);
}
#endif /* BATTERY_CHECK */

#ifdef ESD_CHECK
static void ilitek_platform_esd_recovery(struct work_struct *work)
{
	int ret = 0;

	mutex_lock(&ipd->plat_mutex);
	ret = ilitek_platform_tp_hw_reset(true);
	if(ret < 0)
		ipio_err("host download failed!\n");
	mutex_unlock(&ipd->plat_mutex);
}

static void ilitek_platform_esd_check(struct work_struct *pWork)
{
	int ret;
	uint8_t tx_data = 0x82, rx_data = 0;

#if (INTERFACE == SPI_INTERFACE)
	ipio_debug(DEBUG_BATTERY, "isEnablePollCheckEsd = %d\n", ipd->isEnablePollCheckEsd);
	if (spi_write_then_read(core_spi->spi, &tx_data, 1, &rx_data, 1) < 0) {
		ipio_err("spi Write Error\n");
	}

	if(rx_data == 0x82) {
		ipio_info("Doing ESD recovery (0x%x)\n", rx_data);
		schedule_work(&ipd->esd_recovery);
	} else {
		if (ipd->isEnablePollCheckEsd)
			queue_delayed_work(ipd->check_esd_status_queue,
				&ipd->check_esd_status_work, ipd->esd_check_time);
	}
#endif /* SPI_INTERFACE */
}
#endif /* ESD_CHECK */

#if (TP_PLATFORM == PT_MTK)
static void tpd_resume(struct device *h)
{
	ipio_info("TP Resuem\n");

	if (!core_firmware->isUpgrading) {
		core_config_ic_resume();
	}
}
#if 1
void FIH_ili_tp_lcm_resume(void)
{
	 pr_err("F@TOUCH %s resume \n",__func__);
	 tpd_resume(&ipd->client->dev);
}

EXPORT_SYMBOL(FIH_ili_tp_lcm_resume);
#endif
static void tpd_suspend(struct device *h)
{
	ipio_info("TP Suspend\n");

	if (!core_firmware->isUpgrading) {
		core_config_ic_suspend();
	}
}
#if 1
void FIH_ili_tp_lcm_suspend(void)
{
	 pr_info("F@TOUCH %s sleep start\n",__func__);
	 tpd_suspend(&ipd->client->dev);
}
EXPORT_SYMBOL(FIH_ili_tp_lcm_suspend);
#endif
#elif defined CONFIG_FB
static int ilitek_platform_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);

	/*
	 *  FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *  FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data && (event == FB_EVENT_BLANK)) {
		blank = evdata->data;

#if (TP_PLATFORM == PT_SPRD)
		if (*blank == DRM_MODE_DPMS_OFF)
#else
		if (*blank == FB_BLANK_POWERDOWN)
#endif /* PT_SPRD */
		{
			ipio_info("TP Suspend\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_suspend();
			}
		}
#if (TP_PLATFORM == PT_SPRD)
		else if (*blank == DRM_MODE_DPMS_ON)
#else
		else if (*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL)
#endif /* PT_SPRD */
		{
			ipio_info("TP Resuem\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_resume();
			}
		}
	}

	return NOTIFY_OK;
}
#else /* CONFIG_HAS_EARLYSUSPEND */
static void ilitek_platform_early_suspend(struct early_suspend *h)
{
	ipio_info("TP Suspend\n");

	/* TODO: there is doing nothing if an upgrade firmware's processing. */

	core_fr_touch_release(0, 0, 0);

	input_sync(core_fr->input_device);

	core_fr->isEnableFR = false;

	core_config_ic_suspend();
}

static void ilitek_platform_late_resume(struct early_suspend *h)
{
	ipio_info("TP Resuem\n");

	core_fr->isEnableFR = true;
	core_config_ic_resume();
}
#endif /* PT_MTK */

/**
 * reg_power_check - register a thread to inquery status at certain time.
 */
static int ilitek_platform_reg_power_check(void)
{
	int res = 0;

#ifdef BATTERY_CHECK
	INIT_DELAYED_WORK(&ipd->check_power_status_work, ilitek_platform_vpower_notify);
	ipd->check_power_status_queue = create_workqueue("ili_power_check");
	ipd->work_delay = msecs_to_jiffies(CHECK_BATTERY_TIME);
	ipd->isEnablePollCheckPower = true;
	if (!ipd->check_power_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vpower_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->work_delay);

		if (ipd->isEnablePollCheckPower) {
			queue_delayed_work(ipd->check_power_status_queue, &ipd->check_power_status_work,
					   ipd->work_delay);
			ipd->vpower_reg_nb = true;
		}
	}
#endif /* BATTERY_CHECK */

	return res;
}

static int ilitek_platform_reg_esd_check(void)
{
	int res = 0;

#ifdef ESD_CHECK
	INIT_DELAYED_WORK(&ipd->check_esd_status_work, ilitek_platform_esd_check);
	ipd->check_esd_status_queue = create_workqueue("ili_esd_check");
	ipd->esd_check_time = msecs_to_jiffies(CHECK_ESD_TIME);
	ipd->isEnablePollCheckEsd = true;
	if (!ipd->check_esd_status_queue) {
		ipio_err("Failed to create a work thread to check power status\n");
		ipd->vesd_reg_nb = false;
		res = -1;
	} else {
		ipio_info("Created a work thread to check power status at every %u jiffies\n",
			 (unsigned)ipd->esd_check_time);

		INIT_WORK(&ipd->esd_recovery, ilitek_platform_esd_recovery);

		if (ipd->isEnablePollCheckEsd) {
			queue_delayed_work(ipd->check_esd_status_queue, &ipd->check_esd_status_work,
					   ipd->esd_check_time);
			ipd->vesd_reg_nb = true;
		}
	}
#endif /* ESD_CHECK */

	return res;
}
/**
 * Register a callback function when the event of suspend and resume occurs.
 *
 * The default used to wake up the cb function comes from notifier block mechnaism.
 * If you'd rather liek to use early suspend, CONFIG_HAS_EARLYSUSPEND in kernel config
 * must be enabled.
 */
static int ilitek_platform_reg_suspend(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	ipio_info("It does nothing if platform is MTK\n");
#else
	ipio_info("Register suspend/resume callback function\n");
#ifdef CONFIG_FB
	ipd->notifier_fb.notifier_call = ilitek_platform_notifier_fb;
#if (TP_PLATFORM == PT_SPRD)
	res = adf_register_client(&ipd->notifier_fb);
#else
	res = fb_register_client(&ipd->notifier_fb);
#endif /* PT_SPRD */
#else
	ipd->early_suspend->suspend = ilitek_platform_early_suspend;
	ipd->early_suspend->esume = ilitek_platform_late_resume;
	ipd->early_suspend->level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	res = register_early_suspend(ipd->early_suspend);
#endif /* CONFIG_FB */
#endif /* PT_MTK */

	return res;
}

#ifndef USE_KTHREAD
static void ilitek_platform_work_queue(struct work_struct *work)
{
	ipio_debug(DEBUG_IRQ, "work_queue: IRQ = %d\n", ipd->isEnableIRQ);

	core_fr_handler();

	if (!ipd->isEnableIRQ)
		ilitek_platform_enable_irq();
}
#endif /* USE_KTHREAD */

static irqreturn_t ilitek_platform_irq_handler(int irq, void *dev_id)
{
	ipio_debug(DEBUG_IRQ, "IRQ = %d\n", ipd->isEnableIRQ);

	if (ipd->isEnableIRQ) {
		ilitek_platform_disable_irq();
#ifdef USE_KTHREAD
		ipd->irq_trigger = true;
		wake_up_interruptible(&waiter);
#else
		schedule_work(&ipd->report_work_queue);
#endif /* USE_KTHREAD */
	}

	return IRQ_HANDLED;
}

static int ilitek_platform_input_init(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	int i;

	ipd->input_device = tpd->dev;

	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++) {
			input_set_capability(ipd->input_device, EV_KEY, tpd_dts_data.tpd_key_local[i]);
		}
	}
	core_fr_input_set_param(ipd->input_device);
	return res;
#else
	ipd->input_device = input_allocate_device();

	if (ERR_ALLOC_MEM(ipd->input_device)) {
		ipio_err("Failed to allocate touch input device\n");
		res = -ENOMEM;
		goto fail_alloc;
	}

#if (INTERFACE == I2C_INTERFACE)
	ipd->input_device->name = ipd->client->name;
	ipd->input_device->phys = "I2C";
	ipd->input_device->dev.parent = &ipd->client->dev;
	ipd->input_device->id.bustype = BUS_I2C;
#else
	ipd->input_device->name = DEVICE_ID;
	ipd->input_device->phys = "SPI";
	ipd->input_device->dev.parent = &ipd->spi->dev;
	ipd->input_device->id.bustype = BUS_SPI;
#endif

	core_fr_input_set_param(ipd->input_device);
	/* register the input device to input sub-system */
	res = input_register_device(ipd->input_device);
	if (res < 0) {
		ipio_err("Failed to register touch input device, res = %d\n", res);
		goto out;
	}

	return res;

fail_alloc:
	input_free_device(core_fr->input_device);
	return res;

out:
	input_unregister_device(ipd->input_device);
	input_free_device(core_fr->input_device);
	return res;
#endif /* PT_MTK */
}

static int kthread_handler(void *arg)
{
	int res = 0;
	char *str = (char *)arg;

	if (strcmp(str, "boot_fw") == 0) {
		/* FW Upgrade event */
		core_firmware->isboot = true;

		ilitek_platform_disable_irq();

#ifdef HOST_DOWNLOAD
		res = core_firmware_boot_host_download();
#else
#ifdef BOOT_FW_UPGRADE
		res = core_firmware_boot_upgrade();
#endif /* BOOT_FW_UPGRADE */
#endif /* HOST_DOWNLOAD */
		if (res < 0)
			ipio_err("Failed to upgrade FW at boot stage\n");

		ilitek_platform_enable_irq();

		ilitek_platform_input_init();

		core_firmware->isboot = false;
	} else if (strcmp(str, "irq") == 0) {
		/* IRQ event */
		struct sched_param param = {.sched_priority = 4 };

		sched_setscheduler(current, SCHED_RR, &param);

		while (!kthread_should_stop() && !ipd->free_irq_thread) {
			set_current_state(TASK_INTERRUPTIBLE);
			wait_event_interruptible(waiter, ipd->irq_trigger);
			ipd->irq_trigger = false;
			set_current_state(TASK_RUNNING);
			mutex_lock(&ipd->touch_mutex);
			core_fr_handler();
			mutex_unlock(&ipd->touch_mutex);
			//ilitek_platform_enable_irq();
		}
	} else {
		ipio_err("Unknown EVENT\n");
	}

	return res;
}

static int ilitek_platform_isr_register(void)
{
	int res = 0;
#if (TP_PLATFORM == PT_MTK)
	u32 ints[2] = { 0, 0 };
	struct device_node *node;
#endif /* PT_MTK */

#ifdef USE_KTHREAD
	ipd->irq_thread = kthread_run(kthread_handler, "irq", "ili_irq_thread");
	if (ipd->irq_thread == (struct task_struct *)ERR_PTR) {
		ipd->irq_thread = NULL;
		ipio_err("Failed to create kthread\n");
		res = -ENOMEM;
		goto out;
	}
	ipd->irq_trigger = false;
	ipd->free_irq_thread = false;
#else
	INIT_WORK(&ipd->report_work_queue, ilitek_platform_work_queue);
#endif /* USE_KTHREAD */

#if (TP_PLATFORM == PT_MTK)
	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		ipd->isr_gpio = irq_of_parse_and_map(node, 0);
	}
#else
	ipd->isr_gpio = gpio_to_irq(ipd->int_gpio);
#endif /* PT_MTK */

	ipio_info("ipd->isr_gpio = %d\n", ipd->isr_gpio);

	res = request_irq(ipd->isr_gpio, ilitek_platform_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", ipd);

	if (res > 0) {
		res = -1;
		ipio_err("Failed to register irq handler, irq = %d, res = %d\n", ipd->isr_gpio, res);
		goto out;
	}
	ipio_info("irq:%d, debounce:%d-%d:res= %d\n", ipd->isr_gpio, ints[0], ints[1],res);
	ipd->isEnableIRQ = true;

out:
	return res;
}

static int ilitek_platform_gpio(void)
{
	int res = 0;
#if (TP_PLATFORM == PT_MTK)
	//ipd->int_gpio = tpd_gpio_as_int(MTK_INT_GPIO);
	tpd_gpio_as_int(MTK_INT_GPIO);
	ipd->reset_gpio = MTK_RST_GPIO;
	return res;
#else
#ifdef CONFIG_OF
#if(INTERFACE == I2C_INTERFACE)
	struct device_node *dev_node = ipd->client->dev.of_node;
#else
	struct device_node *dev_node = ipd->spi->dev.of_node;
#endif
	uint32_t flag;

	ipd->int_gpio = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	ipd->reset_gpio = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
#endif /* CONFIG_OF */
#endif /* PT_MTK */

	ipio_info("GPIO INT: %d\n", ipd->int_gpio);
	ipio_info("GPIO RESET: %d\n", ipd->reset_gpio);

	if (!gpio_is_valid(ipd->int_gpio)) {
		ipio_err("Invalid INT gpio: %d\n", ipd->int_gpio);
		return -EBADR;
	}

	if (!gpio_is_valid(ipd->reset_gpio)) {
		ipio_err("Invalid RESET gpio: %d\n", ipd->reset_gpio);
		return -EBADR;
	}

	res = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
	if (res < 0) {
		ipio_err("Request IRQ GPIO failed, res = %d\n", res);
		gpio_free(ipd->int_gpio);
		res = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
		if (res < 0) {
			ipio_err("Retrying request INT GPIO still failed , res = %d\n", res);
			goto out;
		}
			ipio_err("Retry request INT GPIO Done\n");
	}
#if 0
	res = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
	if (res < 0) {
		ipio_err("Request RESET GPIO failed, res = %d\n", res);
		gpio_free(ipd->reset_gpio);
		res = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
		if (res < 0) {
			ipio_err("Retrying request RESET GPIO still failed , res = %d\n", res);
			goto out;
		}
	}
#endif
	gpio_direction_input(ipd->int_gpio);

out:
	return res;
}

void ilitek_platform_read_tp_info(void)
{
	if (core_config_get_chip_id() < 0) {
		ipio_err("Failed to get chip id\n");
	}

	if (core_config_get_protocol_ver() < 0) {
		ipio_err("Failed to get protocol version\n");
	}

	if (core_config_get_fw_ver() < 0) {
		ipio_err("Failed to get firmware version\n");
	}

	if (core_config_get_core_ver() < 0) {
		ipio_err("Failed to get core version\n");
	}

	if (core_config_get_tp_info() < 0) {
		ipio_err("Failed to get TP information\n");
	}

	if (core_config_get_key_info() < 0) {
		ipio_err("Failed to get key information\n");
	}
}
EXPORT_SYMBOL(ilitek_platform_read_tp_info);

/**
 * The function is to initialise all necessary structurs in those core APIs,
 * they must be called before the i2c dev probes up successfully.
 */
static int ilitek_platform_core_init(void)
{
	ipio_info("Initialise core's components\n");

	if (core_config_init() < 0 || core_protocol_init() < 0 ||
		core_firmware_init() < 0 || core_fr_init() < 0 ||
		core_gesture_init () < 0) {
		ipio_err("Failed to initialise core components\n");
		return -EINVAL;
	}

#if (INTERFACE == I2C_INTERFACE)
	if (core_i2c_init(ipd->client) < 0) {
#else
	if (core_spi_init(ipd->spi) < 0) {
#endif

		ipio_err("Failed to initialise interface\n");
		return -EINVAL;
	}
	return 0;
}

void fih_ili_tpfwver_read(char *fw_ver)
{
		ipio_info("Get FW Ver\n");
		if( core_config_get_fw_ver()<0)
			ipio_err("Failed to get firmware version\n");

		ipio_info("fih_ili_tpfwver_read mtkfb_lcm_name= %s \n",mtkfb_lcm_name);
           ipio_err("core_config->firmware_ver %s",core_config->firmware_ver);
	if (strcmp(mtkfb_lcm_name, "ili9881h_hdplus_dsi_vdo_tianma_rt5081_drv") == 0)
	    snprintf(fw_ver, PAGE_SIZE,"TM-Ilitek-%d.%d.%d.%d\n", core_config->firmware_ver[1], core_config->firmware_ver[2], core_config->firmware_ver[3], core_config->firmware_ver[4]);
    else
        snprintf(fw_ver, PAGE_SIZE,"TM-Ilitek-0.0.0.0\n");
}
void fih_ili_vendor_read(char *buf)
{
    sprintf(buf, "%s\n","IliTek");
}


#if (INTERFACE == I2C_INTERFACE)
static int ilitek_platform_remove(struct i2c_client *client)
#else
static int ilitek_platform_remove(struct spi_device *spi)
#endif
{
	ipio_info("Remove platform components\n");

	if (ipd->isEnableIRQ) {
		disable_irq_nosync(ipd->isr_gpio);
	}

	if (ipd->isr_gpio != 0 && ipd->int_gpio != 0 && ipd->reset_gpio != 0) {
		free_irq(ipd->isr_gpio, (void *)ipd->i2c_id);
		gpio_free(ipd->int_gpio);
		gpio_free(ipd->reset_gpio);
	}
#ifdef CONFIG_FB
	fb_unregister_client(&ipd->notifier_fb);
#else
	unregister_early_suspend(&ipd->early_suspend);
#endif /* CONFIG_FB */

#ifdef USE_KTHREAD
	if (ipd->irq_thread != NULL) {
		ipd->irq_trigger = true;
		ipd->free_irq_thread = true;
		wake_up_interruptible(&waiter);
		kthread_stop(ipd->irq_thread);
		ipd->irq_thread = NULL;
	}
#endif /* USE_KTHREAD */

	if (ipd->input_device != NULL) {
		input_unregister_device(ipd->input_device);
		input_free_device(ipd->input_device);
	}

	if (ipd->vpower_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		destroy_workqueue(ipd->check_power_status_queue);
	}

	if (ipd->vesd_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_esd_status_work);
		destroy_workqueue(ipd->check_esd_status_queue);
	}

	ilitek_proc_remove();
	return 0;
}

/**
 * The probe func would be called after an i2c device was detected by kernel.
 *
 * It will still return zero even if it couldn't get a touch ic info.
 * The reason for why we allow it passing the process is because users/developers
 * might want to have access to ICE mode to upgrade a firwmare forcelly.
 */
#if (INTERFACE == I2C_INTERFACE)
static int ilitek_platform_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int ilitek_platform_probe(struct spi_device *spi)
#endif
{
#ifdef REGULATOR_POWER_ON
#if (TP_PLATFORM == PT_MTK)
	const char *vdd_name = "vtouch";
#else
	const char *vdd_name = "vdd";
#endif /* PT_MTK */
	const char *vcc_i2c_name = "vcc_i2c";
#endif /* REGULATOR_POWER_ON */

		ipio_err("enter ilitek_platform_probe\n");
#if (INTERFACE == I2C_INTERFACE)
	if (client == NULL) {
		ipio_err("i2c client is NULL\n");
		return -ENODEV;
	}

	/* Set i2c slave addr if it's not configured */
	ipio_info("I2C Slave address = 0x%x\n", client->addr);
	if (client->addr != ILI9881_SLAVE_ADDR) {
		client->addr = ILI9881_SLAVE_ADDR;
		ipio_err("I2C Slave addr doesn't be set up, use default : 0x%x\n", client->addr);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ipio_err("I2C not supported\n");
		return -ENODEV;
	}
#else
	if (spi == NULL) {
		ipio_err("spi device is NULL\n");
		return -ENODEV;
	}
#endif

	/* initialise the struct of touch ic memebers. */
#if (INTERFACE == I2C_INTERFACE)
	ipd = devm_kzalloc(&client->dev, sizeof(*ipd), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ipd)) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		return -ENOMEM;
	}

	ipd->client = client;
	ipd->i2c_id = id;
	ipd->dev = &client->dev;
#else
	ipd = devm_kzalloc(&spi->dev, sizeof(*ipd), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ipd)) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		return -ENOMEM;
	}

	ipd->spi = spi;
	ipd->dev = &spi->dev;
#endif

	ipd->chip_id = TP_TOUCH_IC;
	ipd->isEnableIRQ = false;
	ipd->isEnablePollCheckPower = false;
	ipd->isEnablePollCheckEsd = false;
	ipd->vpower_reg_nb = false;
	ipd->vesd_reg_nb = false;

	ipio_info("Driver Version : %s\n", DRIVER_VERSION);
	ipio_info("Driver for Touch IC :  %x\n", TP_TOUCH_IC);
	ipio_info("Driver on platform :  %x\n", TP_PLATFORM);
	ipio_info("Driver interface :  %s\n", (INTERFACE == I2C_INTERFACE) ? "I2C" : "SPI");

	/*
	 * Different ICs may require different delay time for the reset.
	 * They may also depend on what your platform need to.
	 */
	 if (ipd->chip_id == CHIP_TYPE_ILI9881) {
		 ipd->delay_time_high = 10;
		 ipd->delay_time_low = 5;
#if (INTERFACE == I2C_INTERFACE)
		 ipd->edge_delay = 100;
#else
		 ipd->edge_delay = 1;
#endif
	}

	mutex_init(&ipd->plat_mutex);
	mutex_init(&ipd->touch_mutex);
	spin_lock_init(&ipd->plat_spinlock);

	/* Init members for debug */
	mutex_init(&ipd->ilitek_debug_mutex);
	mutex_init(&ipd->ilitek_debug_read_mutex);
	init_waitqueue_head(&(ipd->inq));
	ipd->debug_data_frame = 0;
	ipd->debug_node_open = false;

#ifdef REGULATOR_POWER_ON
#if (TP_PLATFORM == PT_MTK)
	ipd->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	tpd->reg = ipd->vdd;
#else
	ipd->vdd = regulator_get(&ipd->client->dev, vdd_name);
#endif /* PT_MTK */
	if (ERR_ALLOC_MEM(ipd->vdd)) {
		ipio_err("regulator_get vdd fail\n");
		ipd->vdd = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
			ipio_err("Failed to set vdd %d.\n", VDD_VOLTAGE);
	}

	ipd->vdd_i2c = regulator_get(&ipd->client->dev, vcc_i2c_name);
	if (ERR_ALLOC_MEM(ipd->vdd_i2c)) {
		ipio_err("regulator_get vdd_i2c fail.\n");
		ipd->vdd_i2c = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd_i2c, VDD_I2C_VOLTAGE, VDD_I2C_VOLTAGE) < 0)
			ipio_err("Failed to set vdd_i2c %d\n", VDD_I2C_VOLTAGE);
	}
	ilitek_regulator_power_on(true);
#endif /* REGULATOR_POWER_ON */

	if (ilitek_platform_gpio() < 0)
		ipio_err("Failed to request gpios\n ");

	/* If kernel failes to allocate memory to the core components, driver will be unloaded. */
	if (ilitek_platform_core_init() < 0) {
		ipio_err("Failed to allocate cores' mem\n");
		return -ENOMEM;
	}

#ifdef HOST_DOWNLOAD
	core_firmware_boot_host_download();
#else
	//ilitek_platform_tp_hw_reset(0);
#endif

	/* get our tp ic information */
	ilitek_platform_read_tp_info();

	/* If it defines boot upgrade, input register will be done inside boot function. */
#ifndef BOOT_FW_UPGRADE
	if (ilitek_platform_input_init() < 0)
		ipio_err("Failed to init input device in kernel\n");
#endif /* BOOT_FW_UPGRADE */

	if (ilitek_platform_isr_register() != 0)
		ipio_err("Failed to register ISR\n");

	if (ilitek_platform_reg_suspend() < 0)
		ipio_err("Failed to register suspend/resume function\n");

	if (ilitek_platform_reg_power_check() < 0)
		ipio_err("Failed to register power check function\n");

	if (ilitek_platform_reg_esd_check() < 0)
		ipio_err("Failed to register esd check function\n");
	/* Create nodes for users */
	ilitek_proc_init();

#if (TP_PLATFORM == PT_MTK)
	tpd_load_status = 1;
#endif /* PT_MTK */

#ifdef BOOT_FW_UPGRADE
	ipd->update_thread = kthread_run(kthread_handler, "boot_fw", "ili_fw_boot");
	if (ipd->update_thread == (struct task_struct *)ERR_PTR) {
		ipd->update_thread = NULL;
		ipio_err("Failed to create fw upgrade thread\n");
	}
#endif /* BOOT_FW_UPGRADE */

    tp_probe_success = 1;
    fih_touch_proc_init();
    touch_cb.touch_selftest = fih_ili_selftest_open;
    touch_cb.touch_selftest_result = fih_ili_selftest_read;
    touch_cb.touch_tpfwver_read = fih_ili_tpfwver_read;
// touch_cb.touch_tpfwimver_read = touch_tpfwimver_read;
// touch_cb.touch_fwupgrade = touch_fwupgrade;
// touch_cb.touch_fwupgrade_read = touch_fwupgrade_read;
    touch_cb.touch_vendor_read = fih_ili_vendor_read;
    //touch_cb.touch_double_tap_read = touch_double_tap_read_nvt;
    //touch_cb.touch_double_tap_write = touch_double_tap_write_nvt;
// touch_cb.touch_alt_rst = fts_fih_tp_rst;
// touch_cb.touch_alt_st_count = read_register_result;
// touch_cb.touch_alt_st_enable = fts_fih_tp_enable;
		ipio_err("end of ilitek_platform_probe\n");
	return 0;
}

static const struct i2c_device_id tp_device_id[] = {
	{DEVICE_ID, 0},
	{},			/* should not omitted */
};

MODULE_DEVICE_TABLE(i2c, tp_device_id);

/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

#if (TP_PLATFORM == PT_MTK)
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	ipio_info("TPD detect i2c device\n");
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
#endif /* PT_MTK */

#if (INTERFACE == I2C_INTERFACE)
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		   .name = DEVICE_ID,
		   .owner = THIS_MODULE,
		   .of_match_table = tp_match_table,
		   },
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
	.id_table = tp_device_id,
#if (TP_PLATFORM == PT_MTK)
	.detect = tpd_detect,
#endif /* PT_MTK */
};
#else
static struct spi_driver tp_spi_driver = {
	.driver = {
		.name	= DEVICE_ID,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
	},
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
};
#endif

#if (TP_PLATFORM == PT_MTK)
static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

	if (i2c_add_driver(&tp_i2c_driver) != 0) {
		ipio_err("Unable to add i2c driver\n");
		return -1;
	}
	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");

		i2c_del_driver(&tp_i2c_driver);
		return -1;
	}

	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = DEVICE_ID,
	.tpd_local_init = tpd_local_init,
	//.suspend = tpd_suspend,
	//.resume = tpd_resume,
};
#endif /* PT_MTK */

static int __init ilitek_platform_init(void)
{
	int res = 0;

#if (TP_PLATFORM == PT_MTK)
	ipio_info("TDDI TP driver add i2c interface for MTK\n");
	tpd_get_dts_info();
	res = tpd_driver_add(&tpd_device_driver);
	if (res < 0) {
		ipio_err("TPD add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}
#else
#if (INTERFACE == I2C_INTERFACE)
	ipio_info("TDDI TP driver add i2c interface\n");
	return i2c_add_driver(&tp_i2c_driver);
#else
	ipio_info("TDDI TP driver add spi interface\n");
	res = spi_register_driver(&tp_spi_driver);
	if (res < 0) {
		ipio_err("Failed to add ilitek driver\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}
#endif
#endif /* PT_MTK */
		return 0;
}

static void __exit ilitek_platform_exit(void)
{
	ipio_info("I2C driver has been removed\n");

#if (TP_PLATFORM == PT_MTK)
	tpd_driver_remove(&tpd_device_driver);
#else
#if (INTERFACE == I2C_INTERFACE)
	i2c_del_driver(&tp_i2c_driver);
#else
	spi_unregister_driver(&tp_spi_driver);
#endif
#endif /* PT_MTK */
}

module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
