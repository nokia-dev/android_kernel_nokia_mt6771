#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <net/sock.h>
#include <net/netlink.h>

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

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

#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include "elan_fp_mtk_tee.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define VERSION_LOG	"2.2.0"

#define GPIO_PINCTRL    (1)
#define NET_LINK        (0)

static int elan_debug = 1;
#define ELAN_DEBUG(format, args ...) \
do { \
    if (elan_debug) \
        printk("[ELAN] " format, ##args); \
    } while(0)

#define KEY_FP_INT      KEY_POWER //KEY_WAKEUP // change by customer & framework support
#define KEY_FP_INT2     KEY_1 // change by customer & framework support
#define SPI_MAX_SPEED   3*1000*1000

static int factory_status = 0;
static DEFINE_MUTEX(elan_factory_mutex);
static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);
static int elan_work_flag = 0;
static volatile int display_status = 0; // Screen On:0 Off:1
static int display_notify_option = 0;
static int register_fb_flag = 0;

#if defined(CONFIG_FB)
static struct notifier_block fb_notif;
#endif

struct elan_data {
    struct spi_device       *spi;
    struct input_dev        *input_dev;
    int                     int_gpio;
    int                     irq;
    int                     rst_gpio;
    int                     irq_is_disable;
    struct miscdevice       elan_dev; /* char device for ioctl */
    spinlock_t              irq_lock;
    struct wake_lock	    wake_lock;
    struct wake_lock	    hal_wake_lock;
    struct pinctrl          *elan_pinctrl;
    struct pinctrl_state    *eint_as_int, *fp_rst_low, *fp_rst_high;
};

#if NET_LINK
struct sock *nl_sk = NULL;

int pid = 0;

#define NETLINK_TEST 25
#define MAX_MSGSIZE 32


void sendnlmsg(char *msg)
{
	struct sk_buff *skb_1;
	struct nlmsghdr *nlh;
	int len = NLMSG_SPACE(MAX_MSGSIZE);
	int ret = 0;
	if (!msg || !nl_sk || !pid) {
		return ;
	}
	skb_1 = alloc_skb(len, GFP_KERNEL);
	if (!skb_1) {
		pr_err("alloc_skb error\n");
		return;
	}

	nlh = nlmsg_put(skb_1, 0, 0, 0, MAX_MSGSIZE, 0);

	NETLINK_CB(skb_1).portid = 0;
	NETLINK_CB(skb_1).dst_group = 0;

	memcpy(NLMSG_DATA(nlh), msg, sizeof(char));
	pr_debug("send message: %d\n", *(char *)NLMSG_DATA(nlh));

	ret = netlink_unicast(nl_sk, skb_1, pid, MSG_DONTWAIT);
	if (!ret) {
		//kfree_skb(skb_1);
		pr_err("send msg from kernel to usespace failed ret 0x%x\n", ret);
	}
}

void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	char str[100];
	skb = skb_get (__skb);

	if(skb->len >= NLMSG_SPACE(0))
	{
		nlh = nlmsg_hdr(skb);

		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		pid = nlh->nlmsg_pid;

		kfree_skb(skb);
		ELAN_DEBUG("got it %d\n",nlh->nlmsg_pid);

	}

}

int netlink_unicast_init(void)
{
	struct netlink_kernel_cfg netlink_cfg;
	memset(&netlink_cfg, 0, sizeof(struct netlink_kernel_cfg));

	netlink_cfg.groups = 0;
	netlink_cfg.flags = 0;
	netlink_cfg.input = nl_data_ready;
	netlink_cfg.cb_mutex = NULL;

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST,
			&netlink_cfg);

	if(!nl_sk){
		ELAN_DEBUG("create netlink socket error\n");
		return 1;
	}

	return 0;
}
#endif

static irqreturn_t elan_irq_handler(int irq, void *_fp);

void elan_irq_enable(void *_fp)
{
    struct elan_data *fp = _fp;
    unsigned long irqflags = 0;
    ELAN_DEBUG("IRQ Enable = %d.\n", fp->irq);

    spin_lock_irqsave(&fp->irq_lock, irqflags);
    if (fp->irq_is_disable) {
        enable_irq(fp->irq);
        fp->irq_is_disable = 0;
    }
    spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

void elan_irq_disable(void *_fp)
{
    struct elan_data *fp = _fp;
    unsigned long irqflags;
    ELAN_DEBUG("IRQ Disable = %d.\n", fp->irq);

    spin_lock_irqsave(&fp->irq_lock, irqflags);
    if (!fp->irq_is_disable) {
        fp->irq_is_disable = 1;
        disable_irq_nosync(fp->irq);
    }
    spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", VERSION_LOG);
}
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static ssize_t elan_debug_value(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (elan_debug) {
        elan_debug=0;
    }
    else {
        elan_debug=1;
    }
    return sprintf(buf, "[ELAN] elan debug %d\n", elan_debug);
}
static DEVICE_ATTR(elan_debug, S_IRUGO, elan_debug_value, NULL);

static struct attribute *elan_attributes[] = {
    &dev_attr_drv_version.attr,
    &dev_attr_elan_debug.attr,
    NULL
};

static struct attribute_group elan_attr_group = {
    .attrs = elan_attributes,
};

static void elan_reset(struct elan_data *fp)
{
#if GPIO_PINCTRL
    pinctrl_select_state(fp->elan_pinctrl, fp->fp_rst_low);
    mdelay(5);
    pinctrl_select_state(fp->elan_pinctrl, fp->fp_rst_high);
    mdelay(50);
#else
    gpio_set_value(fp->rst_gpio, 0);
    mdelay(5);
    gpio_set_value(fp->rst_gpio, 1);
    mdelay(50);
#endif
}

static void elan_spi_clk_enable(struct elan_data *fp, u8 bonoff)
{
#ifdef CONFIG_MTK_CLKMGR
    if (bonoff)
        enable_clock(MT_CG_PERI_SPI0, "spi");
    else
        disable_clock(MT_CG_PERI_SPI0, "spi");

#else
    static int count;

    if (bonoff && (count == 0)) {
        mt_spi_enable_master_clk(fp->spi);
        count = 1;
    } else if ((count > 0) && (bonoff == 0)) {
        mt_spi_disable_master_clk(fp->spi);
        count = 0;
    }

    ELAN_DEBUG("bus_num %d",fp->spi->master->bus_num);
#endif
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = evdata->data;
    ELAN_DEBUG("%s fb notifier callback event = %lu, evdata->data = %d\n",__func__, event, *blank);
    if (evdata && evdata->data) {
        if (event == FB_EVENT_BLANK) {
            if (*blank == FB_BLANK_UNBLANK) {
                display_status = 0;
                elan_work_flag = 1;
                wake_up(&elan_poll_wq);
                ELAN_DEBUG("Display On\n");
            }
            else if (*blank == FB_BLANK_POWERDOWN) {
                display_status = 1;
                elan_work_flag = 1;
                wake_up(&elan_poll_wq);
                ELAN_DEBUG("Display Off\n");
            }
        }
    }
    return 0;
}
#endif

static long elan_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct elan_data *fp = filp->private_data;
    int keycode;
    int wake_lock_arg;
    int ret = 0;

    ELAN_DEBUG("%s() : cmd = [%04X]\n", __func__, cmd);

    switch(cmd) {
        case ID_IOCTL_RESET: //6
            elan_reset(fp);
            ELAN_DEBUG("[IOCTL] RESET\n");
            break;

        case ID_IOCTL_POLL_INIT: //20
            elan_work_flag = 0;
            ELAN_DEBUG("[IOCTL] POLL INIT\n");
            break;

        case ID_IOCTL_INPUT_KEYCODE: //22
            keycode =(int __user)arg;
            ELAN_DEBUG("[IOCTL] KEYCODE DOWN & UP, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("Keycode %d not defined, ignored\n", (int __user)arg);
                break;
            }
            input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
            input_sync(fp->input_dev);
            input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
            input_sync(fp->input_dev);
            break;

        case ID_IOCTL_POLL_EXIT: //23
            elan_work_flag = 1;
            wake_up(&elan_poll_wq);
            ELAN_DEBUG("[IOCTL] POLL EXIT\n");
            break;

        case ID_IOCTL_SET_KEYCODE: //24
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] SET KEYCODE, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("Keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }
            input_set_capability(fp->input_dev, EV_KEY, keycode);
            break;

        case ID_IOCTL_READ_FACTORY_STATUS: //26
            mutex_lock(&elan_factory_mutex);
            ELAN_DEBUG("[IOCTL] READ factory_status = %d\n", factory_status);
            mutex_unlock(&elan_factory_mutex);
            return factory_status;
            break;

        case ID_IOCTL_WRITE_FACTORY_STATUS: //27
            mutex_lock(&elan_factory_mutex);
            factory_status = (int __user)arg;
            ELAN_DEBUG("[IOCTL] WRITE factory_status = %d\n", factory_status);
            mutex_unlock(&elan_factory_mutex);
            break;

        case ID_IOCTL_INPUT_KEYCODE_DOWN: //28
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] KEYCODE DOWN, keycode = %d \n", keycode);
            if(!keycode) {
                ELAN_DEBUG("Keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }
            input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
            input_sync(fp->input_dev);
            break;

        case ID_IOCTL_INPUT_KEYCODE_UP: //29
            keycode = (int __user)arg;
            ELAN_DEBUG("[IOCTL] KEYCODE UP, keycode = %d \n", keycode);
            if (!keycode) {
                ELAN_DEBUG("Keycode %d not defined, ignored\n", (int __user)arg);
                break ;
            }
            input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
            input_sync(fp->input_dev);
            break;

        case ID_IOCTL_INT_STATUS: //40
#if GPIO_PINCTRL
            fp->int_gpio = irq_to_gpio(fp->irq);
#endif
            return gpio_get_value(fp->int_gpio);

        case ID_IOCTL_WAKE_LOCK_UNLOCK: //41
            wake_lock_arg = (int __user)arg;
            if (!wake_lock_arg) {
                wake_unlock(&fp->hal_wake_lock);
                ELAN_DEBUG("[IOCTL] HAL WAKE UNLOCK = %d\n", wake_lock_arg);
            }
            else if (wake_lock_arg) {
                wake_lock(&fp->hal_wake_lock);
                ELAN_DEBUG("[IOCTL] HAL WAKE LOCK = %d\n", wake_lock_arg);
            }
            else
                ELAN_DEBUG("[IOCTL] ERROR WAKE LOCK ARGUMENT\n");
            break;

        case ID_IOCTL_EN_IRQ: //55
            elan_irq_enable(fp);
            ELAN_DEBUG("[IOCTL] ENABLE IRQ\n");
            break;

        case ID_IOCTL_DIS_IRQ: //66
            elan_irq_disable(fp);
            ELAN_DEBUG("[IOCTL] DISABLE IRQ\n");
            break;

        case ID_IOCTL_ENABLE_SPI_CLK: //77
            ELAN_DEBUG("[IOCTL] ENABLE SPI CLK\n");
            elan_spi_clk_enable(fp, 1);
            break;

        case ID_IOCTL_DISABLE_SPI_CLK: //88
            ELAN_DEBUG("[IOCTL] DISABLE SPI CLK\n");
            elan_spi_clk_enable(fp, 0);
            break;

        case ID_IOCTL_SET_IRQ_TYPE: //91
            ELAN_DEBUG("[IOCTL] SET IRQ TYPE\n");
            irq_set_irq_type(fp->irq, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
            break;

        case ID_IOCTL_DISPLAY_STATUS: //93
            ELAN_DEBUG("[IOCTL] DISPLAY_STATUS = %d\n", display_status);
            return display_status;

        case ID_IOCTL_DISPLAY_NOTIFY: //94
            display_notify_option = (int __user)arg;
            ELAN_DEBUG("[IOCTL] DISPLAY_NOTIFY = %d\n", display_notify_option);
#if defined(CONFIG_FB)
            if (display_notify_option && !register_fb_flag) {
                ELAN_DEBUG("[IOCTL] Register fb_notify\n");
                fb_notif.notifier_call = fb_notifier_callback;
                fb_register_client(&fb_notif);
                register_fb_flag = 1;
            }
#endif
            break;

        case ID_IOCTL_REQUEST_IRQ: //95
            ret = request_irq(fp->irq, elan_irq_handler,
                    IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                    fp->spi->dev.driver->name, fp);
            ELAN_DEBUG("[IOCTL Register irq ,ret = %d\n", ret);
            if (ret < 0) {
                ELAN_DEBUG("[IOCTL] REGISTER IRQ FAILED, ret = %d\n", ret);
                if (ret == -EBUSY)
                    ret = 99;
                else
                    ret = 100;

                return ret;
            }
            ELAN_DEBUG("[IOCTL] REGISTER IRQ SUCCESS, ret = %d\n", ret);
            break;

        case ID_IOCTL_FREE_IRQ: //96
            ELAN_DEBUG("[IOCTL] FREE IRQ\n");
            free_irq(fp->irq, fp);
            break;

        default:
            ELAN_DEBUG("INVALID COMMAND\n");
            break;
    }
    return 0;
}

static unsigned int elan_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    //ELAN_DEBUG("%s()\n",__func__);

    //wait_event_interruptible(elan_poll_wq, elan_work_flag > 0);
    poll_wait(file, &elan_poll_wq, wait);
    if (elan_work_flag > 0)
        mask = elan_work_flag;

    elan_work_flag = 0;
    return mask;
}

static int elan_open(struct inode *inode, struct file *filp)
{
    struct elan_data *fp = container_of(filp->private_data, struct elan_data, elan_dev);
    filp->private_data = fp;
    ELAN_DEBUG("%s()\n", __func__);
    return 0;
}

static int elan_close(struct inode *inode, struct file *filp)
{
    ELAN_DEBUG("%s()\n", __func__);
    return 0;
}

static const struct file_operations elan_fops = {
    .owner          = THIS_MODULE,
    .open           = elan_open,
    .unlocked_ioctl = elan_ioctl,
    .poll           = elan_poll,
    .release        = elan_close,
};

static irqreturn_t elan_irq_handler(int irq, void *_fp)
{
    struct elan_data *fp = (struct elan_data *)_fp;

    ELAN_DEBUG("%s()\n", __func__);
    if (fp == NULL)
        return IRQ_NONE;

    wake_lock_timeout(&fp->wake_lock,msecs_to_jiffies(1000));

#if NET_LINK
    sendnlmsg("7");
#endif

    elan_work_flag = 1;
    wake_up(&elan_poll_wq);

    return IRQ_HANDLED;
}

static int elan_setup_cdev(struct elan_data *fp)
{
    fp->elan_dev.minor = MISC_DYNAMIC_MINOR;
    fp->elan_dev.name = "elan_fp";
    fp->elan_dev.fops = &elan_fops;
    fp->elan_dev.mode = S_IFREG|S_IRWXUGO;
    if (misc_register(&fp->elan_dev) < 0) {
        ELAN_DEBUG("misc_register failed\n");
        return -1;
    }
    else {
        ELAN_DEBUG("misc_register finished\n");
    }
    return 0;
}

static int elan_sysfs_create(struct elan_data *sysfs)
{
    struct elan_data *fp = spi_get_drvdata(sysfs->spi);
    int ret = 0;

    /* Register sysfs */
    ret = sysfs_create_group(&fp->spi->dev.kobj, &elan_attr_group);
    if (ret) {
        ELAN_DEBUG("create sysfs attributes failed, ret = %d\n", ret);
        goto fail_un;
    }
    return 0;
fail_un:
    /* Remove sysfs */
    sysfs_remove_group(&fp->spi->dev.kobj, &elan_attr_group);

    return ret;
}

static int elan_gpio_config(struct elan_data *fp, struct device_node *np)
{
    int ret = 0;

#if GPIO_PINCTRL
    pinctrl_select_state(fp->elan_pinctrl, fp->eint_as_int);
    fp->irq = irq_of_parse_and_map(np, 0);
    ELAN_DEBUG("gpio to irq success, irq = %d\n",fp->irq);
    if (!fp->irq) {
        ELAN_DEBUG("irq_of_parse_and_map failed");
        ret = -1;
    }
#else
    // Configure INT GPIO (Input)
    ret = gpio_request(fp->int_gpio, "elan-irq");
    if (ret < 0)
        ELAN_DEBUG("interrupt pin request gpio failed, ret = %d\n", ret);
    else {
        gpio_direction_input(fp->int_gpio);
        fp->irq = gpio_to_irq(fp->int_gpio);
        if (fp->irq < 0) {
            ELAN_DEBUG("gpio to irq failed, irq = %d", fp->irq);
            ret = -1;
        }
        else
            ELAN_DEBUG("gpio to irq success, irq = %d\n",fp->irq);
    }

    // Configure RST GPIO (Output)
    ret = gpio_request(fp->rst_gpio, "elan-rst");
    if (ret < 0) {
        gpio_free(fp->int_gpio);
        free_irq(fp->irq, fp);
        ELAN_DEBUG("reset pin request gpio failed, ret = %d\n", ret);
    }
    else
        gpio_direction_output(fp->rst_gpio, 1);
#endif
    return ret;
}

static int elan_dts_init(struct elan_data *fp, struct device_node *np)
{
    int ret = 0;
    ELAN_DEBUG("%s()\n", __func__);
#if GPIO_PINCTRL
    fp->fp_rst_high = pinctrl_lookup_state(fp->elan_pinctrl, "fp_rst_high");
    if (IS_ERR(fp->fp_rst_high)) {
        ret = PTR_ERR(fp->fp_rst_high);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl fp_rst_high!\n");
        return ret;
    }
    fp->fp_rst_low = pinctrl_lookup_state(fp->elan_pinctrl, "fp_rst_low");
    if (IS_ERR(fp->fp_rst_low)) {
        ret = PTR_ERR(fp->fp_rst_low);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl fp_rst_low!\n");
        return ret;
    }
    fp->eint_as_int = pinctrl_lookup_state(fp->elan_pinctrl, "eint_as_int");
    if (IS_ERR(fp->eint_as_int)) {
        ret = PTR_ERR(fp->eint_as_int);
        dev_err(&fp->spi->dev, "fwq Cannot find fp pinctrl eint_as_int!\n");
        return ret;
    }

#else
    fp->rst_gpio = of_get_named_gpio(np, "elan,rst-gpio", 0);
    ELAN_DEBUG("rst_gpio = %d\n", fp->rst_gpio);
    if (fp->rst_gpio < 0)
        return fp->rst_gpio;

    fp->int_gpio = of_get_named_gpio(np, "elan,irq-gpio", 0);
    ELAN_DEBUG("int_gpio = %d\n", fp->int_gpio);
    if (fp->int_gpio < 0)
        return fp->int_gpio;
#endif
    return ret;
}

static int elan_probe(struct spi_device *spi)
{
    struct elan_data *fp = NULL;
    struct input_dev *input_dev = NULL;
    int ret = 0;

    ELAN_DEBUG("%s(), version = %s\n", __func__, VERSION_LOG);

#if NET_LINK
    netlink_unicast_init(); //initial netlink
#endif

    ret = spi_setup(spi);
    if (ret < 0)
        ELAN_DEBUG("spi_setup failed, ret = %d\n", ret);

    /* Allocate Device Data */
    fp = kzalloc(sizeof(struct elan_data), GFP_KERNEL);
    if (!fp)
        ELAN_DEBUG("kzmalloc elan data failed\n");

    /* Init Input Device */
    input_dev = input_allocate_device();
    if (!input_dev)
        ELAN_DEBUG("alloc input_dev failed\n");

    fp->spi = spi;
    spi_set_drvdata(spi, fp);

    input_dev->name = "elan";
    input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = &spi->dev;
    input_set_drvdata(input_dev, fp);

    input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
    input_set_capability(input_dev, EV_KEY, KEY_FP_INT); // change by customer, send key event to framework. KEY_xxx could be changed.
    input_set_capability(input_dev, EV_KEY, KEY_FP_INT2); // change by customer, send key event to framework. KEY_xxx could be changed.

    fp->input_dev = input_dev;

    /* Init Sysfs */
    ret = elan_sysfs_create(fp);
    if (ret < 0)
        ELAN_DEBUG("sysfs create failed, ret = %d\n", ret);

    /* Init Char Device */
    ret = elan_setup_cdev(fp);
    if (ret < 0)
        ELAN_DEBUG("setup device failed, ret = %d\n", ret);

    /* Register Input Device */
    ret = input_register_device(input_dev);
    if (ret)
        ELAN_DEBUG("register input device failed, ret = %d\n", ret);

    /*  For multiple fingerprint support. Point the compatible node to goodix-fp */
    /* spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,elan-fp"); */
    spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
#if GPIO_PINCTRL
    fp->elan_pinctrl = devm_pinctrl_get(&spi->dev);
	if (IS_ERR(fp->elan_pinctrl)) {
        ret = PTR_ERR(fp->elan_pinctrl);
        dev_err(&spi->dev, "fwq Cannot find fp elan_pinctrl!\n");
        return ret;
    }
    dev_err(&spi->dev, "find fp elan_pinctrl!\n");
#endif
    ret = elan_dts_init(fp, spi->dev.of_node);
    if (ret < 0)
        ELAN_DEBUG("device tree initial failed, ret = %d\n", ret);

    ret = elan_gpio_config(fp, spi->dev.of_node);
    if (ret < 0)
        ELAN_DEBUG("gpio config failed, ret = %d\n", ret);

    elan_reset(fp);

    wake_lock_init(&fp->wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");
    wake_lock_init(&fp->hal_wake_lock, WAKE_LOCK_SUSPEND, "hal_fp_wake_lock");

    ret = request_irq(fp->irq, elan_irq_handler,
            IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
            spi->dev.driver->name, fp);
    if (ret)
        ELAN_DEBUG("request irq failed, ret = %d\n", ret);

    irq_set_irq_wake(fp->irq, 1);
    spin_lock_init(&fp->irq_lock);

    ELAN_DEBUG("%s() End\n", __func__);
    return 0;
}

static int elan_remove(struct spi_device *spi)
{
    struct elan_data *fp = spi_get_drvdata(spi);

    if (fp->irq)
        free_irq(fp->irq, fp);

    gpio_free(fp->int_gpio);
    gpio_free(fp->rst_gpio);

    misc_deregister(&fp->elan_dev);
    input_free_device(fp->input_dev);

    kfree(fp);

    spi_set_drvdata(spi, NULL);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elan_suspend(struct device *dev)
{
    //ELAN_DEBUG("elan suspend!\n");
    return 0;
}

static int elan_resume(struct device *dev)
{
    //ELAN_DEBUG("elan resume!\n");
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(elan_pm_ops, elan_suspend, elan_resume);

#ifdef CONFIG_OF
static const struct of_device_id elan_of_match[] = {
    /*  For multiple fingerprint support. Point the compatible node to goodix-fp */
    { .compatible = "goodix,goodix-fp", },
    /* { .compatible = "elan,elan-fp", }, */
    {},
};
MODULE_DEVICE_TABLE(of, elan_of_match);
#endif

static struct spi_driver elan_driver = {
    .driver = {
        .name           = "elan_fp",
        .owner          = THIS_MODULE,
        .of_match_table = elan_of_match,
    },
    .probe      = elan_probe,
    .remove     = elan_remove,
};

static int __init elan_init(void)
{
    int ret = 0;
    ELAN_DEBUG("%s() Start\n", __func__);

    if(((strstr(saved_command_line, "androidboot.fp=enfp680rc_pda") == NULL)) || (strstr(saved_command_line,"androidboot.mode=charger") != NULL)) {
        pr_info("%s This FPM vendor is not elan.\n",__func__);
        return 0;
    }

    ret = spi_register_driver(&elan_driver);
    if (ret < 0)
        ELAN_DEBUG("%s FAIL !\n", __func__);

    ELAN_DEBUG("%s() End\n", __func__);
    return 0;
}

static void __exit elan_exist(void)
{
    spi_unregister_driver(&elan_driver);
}

module_init(elan_init);
module_exit(elan_exist);

MODULE_AUTHOR("ELAN");
MODULE_DESCRIPTION("fingerprint driver for tee");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
