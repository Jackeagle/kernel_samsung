/*
 * SAMSUNG NFC Controller
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Woonki Lee <woonki84.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Last update: 2014-01-20
 *
 */
#ifdef CONFIG_SEC_NFC_IF_I2C_GPIO
#define CONFIG_SEC_NFC_IF_I2C
#endif

#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/nfc/sec_nfc.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SEC_NFC_CLK_REQ
#include <linux/interrupt.h>
#include <linux/clk.h>
#endif
#ifdef CONFIG_SEC_NFC_USE_8916_BBCLK2
#include <linux/clk.h>
#endif

#ifndef CONFIG_SEC_NFC_IF_I2C
struct sec_nfc_i2c_info {};
#define sec_nfc_read			NULL
#define sec_nfc_write			NULL
#define sec_nfc_poll			NULL
#define sec_nfc_i2c_irq_clear(x)

#define SEC_NFC_GET_INFO(dev) platform_get_drvdata(to_platform_device(dev))

#else /* CONFIG_SEC_NFC_IF_I2C */
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/i2c.h>

#define SEC_NFC_GET_INFO(dev) i2c_get_clientdata(to_i2c_client(dev))
enum sec_nfc_irq {
	SEC_NFC_NONE,
	SEC_NFC_INT,
	SEC_NFC_SKIP,
};

struct sec_nfc_i2c_info {
	struct i2c_client *i2c_dev;
	struct mutex read_mutex;
	enum sec_nfc_irq read_irq;
	wait_queue_head_t read_wait;
	size_t buflen;
	u8 *buf;
};
#endif

struct sec_nfc_info {
	struct miscdevice miscdev;
	struct mutex mutex;
	enum sec_nfc_mode mode;
	struct device *dev;
	struct sec_nfc_platform_data *pdata;
	struct sec_nfc_i2c_info i2c_info;
#ifdef CONFIG_SEC_NFC_CLK_REQ
	bool clk_ctl;
	bool clk_state;
#endif
	struct wake_lock wake_lock;
};

#ifdef CONFIG_SEC_NFC_IF_I2C
static irqreturn_t sec_nfc_irq_thread_fn(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;

	mutex_lock(&info->i2c_info.read_mutex);
#if 1	/* workaround for SPRD AP: do not read after power swiching */
	if (info->i2c_info.read_irq == SEC_NFC_SKIP) {
		dev_dbg(info->dev, "%s: Now power swiching. Skip this IRQ\n", __func__);
		mutex_unlock(&info->i2c_info.read_mutex);
		return IRQ_HANDLED;
	}
#endif
	info->i2c_info.read_irq = SEC_NFC_INT;
	mutex_unlock(&info->i2c_info.read_mutex);

	wake_up_interruptible(&info->i2c_info.read_wait);

	wake_lock_timeout(&info->wake_lock, 2 * HZ);
	return IRQ_HANDLED;
}

static ssize_t sec_nfc_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info,
						miscdev);
	enum sec_nfc_irq irq;
	int ret = 0;

	dev_dbg(info->dev, "[NFC] %s: info: %p, count: %zu\n", __func__,
		info, count);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "[NFC] sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	irq = info->i2c_info.read_irq;
	mutex_unlock(&info->i2c_info.read_mutex);
	if (irq == SEC_NFC_NONE) {
		if (file->f_flags & O_NONBLOCK) {
			dev_err(info->dev, "[NFC] it is nonblock\n");
			ret = -EAGAIN;
			goto out;
		}
	}

	/* i2c recv */
	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		dev_err(info->dev, "[NFC] user required wrong size :%d\n",
				count);
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	memset(info->i2c_info.buf, 0, count);
	ret = i2c_master_recv(info->i2c_info.i2c_dev, info->i2c_info.buf,
			count);
	pr_info("[NFC] %s : recv size : %d\n", __func__, ret);

	if (ret == -EREMOTEIO) {
		ret = -ERESTART;
		goto read_error;
	} else if (ret != count) {
		dev_err(info->dev, "[NFC] read failed: return: %d count: %d\n",
			ret, count);
		/* ret = -EREMOTEIO; */
		goto read_error;
	}

	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);

	if (copy_to_user(buf, info->i2c_info.buf, ret)) {
		dev_err(info->dev, "[NFC] copy failed to user\n");
		ret = -EFAULT;
	}

	goto out;

read_error:
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
out:
	mutex_unlock(&info->mutex);

	return ret;
}

static ssize_t sec_nfc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	int ret = 0;

	dev_dbg(info->dev, "[NFC] %s: info: %p, count %zu\n", __func__,
		info, count);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "[NFC] sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		dev_err(info->dev, "[NFC] user required wrong size :%d\n",
			count);
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(info->i2c_info.buf, buf, count)) {
		dev_err(info->dev, "[NFC] copy failed from user\n");
		ret = -EFAULT;
		goto out;
	}

#if 1	/* workaround for SPRD AP: do not read after power swiching */
	mutex_lock(&info->i2c_info.read_mutex);
	ret = i2c_master_send(info->i2c_info.i2c_dev, info->i2c_info.buf, count);
	if (info->i2c_info.read_irq == SEC_NFC_SKIP)
		info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
#else
	ret = i2c_master_send(info->i2c_info.i2c_dev, info->i2c_info.buf, count);
#endif

	if (ret == -EREMOTEIO) {
		dev_err(info->dev, "[NFC] send failed: return: %d count: %d\n",
		ret, count);
		ret = -ERESTART;
		goto out;
	}

	if (ret != count) {
		dev_err(info->dev, "[NFC] send failed: return: %d count: %d\n",
		ret, count);
		ret = -EREMOTEIO;
	}

out:
	mutex_unlock(&info->mutex);

	return ret;
}

static unsigned int sec_nfc_poll(struct file *file, poll_table *wait)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	enum sec_nfc_irq irq;

	int ret = 0;

	dev_dbg(info->dev, "[NFC] %s: info: %p\n", __func__, info);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "[NFC] sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	poll_wait(file, &info->i2c_info.read_wait, wait);

	mutex_lock(&info->i2c_info.read_mutex);
	irq = info->i2c_info.read_irq;
	if (irq == SEC_NFC_INT)
		ret = (POLLIN | POLLRDNORM);
	mutex_unlock(&info->i2c_info.read_mutex);

out:
	mutex_unlock(&info->mutex);

	return ret;
}

void sec_nfc_i2c_irq_clear(struct sec_nfc_info *info)
{
	/* clear interrupt. Interrupt will be occured at power off */
	mutex_lock(&info->i2c_info.read_mutex);
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
}

int sec_nfc_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct sec_nfc_platform_data *pdata = info->pdata;
	int ret;

	pr_info("[NFC] %s start\n", __func__);

	pr_info("info      : %p\n", info);
	pr_info("pdata     : %p\n", pdata);
	pr_info("pdata->irq: %d\n", pdata->irq);
	pr_info("client->irq:%d\n", client->irq);
	/* pdata->irq = client->irq;*/
	info->i2c_info.buflen = SEC_NFC_MAX_BUFFER_SIZE;
	info->i2c_info.buf = kzalloc(SEC_NFC_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!info->i2c_info.buf) {
		dev_err(dev,
		 "[NFC] failed to allocate memory for sec_nfc_info->buf\n");
		return -ENOMEM;
	}
	info->i2c_info.i2c_dev = client;
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_init(&info->i2c_info.read_mutex);
	init_waitqueue_head(&info->i2c_info.read_wait);

	i2c_set_clientdata(client, info);

	info->dev = dev;

	ret = request_threaded_irq(client->irq, NULL, sec_nfc_irq_thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, SEC_NFC_DRIVER_NAME,
			info);
	if (ret < 0) {
		dev_err(dev, "[NFC] failed to register IRQ handler\n");
		kfree(info->i2c_info.buf);
		return ret;
	}

	pr_info("[NFC] %s success\n", __func__);
	return 0;
}

void sec_nfc_i2c_remove(struct device *dev)
{
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_info.i2c_dev;
	struct sec_nfc_platform_data *pdata = info->pdata;
	free_irq(client->irq, info);
	gpio_free(pdata->irq);
}
#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef CONFIG_SEC_NFC_CLK_REQ

#ifdef CONFIG_SEC_NFC_USE_8916_BBCLK2
/* PMIC */
#define sec_nfc_clk_on(clk)			clk_prepare_enable(clk)
#define sec_nfc_clk_off(clk)			clk_disable_unprepare(clk)

#else
/* default GPIO */
#define sec_nfc_clk_on(clk)			gpio_set_value(clk, 1)
#define sec_nfc_clk_off(clk)			gpio_set_value(clk, 0)
#endif

static irqreturn_t sec_nfc_clk_irq_thread(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;
	struct sec_nfc_platform_data *pdata = info->pdata;
	bool value;

	value = gpio_get_value(pdata->clk_req) > 0 ? false : true;

	if (value == info->clk_state)
		return IRQ_HANDLED;

	/* pr_info("[NFC] %s: %s\n", __func__, value ? "True" : "False");*/
	if (value)
		sec_nfc_clk_on(pdata->clk_enable);
	else
		sec_nfc_clk_off(pdata->clk_enable);

	info->clk_state = value;

	return IRQ_HANDLED;
}

void sec_nfc_clk_ctl_enable(struct sec_nfc_info *info)
{
	struct sec_nfc_platform_data *pdata = info->pdata;
	int ret;

	pr_info("[NFC] %s : start\n", __func__);

	if (info->clk_ctl)
		return;
	info->clk_state = false;
	pdata->clk_irq = gpio_to_irq(pdata->clk_req);
	pr_info("[NFC] %s : goio to irq = %d\n", __func__, pdata->clk_irq);
	ret = request_threaded_irq(pdata->clk_irq, NULL, sec_nfc_clk_irq_thread,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT,
			SEC_NFC_DRIVER_NAME, info);
	if (ret < 0)
		dev_err(info->dev, "[NFC] failed to register CLK REQ IRQ handler\n");
	info->clk_ctl = true;
}

void sec_nfc_clk_ctl_disable(struct sec_nfc_info *info)
{
	struct sec_nfc_platform_data *pdata = info->pdata;

	pr_info("[NFC] %s : start\n", __func__);

	if (!info->clk_ctl)
		return;

	free_irq(pdata->clk_irq, info);
	pr_info("[NFC] %s : free irq = %d\n", __func__, pdata->clk_irq);
	if (info->clk_state)
		sec_nfc_clk_off(pdata->clk_enable);

	info->clk_state = false;
	info->clk_ctl = false;
}
#else
#define sec_nfc_clk_ctl_enable(x)
#define sec_nfc_clk_ctl_disable(x)
#endif /* CONFIG_SEC_NFC_CLK_REQ */

static void sec_nfc_set_mode(struct sec_nfc_info *info,
					enum sec_nfc_mode mode)
{
	struct sec_nfc_platform_data *pdata = info->pdata;

	pr_info("[NFC] %s : start %d\n", __func__, mode);
	/* intfo lock is aleady getten before calling this function */
	info->mode = mode;
#if 1	/* workaround for SPRD AP: do not read after power swiching */
#ifdef CONFIG_SEC_NFC_IF_I2C
	mutex_lock(&info->i2c_info.read_mutex);
	info->i2c_info.read_irq = SEC_NFC_SKIP;
	mutex_unlock(&info->i2c_info.read_mutex);
#endif
#endif
	gpio_set_value(pdata->ven, SEC_NFC_PW_OFF);
	if (pdata->firm)
		gpio_set_value(pdata->firm, SEC_NFC_FW_OFF);

	if (mode == SEC_NFC_MODE_BOOTLOADER)
		if (pdata->firm)
			gpio_set_value(pdata->firm, SEC_NFC_FW_ON);

	if (mode != SEC_NFC_MODE_OFF) {
		msleep(SEC_NFC_VEN_WAIT_TIME);
		gpio_set_value_cansleep(pdata->ven, 1);
		gpio_set_value(pdata->ven, SEC_NFC_PW_ON);
		sec_nfc_clk_ctl_enable(info);
		msleep(SEC_NFC_VEN_WAIT_TIME/2);
#ifdef CONFIG_SEC_NFC_IF_I2C
		enable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
	} else {
		sec_nfc_clk_ctl_disable(info);
#ifdef CONFIG_SEC_NFC_IF_I2C
		disable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
	}

//	sec_nfc_i2c_irq_clear(info);
	dev_dbg(info->dev, "[NFC] Power mode is : %d\n", mode);
}

static long sec_nfc_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
#ifdef CONFIG_SEC_NFC_PRODUCT_N5
	struct sec_nfc_platform_data *pdata = info->pdata;
#endif
	unsigned int new = (unsigned int)arg;
	int ret = 0;

	dev_dbg(info->dev, "[NFC] %s: info: %p, cmd: 0x%x\n",
			__func__, info, cmd);

	mutex_lock(&info->mutex);

	switch (cmd) {
	case SEC_NFC_SET_MODE:
		dev_dbg(info->dev, "[NFC] %s: SEC_NFC_SET_MODE\n", __func__);

		if (info->mode == new)
			break;

		if (new >= SEC_NFC_MODE_COUNT) {
			dev_err(info->dev, "[NFC] wrong mode (%d)\n", new);
			ret = -EFAULT;
			break;
		}
		sec_nfc_set_mode(info, new);

		break;

#ifdef CONFIG_SEC_NFC_PRODUCT_N3
	case SEC_NFC_SLEEP:
	case SEC_NFC_WAKEUP:
		break;

#elif defined(CONFIG_SEC_NFC_PRODUCT_N5)
	case SEC_NFC_SLEEP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			if(wake_lock_active(&info->wake_lock))
				wake_unlock(&info->wake_lock);
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_SLEEP);
		}
		break;

	case SEC_NFC_WAKEUP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_UP);
			if(!wake_lock_active(&info->wake_lock))
				wake_lock(&info->wake_lock);
		}
		break;
#endif
	default:
		dev_err(info->dev, "[NFC] Unknow ioctl 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_open(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	int ret = 0;

	pr_info("[NFC] %s : start\n", __func__);
	dev_dbg(info->dev, "[NFC] %s: info : %p\n" , __func__, info);

	mutex_lock(&info->mutex);
	if (info->mode != SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "[NFC] sec_nfc is busy\n");
		ret = -EBUSY;
		goto out;
	}

	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
out:
	mutex_unlock(&info->mutex);
	return ret;
}

static int sec_nfc_close(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);

	pr_info("[NFC] %s : start\n", __func__);
	dev_dbg(info->dev, "[NFC] %s: info : %p\n" , __func__, info);

	mutex_lock(&info->mutex);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	mutex_unlock(&info->mutex);

	return 0;
}

static const struct file_operations sec_nfc_fops = {
	.owner		= THIS_MODULE,
	.read		= sec_nfc_read,
	.write		= sec_nfc_write,
	.poll		= sec_nfc_poll,
	.open		= sec_nfc_open,
	.release	= sec_nfc_close,
	.unlocked_ioctl	= sec_nfc_ioctl,
};

#ifdef CONFIG_PM
static int sec_nfc_suspend(struct device *dev)
{
	struct sec_nfc_info *info = SEC_NFC_GET_INFO(dev);
	int ret = 0;

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_BOOTLOADER)
		ret = -EPERM;

	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sec_nfc_pm_ops, sec_nfc_suspend, sec_nfc_resume);
#endif

#ifdef CONFIG_OF
/*device tree parsing*/
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	int gpio;
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;
	pdata->ven = of_get_named_gpio_flags(np, "sec-nfc,ven-gpio",
		0, &pdata->ven_gpio_flags);
	pdata->firm = of_get_named_gpio_flags(np, "sec-nfc,firm-gpio",
		0, &pdata->firm_gpio_flags);
	pdata->wake = pdata->firm;
	gpio = of_get_named_gpio_flags(np, "sec-nfc,irq-gpio",
		0, &flags);
	pdata->irq = gpio_to_irq(gpio);
	if (!pdata->irq)
		pr_err("[NFC] %s : nfc irq fail\n", __func__);
#ifdef CONFIG_SEC_NFC_CLK_REQ
	pdata->clk_enable = of_get_named_gpio_flags(np, "sec-nfc,ext_clk-gpio",
		0, &pdata->irq_gpio_flags);
#endif

#ifdef CONFIG_SEC_NFC_USE_8916_BBCLK2
	pdata->nfc_clk = clk_get(dev, "nfc_clock");
	if (IS_ERR(pdata->nfc_clk)) {
		pr_err("[NFC] %s: Couldn't get D1)\n",
					__func__);
	} else {
		if (clk_prepare_enable(pdata->nfc_clk))
			pr_err("[NFC] %s: Couldn't prepare D1\n",
					__func__);
	}
#endif
	pr_info("[NFC] %s : result\n", __func__);
	pr_info("      ven-gpio\t= %d\n", pdata->ven);
	pr_info("      firm-gpio\t= %d\n", pdata->firm);
	pr_info("      wake-gpio\t= %d\n", pdata->wake);
	pr_info("      irq-gpio\t= %d\n", gpio);
	pr_info("      irq num\t= %d\n", pdata->irq);

	pr_info("[NFC] %s : done\n", __func__);
	return 0;
}

static int sec_nfc_pinctrl(struct device *dev)
{
	int ret = 0;
	struct pinctrl *nfc_pinctrl;
	struct pinctrl_state *nfc_suspend;
	struct pinctrl_state *nfc_active;

	/* Get pinctrl if target uses pinctrl */
	nfc_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(nfc_pinctrl)) {
		pr_debug("Target does not use pinctrl\n");
		nfc_pinctrl = NULL;
	} else {
		nfc_suspend = pinctrl_lookup_state(nfc_pinctrl, "nfc_suspend");
		nfc_active = pinctrl_lookup_state(nfc_pinctrl, "nfc_active");
		if (IS_ERR(nfc_suspend)) {
			pr_info("%s fail to suspend lookup_state\n", __func__);
			goto err_exit;
		}
		if (IS_ERR(nfc_active)) {
			pr_info("%s fail to active lookup_state\n", __func__);
			goto err_exit;
		}
		ret = pinctrl_select_state(nfc_pinctrl, nfc_suspend);
		if (ret != 0) {
			pr_err("%s: fail to select_state suspend\n", __func__);
			goto err_exit;
		}
		ret = pinctrl_select_state(nfc_pinctrl, nfc_active);
		if (ret != 0) {
			pr_err("%s: fail to select_state active\n", __func__);
			goto err_exit;
		}
		devm_pinctrl_put(nfc_pinctrl);
	}

err_exit:
	return ret;
}

#else
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	return -ENODEV;
}

static int sec_nfc_pinctrl(struct device *dev)
{
	return 0;
}
#endif

static int __sec_nfc_probe(struct device *dev)
{
	struct sec_nfc_info *info;
	struct sec_nfc_platform_data *pdata = NULL;
	int ret = 0;

	pr_info("[NFC] %s : start\n", __func__);
	if (dev->of_node) {
		pdata = devm_kzalloc(dev,
			sizeof(struct sec_nfc_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(dev, "[NFC] Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = sec_nfc_parse_dt(dev, pdata);
		if (ret)
			return ret;
	} else
		pdata = dev->platform_data;

	if (!pdata) {
		dev_err(dev, "[NFC] No platform data\n");
		ret = -ENOMEM;
		goto err_pdata;
	}

	ret = sec_nfc_pinctrl(dev);
	if (ret) {
		dev_err(dev, "[NFC] failed to nfc pinctrl\n");
		goto err_pinctrl;
	}

	info = kzalloc(sizeof(struct sec_nfc_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "[NFC] failed to allocate memory for sec_nfc_info\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}
	info->dev = dev;
	info->pdata = pdata;
	info->mode = SEC_NFC_MODE_OFF;

	mutex_init(&info->mutex);
	dev_set_drvdata(dev, info);

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = SEC_NFC_DRIVER_NAME;
	info->miscdev.fops = &sec_nfc_fops;
	info->miscdev.parent = dev;
	ret = misc_register(&info->miscdev);
	if (ret < 0) {
		dev_err(dev, "[NFC] failed to register Device\n");
		goto err_dev_reg;
	}

	ret = gpio_request(pdata->ven, "ven-gpio");
	if (ret) {
		dev_err(dev, "[NFC] failed to get gpio ven\n");
		goto err_gpio_ven;
	}
	gpio_direction_output(pdata->ven, SEC_NFC_PW_OFF);

	if (pdata->firm) {
		ret = gpio_request(pdata->firm, "firm-gpio");
		if (ret) {
			dev_err(dev, "[NFC] failed to get gpio firm\n");
			goto err_gpio_firm;
		}
		gpio_direction_output(pdata->firm, SEC_NFC_FW_OFF);
	}

	wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");

	dev_dbg(dev, "[NFC] %s: info: %p, pdata %p\n", __func__, info, pdata);
	pr_err("[NFC] %s : success\n", __func__);

	return 0;

err_gpio_firm:
	gpio_free(pdata->ven);
err_gpio_ven:
err_dev_reg:
err_pinctrl:
err_info_alloc:
	kfree(info);
err_pdata:
	return ret;
}

static int __sec_nfc_remove(struct device *dev)
{
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct sec_nfc_platform_data *pdata = info->pdata;

	dev_dbg(info->dev, "%s\n", __func__);
#ifdef CONFIG_SEC_NFC_USE_8916_BBCLK2
	if (pdata->nfc_clk)
		clk_unprepare(pdata->nfc_clk);
#endif
	misc_deregister(&info->miscdev);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	gpio_set_value(pdata->firm, 0);
	gpio_set_value_cansleep(pdata->ven, 0);
	gpio_free(pdata->ven);

	if (pdata->firm)
		gpio_free(pdata->firm);

	wake_lock_destroy(&info->wake_lock);

	kfree(info);

	return 0;
}

#ifdef CONFIG_SEC_NFC_IF_I2C
MODULE_DEVICE_TABLE(i2c, sec_nfc_id_table);
static int sec_nfc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;

	ret = __sec_nfc_probe(&client->dev);
	if (ret)
		return ret;

	if (sec_nfc_i2c_probe(client))
		__sec_nfc_remove(&client->dev);

	return ret;
}

static int sec_nfc_remove(struct i2c_client *client)
{
	sec_nfc_i2c_remove(&client->dev);
	return __sec_nfc_remove(&client->dev);
}

static struct i2c_device_id sec_nfc_id_table[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};

#else	/* CONFIG_SEC_NFC_IF_I2C */
MODULE_DEVICE_TABLE(platform, sec_nfc_id_table);
static int sec_nfc_probe(struct platform_device *pdev)
{
	return __sec_nfc_probe(&pdev->dev);
}

static int sec_nfc_remove(struct platform_device *pdev)
{
	return __sec_nfc_remove(&pdev->dev);
}

static struct platform_device_id sec_nfc_id_table[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};

#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef CONFIG_OF
static struct of_device_id nfc_match_table[] = {
	{ .compatible = SEC_NFC_DRIVER_NAME,},
	{},
};
#else
#define nfc_match_table NULL
#endif

#ifdef CONFIG_SEC_NFC_IF_I2C

static struct i2c_driver sec_nfc_driver = {
	.probe = sec_nfc_probe,
	.id_table = sec_nfc_id_table,
	.remove = sec_nfc_remove,
	.driver = {
		.name = SEC_NFC_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &sec_nfc_pm_ops,
#endif
		.of_match_table = nfc_match_table,
	},
};

#else	/* CONFIG_SEC_NFC_IF_I2C */

static struct platform_driver sec_nfc_driver = {
	.probe = sec_nfc_probe,
	.id_table = sec_nfc_id_table,
	.remove = sec_nfc_remove,
	.driver = {
		.name = SEC_NFC_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &sec_nfc_pm_ops,
#endif
		.of_match_table = nfc_match_table,
	},
};

#endif /* CONFIG_SEC_NFC_IF_I2C */

static int __init sec_nfc_init(void)
{
#ifdef CONFIG_SEC_NFC_IF_I2C
	return i2c_add_driver(&sec_nfc_driver);
#else
	return platform_driver_register(&sec_nfc_driver);
#endif
}

static void __exit sec_nfc_exit(void)
{
#ifdef CONFIG_SEC_NFC_IF_I2C
	return i2c_del_driver(&sec_nfc_driver);
#else
	return platform_driver_unregister(&sec_nfc_driver);
#endif
}

module_init(sec_nfc_init);
module_exit(sec_nfc_exit);

MODULE_DESCRIPTION("Samsung sec_nfc driver");
MODULE_LICENSE("GPL");


