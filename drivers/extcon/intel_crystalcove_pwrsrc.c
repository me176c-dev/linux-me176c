/*
 * intel_crystalcove_pwrsrc.c - Intel Crystal Cove Power Source Detect Driver
 *
 * Copyright (C) 2013 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Kannappan R <r.kannappan@intel.com>
 *	Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/usb/phy.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/regmap.h>
#include <linux/mfd/intel_soc_pmic.h>

#define CRYSTALCOVE_PWRSRCIRQ_REG	0x03
#define CRYSTALCOVE_MPWRSRCIRQS0_REG	0x0F
#define CRYSTALCOVE_MPWRSRCIRQSX_REG	0x10
#define CRYSTALCOVE_SPWRSRC_REG		0x1E
#define CRYSTALCOVE_RESETSRC0_REG	0x20
#define CRYSTALCOVE_RESETSRC1_REG	0x21
#define CRYSTALCOVE_WAKESRC_REG		0x22

#define PWRSRC_VBUS_DET			(1 << 0)
#define PWRSRC_DCIN_DET			(1 << 1)
#define PWRSRC_BAT_DET			(1 << 2)

#define CRYSTALCOVE_VBUSCNTL_REG	0x6C
#define VBUSCNTL_EN			(1 << 0)
#define VBUSCNTL_SEL			(1 << 1)

#define PWRSRC_DRV_NAME			"crystal_cove_pwrsrc"

static const unsigned int byt_extcon_cable[] = {
	EXTCON_CHG_AC,
	EXTCON_USB,
	EXTCON_NONE,
};

struct pwrsrc_info {
	struct platform_device *pdev;
	int irq;
	struct extcon_dev *edev;
	struct regmap *regmap;
};

static char *pwrsrc_resetsrc0_info[] = {
	/* bit 0 */ "Last shutdown caused by SOC reporting a thermal event",
	/* bit 1 */ "Last shutdown caused by critical PMIC temperature",
	/* bit 2 */ "Last shutdown caused by critical system temperature",
	/* bit 3 */ "Last shutdown caused by critical battery temperature",
	/* bit 4 */ "Last shutdown caused by VSYS under voltage",
	/* bit 5 */ "Last shutdown caused by VSYS over voltage",
	/* bit 6 */ "Last shutdown caused by battery removal",
	NULL,
};

static char *pwrsrc_resetsrc1_info[] = {
	/* bit 0 */ "Last shutdown caused by VCRIT threshold",
	/* bit 1 */ "Last shutdown caused by BATID reporting battery removal",
	/* bit 2 */ "Last shutdown caused by user pressing the power button",
	NULL,
};

static char *pwrsrc_wakesrc_info[] = {
	/* bit 0 */ "Last wake caused by user pressing the power button",
	/* bit 1 */ "Last wake caused by a battery insertion",
	/* bit 2 */ "Last wake caused by a USB charger insertion",
	/* bit 3 */ "Last wake caused by an adapter insertion",
	NULL,
};

/* Decode and log the given "reset source indicator" register, then clear it */
static void crystalcove_pwrsrc_log_rsi(struct pwrsrc_info *info,
					char **pwrsrc_rsi_info,
					unsigned int reg_s)
{
	char *rsi_info = pwrsrc_rsi_info[0];
	int ret;
	unsigned int val, i = 0;
	unsigned int bit_select, clear_mask = 0x0;

	ret = regmap_read(info->regmap, reg_s, &val);
	if (ret) {
		dev_err(&info->pdev->dev, "Failed to read from regmap: %d\n", ret);
		return;
	}

	while (rsi_info) {
		bit_select = (1 << i);
		if (val & bit_select) {
			dev_info(&info->pdev->dev, "%s\n", rsi_info);
			clear_mask |= bit_select;
		}
		rsi_info = pwrsrc_rsi_info[++i];
	}

	/* Clear the register value for next reboot (write 1 to clear bit) */
	regmap_write(info->regmap, reg_s, clear_mask);
}

static void handle_pwrsrc_event(struct pwrsrc_info *info, unsigned int pwrsrcirq)
{
	unsigned int spwrsrc;
	int ret;

	ret = regmap_read(info->regmap, CRYSTALCOVE_SPWRSRC_REG, &spwrsrc);
	if (ret)
		goto pmic_read_fail;

	if (pwrsrcirq & PWRSRC_VBUS_DET) {
		if (spwrsrc & PWRSRC_VBUS_DET) {
			dev_dbg(&info->pdev->dev, "VBUS attach event\n");
			extcon_set_state_sync(info->edev, EXTCON_USB, true);
		} else {
			dev_dbg(&info->pdev->dev, "VBUS detach event\n");
			extcon_set_state_sync(info->edev, EXTCON_USB, false);
		}
	} else if (pwrsrcirq & PWRSRC_DCIN_DET) {
		if (spwrsrc & PWRSRC_DCIN_DET) {
			dev_dbg(&info->pdev->dev, "ADP attach event\n");
			extcon_set_state_sync(info->edev, EXTCON_CHG_AC, true);
		} else {
			dev_dbg(&info->pdev->dev, "ADP detach event\n");
			extcon_set_state_sync(info->edev, EXTCON_CHG_AC, false);
		}
	} else if (pwrsrcirq & PWRSRC_BAT_DET) {
		if (spwrsrc & PWRSRC_BAT_DET)
			dev_dbg(&info->pdev->dev, "Battery attach event\n");
		else
			dev_dbg(&info->pdev->dev, "Battery detach event\n");
	} else {
		dev_dbg(&info->pdev->dev, "event none or spurious\n");
	}

	return;

pmic_read_fail:
	dev_err(&info->pdev->dev, "SPWRSRC read failed:%d\n", ret);
	return;
}

static irqreturn_t crystalcove_pwrsrc_isr(int irq, void *data)
{
	struct pwrsrc_info *info = data;
	int ret;
	unsigned int pwrsrcirq;

	ret = regmap_read(info->regmap, CRYSTALCOVE_PWRSRCIRQ_REG, &pwrsrcirq);
	if (ret) {
		dev_err(&info->pdev->dev, "PWRSRCIRQ read failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	dev_dbg(&info->pdev->dev, "pwrsrcirq=%x\n", pwrsrcirq);
	handle_pwrsrc_event(info, pwrsrcirq);

	regmap_write(info->regmap, CRYSTALCOVE_PWRSRCIRQ_REG, pwrsrcirq);
	return IRQ_HANDLED;
}

static int pwrsrc_extcon_registration(struct pwrsrc_info *info)
{
	int ret = 0;

	/* register with extcon */
	info->edev = devm_extcon_dev_allocate(&info->pdev->dev, byt_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_err(&info->pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(&info->pdev->dev, info->edev);
	if (ret) {
		dev_err(&info->pdev->dev, "extcon registration failed!!\n");
		return ret;
	}

	/* Workaround: Set VBUS supply mode to HW control mode */
	regmap_write(info->regmap, CRYSTALCOVE_VBUSCNTL_REG, 0x00);

	return ret;
}

static int crystalcove_pwrsrc_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	struct pwrsrc_info *info;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->irq = platform_get_irq(pdev, 0);
	info->regmap = pmic->regmap;
	platform_set_drvdata(pdev, info);

	/* Log reason for last reset and wake events */
	crystalcove_pwrsrc_log_rsi(info, pwrsrc_resetsrc0_info,
				CRYSTALCOVE_RESETSRC0_REG);
	crystalcove_pwrsrc_log_rsi(info, pwrsrc_resetsrc1_info,
				CRYSTALCOVE_RESETSRC1_REG);
	crystalcove_pwrsrc_log_rsi(info, pwrsrc_wakesrc_info,
				CRYSTALCOVE_WAKESRC_REG);

	ret = pwrsrc_extcon_registration(info);
	if (ret)
		return ret;

	// Check current state
	handle_pwrsrc_event(info, PWRSRC_VBUS_DET | PWRSRC_DCIN_DET);

	ret = devm_request_threaded_irq(&info->pdev->dev, info->irq, NULL,
				crystalcove_pwrsrc_isr,
				IRQF_ONESHOT | IRQF_NO_SUSPEND,
				PWRSRC_DRV_NAME, info);
	if (ret) {
		dev_err(&pdev->dev, "unable to register irq %d\n", info->irq);
		return ret;
	}

	/* unmask the PWRSRC interrupts */
	regmap_write(info->regmap, CRYSTALCOVE_MPWRSRCIRQS0_REG, 0x00);
	regmap_write(info->regmap, CRYSTALCOVE_MPWRSRCIRQSX_REG, 0x00);

	return 0;
}

#ifdef CONFIG_PM
static int crystalcove_pwrsrc_suspend(struct device *dev)
{
	return 0;
}

static int crystalcove_pwrsrc_resume(struct device *dev)
{
	return 0;
}
#else
#define crystalcove_pwrsrc_suspend		NULL
#define crystalcove_pwrsrc_resume		NULL
#endif

static const struct dev_pm_ops crystalcove_pwrsrc_driver_pm_ops = {
	.suspend	= crystalcove_pwrsrc_suspend,
	.resume		= crystalcove_pwrsrc_resume,
};

static struct platform_driver crystalcove_pwrsrc_driver = {
	.probe = crystalcove_pwrsrc_probe,
	.driver = {
		.name = PWRSRC_DRV_NAME,
		.pm = &crystalcove_pwrsrc_driver_pm_ops,
	},
};

static int __init crystalcove_pwrsrc_init(void)
{
	return platform_driver_register(&crystalcove_pwrsrc_driver);
}
device_initcall(crystalcove_pwrsrc_init);

static void __exit crystalcove_pwrsrc_exit(void)
{
	platform_driver_unregister(&crystalcove_pwrsrc_driver);
}
module_exit(crystalcove_pwrsrc_exit);

MODULE_AUTHOR("Kannappan R <r.kannappan@intel.com>");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("CrystalCove Power Source Detect Driver");
MODULE_LICENSE("GPL");
