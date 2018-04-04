// SPDX-License-Identifier: GPL-2.0
/*
 * Based on intel_crystalcove_pwrsrc.c
 *   Copyright (C) 2013 Intel Corporation
 */

#include <linux/extcon-provider.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define CRC_PWRSRC_IRQ		0x03
#define CRC_PWRSRC_IRQ_S0	0x0F
#define CRC_PWRSRC_IRQ_SX	0x10
#define CRC_PWRSRC_STS		0x1E
#define CRC_PWRSRC_VBUS		BIT(0)
#define CRC_PWRSRC_VBUSCNTL	0x6C

static const unsigned int crc_extcon_pwrsrc_cables[] = {
	EXTCON_USB,
	EXTCON_NONE,
};

struct crc_extcon_pwrsrc_data {
	struct device *dev;
	struct regmap *regmap;
	struct extcon_dev *edev;
};

static void crc_extcon_pwrsrc_event(struct crc_extcon_pwrsrc_data *ext)
{
	int ret, pwrsrc_sts;

	ret = regmap_read(ext->regmap, CRC_PWRSRC_STS, &pwrsrc_sts);
	if (ret) {
		dev_err(ext->dev, "Error reading pwrsrc status: %d\n", ret);
		return;
	}

	extcon_set_state_sync(ext->edev, EXTCON_USB, pwrsrc_sts & CRC_PWRSRC_VBUS);
}

static irqreturn_t crc_extcon_pwrsrc_isr(int irq, void *data)
{
	struct crc_extcon_pwrsrc_data *ext = data;
	int ret, irqs;

	ret = regmap_read(ext->regmap, CRC_PWRSRC_IRQ, &irqs);
	if (ret) {
		dev_err(ext->dev, "Error reading irqs: %d\n", ret);
		return IRQ_NONE;
	}

	crc_extcon_pwrsrc_event(ext);

	ret = regmap_write(ext->regmap, CRC_PWRSRC_IRQ, irqs);
	if (ret) {
		dev_err(ext->dev, "Error writing irqs: %d\n", ret);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int crc_extcon_pwrsrc_probe(struct platform_device *pdev)
{
	struct intel_soc_pmic *pmic = dev_get_drvdata(pdev->dev.parent);
	struct crc_extcon_pwrsrc_data *ext;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ext = devm_kzalloc(&pdev->dev, sizeof(*ext), GFP_KERNEL);
	if (!ext)
		return -ENOMEM;

	ext->dev = &pdev->dev;
	ext->regmap = pmic->regmap;

	/* Initialize extcon device */
	ext->edev = devm_extcon_dev_allocate(ext->dev, crc_extcon_pwrsrc_cables);
	if (IS_ERR(ext->edev))
		return PTR_ERR(ext->edev);

	/* Workaround: Set VBUS supply mode to HW control mode */
	regmap_write(ext->regmap, CRC_PWRSRC_VBUSCNTL, 0x00);

	/* Register extcon device */
	ret = devm_extcon_dev_register(ext->dev, ext->edev);
	if (ret) {
		dev_err(ext->dev, "Error registering extcon device: %d\n", ret);
		return ret;
	}

	/* Get initial state */
	crc_extcon_pwrsrc_event(ext);

	ret = devm_request_threaded_irq(ext->dev, irq, NULL, crc_extcon_pwrsrc_isr,
					IRQF_ONESHOT, pdev->name, ext);
	if (ret) {
		dev_err(ext->dev, "Error requesting interrupt: %d\n", ret);
		return ret;
	}

	/* Unmask irqs */
	ret = regmap_write(ext->regmap, CRC_PWRSRC_IRQ_S0, 0x00);
	if (ret) {
		dev_err(ext->dev, "Error writing irq-mask: %d\n", ret);
		return ret;
	}

	ret = regmap_write(ext->regmap, CRC_PWRSRC_IRQ_SX, 0x00);
	if (ret) {
		dev_err(ext->dev, "Error writing irq-mask: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, ext);
	return 0;
}

static struct platform_driver crc_extcon_pwrsrc_driver = {
	.probe = crc_extcon_pwrsrc_probe,
	.driver = {
		.name = "crystal_cove_pwrsrc",
	},
};
module_platform_driver(crc_extcon_pwrsrc_driver);

MODULE_AUTHOR("Lambdadroid <lambdadroid@gmail.com>");
MODULE_DESCRIPTION("Intel Baytrail CrystalCove Power Source extcon driver");
MODULE_LICENSE("GPL");
