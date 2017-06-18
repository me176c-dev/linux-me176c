/**
 * tusb1210.c - TUSB1210 USB ULPI PHY driver
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/ulpi/driver.h>
#include <linux/ulpi/regs.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/extcon.h>

#include "ulpi_phy.h"

#define TUSB1210_VENDOR_SPECIFIC2		0x80
#define TUSB1210_VENDOR_SPECIFIC2_IHSTX_SHIFT	0
#define TUSB1210_VENDOR_SPECIFIC2_ZHSDRV_SHIFT	4
#define TUSB1210_VENDOR_SPECIFIC2_DP_SHIFT	6

#define PWRSRC_DRV_NAME			"crystal_cove_pwrsrc"

// Taken from https://www.ti.com/lit/ds/symlink/tusb1211.pdf
#define TUSB1211_POWER_CONTROL			0x3D
#define TUSB1211_VENDOR_SPECIFIC3		0x85
#define TUSB1211_VENDOR_SPECIFIC3_SET	0x86
#define POWER_CONTROL_DET_COMP			(1 << 1)
#define VENDOR_SPECIFIC3_SW_USB_DET		(1 << 4)

extern void bq24192_notify_cable_type(int usb_state);

struct tusb1210 {
	struct ulpi *ulpi;
	struct phy *phy;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_cs;
	u8 vendor_specific2;

	bool current_state;
	struct notifier_block	vbus_nb;
	struct work_struct	charger_work;
};

static int tusb1210_power_on(struct phy *phy)
{
	struct tusb1210 *tusb = phy_get_drvdata(phy);

	gpiod_set_value_cansleep(tusb->gpio_reset, 1);
	gpiod_set_value_cansleep(tusb->gpio_cs, 1);

	/* Restore the optional eye diagram optimization value */
	if (tusb->vendor_specific2)
		ulpi_write(tusb->ulpi, TUSB1210_VENDOR_SPECIFIC2,
			   tusb->vendor_specific2);

	return 0;
}

static int tusb1210_power_off(struct phy *phy)
{
	struct tusb1210 *tusb = phy_get_drvdata(phy);

	gpiod_set_value_cansleep(tusb->gpio_reset, 0);
	gpiod_set_value_cansleep(tusb->gpio_cs, 0);

	return 0;
}

static int tusb1210_set_mode(struct phy *phy, enum phy_mode mode)
{
	struct tusb1210 *tusb = phy_get_drvdata(phy);
	int ret;

	ret = ulpi_read(tusb->ulpi, ULPI_OTG_CTRL);
	if (ret < 0)
		return ret;

	switch (mode) {
	case PHY_MODE_USB_HOST:
		ret |= (ULPI_OTG_CTRL_DRVVBUS_EXT
			| ULPI_OTG_CTRL_ID_PULLUP
			| ULPI_OTG_CTRL_DP_PULLDOWN
			| ULPI_OTG_CTRL_DM_PULLDOWN);
		ulpi_write(tusb->ulpi, ULPI_OTG_CTRL, ret);
		ret |= ULPI_OTG_CTRL_DRVVBUS;
		break;
	case PHY_MODE_USB_DEVICE:
		ret &= ~(ULPI_OTG_CTRL_DRVVBUS
			 | ULPI_OTG_CTRL_DP_PULLDOWN
			 | ULPI_OTG_CTRL_DM_PULLDOWN);
		ulpi_write(tusb->ulpi, ULPI_OTG_CTRL, ret);
		ret &= ~ULPI_OTG_CTRL_DRVVBUS_EXT;
		break;
	default:
		/* nothing */
		return 0;
	}

	return ulpi_write(tusb->ulpi, ULPI_OTG_CTRL, ret);
}

static void tusb1210_reset(struct tusb1210 *tusb)
{
	gpiod_set_value_cansleep(tusb->gpio_reset, 0);
	usleep_range(200, 500);
	gpiod_set_value_cansleep(tusb->gpio_reset, 1);
	msleep(30);

	/* Restore the optional eye diagram optimization value */
	if (tusb->vendor_specific2)
		ulpi_write(tusb->ulpi, TUSB1210_VENDOR_SPECIFIC2,
			   tusb->vendor_specific2);
}

static void tusb1211_update_charger_type(struct tusb1210 *tusb)
{
	int ret;

	// Allow the PHY to finish the charger detection
	msleep(200);

	// Read POWER_CONTROL register
	ret = ulpi_read(tusb->ulpi, TUSB1211_POWER_CONTROL);
	if (ret < 0) {
		dev_err(&tusb->ulpi->dev,
			"Failed to read POWER_CONTROL register for charger detection: %d\n",
			ret);
		ret = 0; // Detect as SDP
	}

	// This is quite naive, but should do the job for now
	// TODO: Implement with proper extcon device
	if (ret & POWER_CONTROL_DET_COMP) {
		// Charging port detected => DCP (TODO: CDP)
		dev_info(&tusb->ulpi->dev, "Detected charger type: DCP\n");
		bq24192_notify_cable_type(EXTCON_CHG_USB_DCP);
	} else {
		// No charging port detected => SDP
		dev_info(&tusb->ulpi->dev, "Detected charger type: SDP\n");
		bq24192_notify_cable_type(EXTCON_CHG_USB_SDP);
	}

	// Reset chip to disable charger detection
	// For some reason we cannot clear the bit without causing the PHY to freeze
	tusb1210_reset(tusb);

	// Set to device mode
	tusb1210_set_mode(tusb->phy, PHY_MODE_USB_DEVICE);
}

static void tusb1211_disable_charging(struct tusb1210 *tusb)
{
	int ret;

	dev_info(&tusb->ulpi->dev, "Detected charger type: NONE\n");
	bq24192_notify_cable_type(EXTCON_NONE);

	// Enable charger detection for next connection
	ret = ulpi_write(tusb->ulpi, TUSB1211_VENDOR_SPECIFIC3_SET,
				VENDOR_SPECIFIC3_SW_USB_DET);

	if (ret) {
		dev_err(&tusb->ulpi->dev,
			"Failed to enable charger detection: %d\n", ret);
	}
}

static void tusb1211_update_charger_state(struct work_struct *work)
{
	struct tusb1210 *tusb = container_of(work, struct tusb1210, charger_work);

	if (tusb->current_state) {
		tusb1211_update_charger_type(tusb);
	} else {
		tusb1211_disable_charging(tusb);
	}
}

static void tusb1211_schedule_charger_check(struct tusb1210 *tusb, bool state)
{
	tusb->current_state = state;
	schedule_work(&tusb->charger_work);
}

static int tusb1211_vbus_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct tusb1210 *tusb = container_of(nb, struct tusb1210, vbus_nb);
	tusb1211_schedule_charger_check(tusb, event);
	return NOTIFY_DONE;
}

static const struct phy_ops phy_ops = {
	.power_on = tusb1210_power_on,
	.power_off = tusb1210_power_off,
	.set_mode = tusb1210_set_mode,
	.owner = THIS_MODULE,
};

static int tusb1210_probe(struct ulpi *ulpi)
{
	struct tusb1210 *tusb;
	struct extcon_dev *edev;
	int ret;
	u8 val, reg;

	tusb = devm_kzalloc(&ulpi->dev, sizeof(*tusb), GFP_KERNEL);
	if (!tusb)
		return -ENOMEM;
	tusb->ulpi = ulpi;

	tusb->gpio_reset = devm_gpiod_get_optional(&ulpi->dev, "reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(tusb->gpio_reset))
		return PTR_ERR(tusb->gpio_reset);

	gpiod_set_value_cansleep(tusb->gpio_reset, 1);

	tusb->gpio_cs = devm_gpiod_get_optional(&ulpi->dev, "cs",
						GPIOD_OUT_LOW);
	if (IS_ERR(tusb->gpio_cs))
		return PTR_ERR(tusb->gpio_cs);

	gpiod_set_value_cansleep(tusb->gpio_cs, 1);

	edev = extcon_get_extcon_dev(PWRSRC_DRV_NAME);
	if (IS_ERR(edev)) {
		return PTR_ERR(edev);
	}

	if (edev) {
		tusb->vbus_nb.notifier_call = tusb1211_vbus_notifier;
		ret = devm_extcon_register_notifier(&ulpi->dev, edev, EXTCON_USB,
						&tusb->vbus_nb);
		if (!ret) {
			// Schedule detection for current state
			INIT_WORK(&tusb->charger_work, tusb1211_update_charger_state);
			tusb1211_schedule_charger_check(tusb, extcon_get_state(edev, EXTCON_USB));
		} else {
			dev_err(&ulpi->dev,
				"Failed to register extcon notifier for charger detection: %d\n",
				ret);
		}
	}

	/*
	 * VENDOR_SPECIFIC2 register in TUSB1210 can be used for configuring eye
	 * diagram optimization and DP/DM swap.
	 */

	/* High speed output drive strength configuration */
	device_property_read_u8(&ulpi->dev, "ihstx", &val);
	reg = val << TUSB1210_VENDOR_SPECIFIC2_IHSTX_SHIFT;

	/* High speed output impedance configuration */
	device_property_read_u8(&ulpi->dev, "zhsdrv", &val);
	reg |= val << TUSB1210_VENDOR_SPECIFIC2_ZHSDRV_SHIFT;

	/* DP/DM swap control */
	device_property_read_u8(&ulpi->dev, "datapolarity", &val);
	reg |= val << TUSB1210_VENDOR_SPECIFIC2_DP_SHIFT;

	if (reg) {
		ulpi_write(ulpi, TUSB1210_VENDOR_SPECIFIC2, reg);
		tusb->vendor_specific2 = reg;
	}

	tusb->phy = ulpi_phy_create(ulpi, &phy_ops);
	if (IS_ERR(tusb->phy))
		return PTR_ERR(tusb->phy);

	phy_set_drvdata(tusb->phy, tusb);
	ulpi_set_drvdata(ulpi, tusb);
	return 0;
}

static void tusb1210_remove(struct ulpi *ulpi)
{
	struct tusb1210 *tusb = ulpi_get_drvdata(ulpi);

	ulpi_phy_destroy(ulpi, tusb->phy);
}

#define TI_VENDOR_ID 0x0451

static const struct ulpi_device_id tusb1210_ulpi_id[] = {
	{ TI_VENDOR_ID, 0x1507, },  /* TUSB1210 */
	{ TI_VENDOR_ID, 0x1508, },  /* TUSB1211 */
	{ },
};
MODULE_DEVICE_TABLE(ulpi, tusb1210_ulpi_id);

static struct ulpi_driver tusb1210_driver = {
	.id_table = tusb1210_ulpi_id,
	.probe = tusb1210_probe,
	.remove = tusb1210_remove,
	.driver = {
		.name = "tusb1210",
		.owner = THIS_MODULE,
	},
};

module_ulpi_driver(tusb1210_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TUSB1210 ULPI PHY driver");
