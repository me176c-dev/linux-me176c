// SPDX-License-Identifier: GPL-2.0-only
/**
 * tusb1210.c - TUSB1210 USB ULPI PHY driver
 *
 * Copyright (C) 2015 Intel Corporation
 *
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */
#include <linux/module.h>
#include <linux/ulpi/driver.h>
#include <linux/ulpi/regs.h>
#include <linux/gpio/consumer.h>
#include <linux/phy/ulpi_phy.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/extcon-provider.h>

#define TUSB1210_VENDOR_SPECIFIC2		0x80
#define TUSB1210_VENDOR_SPECIFIC2_IHSTX_SHIFT	0
#define TUSB1210_VENDOR_SPECIFIC2_ZHSDRV_SHIFT	4
#define TUSB1210_VENDOR_SPECIFIC2_DP_SHIFT	6

/* Taken from https://www.ti.com/lit/ds/symlink/tusb1211.pdf */
#define TUSB1211_POWER_CONTROL			0x3d
#define TUSB1211_VENDOR_SPECIFIC3		0x85
#define TUSB1211_VENDOR_SPECIFIC3_SET	0x86
#define POWER_CONTROL_DET_COMP			(1 << 1)
#define VENDOR_SPECIFIC3_SW_USB_DET		(1 << 4)

struct tusb1210 {
	struct ulpi *ulpi;
	struct phy *phy;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_cs;
	u8 vendor_specific2;

	enum phy_mode current_mode;

	struct extcon_dev *edev;
	struct notifier_block vbus_nb;
	struct work_struct charger_work;
	bool vbus_state;
	int current_cable;
};

static const unsigned int tusb1211_extcon_cables[] = {
	EXTCON_CHG_USB_SDP,
	EXTCON_CHG_USB_DCP,
	EXTCON_NONE,
};

static void tusb1210_setup(struct tusb1210 *tusb)
{
	/* Wait a bit until initialization is complete */
	msleep(30);

	/* Restore the optional eye diagram optimization value */
	if (tusb->vendor_specific2)
		ulpi_write(tusb->ulpi, TUSB1210_VENDOR_SPECIFIC2,
			   tusb->vendor_specific2);
}

static int tusb1210_power_on(struct phy *phy)
{
	struct tusb1210 *tusb = phy_get_drvdata(phy);

	gpiod_set_value_cansleep(tusb->gpio_reset, 1);
	gpiod_set_value_cansleep(tusb->gpio_cs, 1);

	tusb1210_setup(tusb);
	return 0;
}

static int tusb1210_power_off(struct phy *phy)
{
	struct tusb1210 *tusb = phy_get_drvdata(phy);

	gpiod_set_value_cansleep(tusb->gpio_reset, 0);
	gpiod_set_value_cansleep(tusb->gpio_cs, 0);

	return 0;
}

static void tusb1210_reset(struct tusb1210 *tusb)
{
	gpiod_set_value_cansleep(tusb->gpio_reset, 0);
	usleep_range(200, 500);
	gpiod_set_value_cansleep(tusb->gpio_reset, 1);

	tusb1210_setup(tusb);
}

static int tusb1210_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct tusb1210 *tusb = phy_get_drvdata(phy);
	int ret;

	ret = ulpi_read(tusb->ulpi, ULPI_OTG_CTRL);
	if (ret < 0)
		return ret;

	tusb->current_mode = mode;

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

static void tusb1211_update_charger_type(struct tusb1210 *tusb, int new_cable)
{
	if (tusb->current_cable == new_cable)
		return;

	if (new_cable != EXTCON_NONE)
		extcon_set_state_sync(tusb->edev, new_cable, true);
	if (tusb->current_cable != EXTCON_NONE)
		extcon_set_state_sync(tusb->edev, tusb->current_cable, false);

	tusb->current_cable = new_cable;
}

static void tusb1211_detect_charger_type(struct tusb1210 *tusb)
{
	int ret, new_cable;

	/* Power on USB controller */
	ret = pm_runtime_get_sync(tusb->ulpi->dev.parent);
	if (ret < 0) {
		dev_err(&tusb->phy->dev,
			"Failed to power on USB controller for charger detection: %d\n",
			ret);
		return;
	}

	mutex_lock(&tusb->phy->mutex);

	/* Enable charger detection */
	ret = ulpi_write(tusb->ulpi, TUSB1211_VENDOR_SPECIFIC3_SET,
				VENDOR_SPECIFIC3_SW_USB_DET);
	if (ret) {
		dev_err(&tusb->phy->dev,
			"Failed to enable charger detection: %d\n", ret);
		ret = 0; /* Detect as SDP */
		goto err;
	}

	/*
	 * Allow the PHY to finish the charger detection
	 * TODO: Is there a way to check if charger detection is complete?
	 */
	msleep(500);

	/* Read POWER_CONTROL register */
	ret = ulpi_read(tusb->ulpi, TUSB1211_POWER_CONTROL);
	if (ret < 0) {
		dev_err(&tusb->phy->dev,
			"Failed to read POWER_CONTROL register for charger detection: %d\n",
			ret);
		ret = 0; /* Detect as SDP */
	}

err:
	/*
	 * This is quite naive (e.g. doesn't handle CDP and ACA properly),
	 * but should do the job for now.
	 */
	if (ret & POWER_CONTROL_DET_COMP) {
		/* Charging port detected => DCP */
		dev_info(&tusb->phy->dev, "Detected charger type: DCP\n");
		new_cable = EXTCON_CHG_USB_DCP;
	} else {
		/* No charging port detected => SDP */
		dev_info(&tusb->phy->dev, "Detected charger type: SDP\n");
		new_cable = EXTCON_CHG_USB_SDP;
	}

	/*
	 * Reset chip to disable charger detection
	 * For some reason we cannot clear the bit without causing the PHY
	 * to freeze...
	 */
	tusb1210_reset(tusb);

	/* Restore mode */
	tusb1210_set_mode(tusb->phy, tusb->current_mode, 0);

	/* Update state on the Extcon device */
	tusb1211_update_charger_type(tusb, new_cable);

	mutex_unlock(&tusb->phy->mutex);

	pm_runtime_put(tusb->ulpi->dev.parent);
}

static void tusb1211_no_charger(struct tusb1210 *tusb)
{
	dev_info(&tusb->phy->dev, "Detected charger type: NONE\n");
	tusb1211_update_charger_type(tusb, EXTCON_NONE);
}

static void tusb1211_update_charger_state(struct work_struct *work)
{
	struct tusb1210 *tusb = container_of(work, struct tusb1210, charger_work);

	if (tusb->vbus_state) {
		tusb1211_detect_charger_type(tusb);
	} else {
		tusb1211_no_charger(tusb);
	}
}

static inline void tusb1211_schedule_charger_check(
	struct tusb1210 *tusb, bool state)
{
	tusb->vbus_state = state;
	schedule_work(&tusb->charger_work);
}

static int tusb1211_vbus_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct tusb1210 *tusb = container_of(nb, struct tusb1210, vbus_nb);
	tusb1211_schedule_charger_check(tusb, event);
	return NOTIFY_OK;
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
	struct extcon_dev *vbus_edev;
	int ret;
	u8 val, reg;
	const char *extcon_name;

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

	/* Enable charger detection only for TUSB1211 */
	if (ulpi->id.product != 0x1508)
		return 0;

	/* Charger detection depends on an Extcon devices for VBUS notifications */
	if (device_property_read_string(&ulpi->dev,
			"extcon-vbus-name", &extcon_name))
		return 0;

	/* Lookup Extcon device */
	vbus_edev = extcon_get_extcon_dev(extcon_name);
	if (!vbus_edev) {
		dev_err(&tusb->phy->dev, "VBUS extcon device '%s' does not exist\n",
					extcon_name);
		return -ENODEV;
	}
	if (IS_ERR(vbus_edev))
		return PTR_ERR(vbus_edev);

	/* Setup new extcon device for results */
	tusb->edev = devm_extcon_dev_allocate(&tusb->phy->dev,
						tusb1211_extcon_cables);
	if (IS_ERR(tusb->edev))
		return PTR_ERR(tusb->edev);

	ret = devm_extcon_dev_register(&tusb->phy->dev, tusb->edev);
	if (ret) {
		dev_err(&tusb->phy->dev,
			"Error registering extcon device: %d\n", ret);
		return ret;
	}

	INIT_WORK(&tusb->charger_work, tusb1211_update_charger_state);

	/* Setup notifier */
	tusb->vbus_nb.notifier_call = tusb1211_vbus_notifier;
	ret = devm_extcon_register_notifier(&tusb->phy->dev, vbus_edev,
					EXTCON_USB, &tusb->vbus_nb);
	if (ret) {
		dev_err(&tusb->phy->dev,
			"Failed to register VBUS extcon notifier: %d\n", ret);
	}

	/* Detect initial state */
	tusb1211_schedule_charger_check(tusb,
			extcon_get_state(vbus_edev, EXTCON_USB));

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
