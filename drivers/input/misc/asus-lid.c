#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio_keys.h>

struct gpio_keys_button asus_lid_button = {
	.desc = "asus_lid",
	.type = EV_SW,
	.code = SW_LID,
	.wakeup = 1,
	.active_low = 1,
	.debounce_interval = 50,
	.can_disable = 1,
};

struct gpio_keys_platform_data gpio_keys_pdata = {
	.name = "ASUS Lid Cover",
	.buttons = &asus_lid_button,
	.nbuttons = 1,
};

static int asus_lid_set_gpio(struct device *dev)
{
	struct gpio_desc *desc;

	desc = gpiod_get(dev, NULL, GPIOD_ASIS);
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	asus_lid_button.gpio = desc_to_gpio(desc);
	gpiod_put(desc);
	return 0;
}

static int asus_lid_probe(struct platform_device *pdev)
{
	struct platform_device *gpio_keys_pdev;
	int ret;

	ret = asus_lid_set_gpio(&pdev->dev);
	if (ret)
		return ret;

	gpio_keys_pdev = platform_device_alloc("gpio-keys", PLATFORM_DEVID_AUTO);
	if (!gpio_keys_pdev)
		return -ENOMEM;

	gpio_keys_pdev->dev.parent = &pdev->dev;
	platform_set_drvdata(pdev, gpio_keys_pdev);

	ret = platform_device_add_data(gpio_keys_pdev, &gpio_keys_pdata,
					sizeof(gpio_keys_pdata));
	if (ret)
		return ret;

	return platform_device_add(gpio_keys_pdev);
}

static int asus_lid_remove(struct platform_device *pdev)
{
	struct platform_device *gpio_keys_pdev = platform_get_drvdata(pdev);
	platform_device_unregister(gpio_keys_pdev);
	return 0;
}

static const struct acpi_device_id asus_lid_acpi_match[] = {
	{ "APX9131", 0 },
	{ "YOB8251", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, asus_lid_acpi_match);

static struct platform_driver asus_lid_driver = {
	.probe = asus_lid_probe,
	.remove = asus_lid_remove,
	.driver = {
		.name = "asus-lid",
		.acpi_match_table = ACPI_PTR(asus_lid_acpi_match),
	},
};
module_platform_driver(asus_lid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lambdadroid <lambdadroid@gmail.com>");
MODULE_DESCRIPTION("ASUS Lid Cover Driver");
