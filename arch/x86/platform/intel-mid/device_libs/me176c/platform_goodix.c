#include <linux/gpio/machine.h>

// ME176C doesn't have the GPIO interrupt for the Goodix touchscreen in ACPI
// This is a workaround to register the correct GPIO port for the driver.
static struct gpiod_lookup_table goodix_gpio_table = {
	.dev_id = "i2c-GDIX1001:00",
	.table = {
		GPIO_LOOKUP("INT33FC:02", 28, "irq", 0),
		{ },
	},
};

static int __init goodix_platform_init(void)
{
	gpiod_add_lookup_table(&goodix_gpio_table);
	return 0;
}
arch_initcall(goodix_platform_init);
