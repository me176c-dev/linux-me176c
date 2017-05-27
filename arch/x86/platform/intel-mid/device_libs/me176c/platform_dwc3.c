#include <linux/gpio/machine.h>

static struct gpiod_lookup_table dwc3_pci_gpio_table = {
	.dev_id = "0000:00:16.0",
	.table = {
		GPIO_LOOKUP("INT33FC:00", 54, "cs", 0),
		GPIO_LOOKUP("INT33FC:02", 14, "reset", 0),
		{ },
	},
};

static struct gpiod_lookup_table dwc3_ulpi_gpio_table = {
	.dev_id = "dwc3.1.auto.ulpi",
	.table = {
		GPIO_LOOKUP("INT33FC:00", 54, "cs", 0),
		GPIO_LOOKUP("INT33FC:02", 14, "reset", 0),
		{ },
	},
};

static int __init dwc3_platform_init(void)
{
	gpiod_add_lookup_table(&dwc3_pci_gpio_table);
	gpiod_add_lookup_table(&dwc3_ulpi_gpio_table);
	return 0;
}
arch_initcall(dwc3_platform_init);
