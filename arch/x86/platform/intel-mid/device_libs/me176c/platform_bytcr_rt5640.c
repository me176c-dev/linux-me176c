#include <linux/gpio/machine.h>

static struct gpiod_lookup_table bytcr_rt5640_gpio_table = {
	.dev_id = "bytcr_rt5640",
	.table = {
		GPIO_LOOKUP("INT33FC:02", 27, "byt-soc-gpio", 0),
		{ },
	},
};

static int __init bytcr_rt5640_platform_init(void)
{
	gpiod_add_lookup_table(&bytcr_rt5640_gpio_table);
	return 0;
}
arch_initcall(bytcr_rt5640_platform_init);
