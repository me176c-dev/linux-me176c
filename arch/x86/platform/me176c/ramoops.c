#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/platform_device.h>
#include <linux/pstore_ram.h>
#include <linux/memblock.h>

#define RAMOOPS_MEM_SIZE	0x100000  /* 1 MiB */
#define RAMOOPS_MEM_ADDRESS	0x100000

static struct ramoops_platform_data me176c_ramoops_data = {
	.mem_size	= RAMOOPS_MEM_SIZE,
	.mem_address	= RAMOOPS_MEM_ADDRESS,
	.record_size	= 0x40000,
	.console_size	= 0x20000,
	.ftrace_size	= 0x20000,
	.pmsg_size	= 0x40000,
	.dump_oops	= 1,
};

static struct platform_device me176c_ramoops = {
	.name = "ramoops",
	.dev = {
		.platform_data = &me176c_ramoops_data,
	},
};

static __initdata bool enabled;

void __init me176c_ramoops_reserve(void)
{
	int ret = memblock_reserve(RAMOOPS_MEM_ADDRESS, RAMOOPS_MEM_SIZE);
	if (ret) {
		pr_err("Failed to reserve memory region from 0x%x to 0x%x: %d",
			RAMOOPS_MEM_ADDRESS, RAMOOPS_MEM_ADDRESS + RAMOOPS_MEM_SIZE - 1,
			ret);
	} else {
		enabled = true;
	}
}

static int __init me176c_ramoops_register(void)
{
	if (!enabled)
		return -ENODEV;

	return platform_device_register(&me176c_ramoops);
}
device_initcall(me176c_ramoops_register);
