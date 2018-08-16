// SPDX-License-Identifier: GPL-2.0
/**
 * Dummy driver to power off the Intel Atom Image Signal Processor
 * device on Baytrail devices.
 * This is necessary to reach the S0ix states.
 *
 * Based on the original atomisp code
 *   Copyright (c) 2010-2017 Intel Corporation. All Rights Reserved.
 *   Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <asm/iosf_mbi.h>

#define PCI_DEVICE_ID_INTEL_BYT	0x0f38

#define PCI_INTERRUPT_CTRL	0x9C
#define INTR_IER			24
#define INTR_IIR			16

#define MRFLD_PCI_CSI_CONTROL			0xe8
#define MRFLD_ALL_CSI_PORTS_OFF_MASK	0x7

#define MRFLD_ISPSSPM0					0x39
#define MRFLD_ISPSSPM0_ISPSSC_MASK		0x3
#define MRFLD_ISPSSPM0_IUNIT_POWER_ON	0x0
#define MRFLD_ISPSSPM0_IUNIT_POWER_OFF	0x3
#define MRFLD_ISPSSPM0_ISPSSS_OFFSET	24

static int atomisp_dummy_probe(struct pci_dev *pci, const struct pci_device_id *id)
{
	pci->d3_delay = 0;
	pci->d3cold_delay = 0;

	pm_runtime_put(&pci->dev);
	return 0;
}

static void atomisp_dummy_remove(struct pci_dev *pci)
{
	pm_runtime_get(&pci->dev);
}

static const struct pci_device_id atomisp_dummy_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT) },
	{  }
};
MODULE_DEVICE_TABLE(pci, dwc3_pci_id_table);

#ifdef CONFIG_PM
static int atomisp_dummy_power_iunit(struct device *dev, int state)
{
	unsigned long timeout;
	u32 reg;

	iosf_mbi_read(BT_MBI_UNIT_PMC, MBI_REG_READ, MRFLD_ISPSSPM0, &reg);
	reg &= ~MRFLD_ISPSSPM0_ISPSSC_MASK;
	reg |= state;
	iosf_mbi_write(BT_MBI_UNIT_PMC, MBI_REG_WRITE, MRFLD_ISPSSPM0, reg);

	/* Wait until the iunit is powered on/off */
	timeout = jiffies + msecs_to_jiffies(50);
	do {
		iosf_mbi_read(BT_MBI_UNIT_PMC, MBI_REG_READ, MRFLD_ISPSSPM0, &reg);
		if ((reg >> MRFLD_ISPSSPM0_ISPSSS_OFFSET) == state)
			return 0;

		usleep_range(100, 150);
	} while (time_before(jiffies, timeout));

	dev_err(dev, "Changing iunit power state timed out");
	return -EBUSY;
}

static int atomisp_dummy_suspend(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	u32 reg;

	/* atomisp_mrfld_pre_power_down */
	pci_read_config_dword(pci, PCI_INTERRUPT_CTRL, &reg);
	reg &= 1 << INTR_IIR;
	pci_write_config_dword(pci, PCI_INTERRUPT_CTRL, reg);
	pci_read_config_dword(pci, PCI_INTERRUPT_CTRL, &reg);
	reg &= ~(1 << INTR_IER);
	pci_write_config_dword(pci, PCI_INTERRUPT_CTRL, reg);

	/* atomisp_ospm_dphy_down */
	pci_read_config_dword(pci, MRFLD_PCI_CSI_CONTROL, &reg);
	reg |= MRFLD_ALL_CSI_PORTS_OFF_MASK;
	pci_write_config_dword(pci, MRFLD_PCI_CSI_CONTROL, reg);

	/* atomisp_mrfld_power_down */
	return atomisp_dummy_power_iunit(dev, MRFLD_ISPSSPM0_IUNIT_POWER_OFF);
}

static int atomisp_dummy_resume(struct device *dev)
{
	/* atomisp_mrfld_power_up */
	return atomisp_dummy_power_iunit(dev, MRFLD_ISPSSPM0_IUNIT_POWER_ON);
}
#endif /* CONFIG_PM */

static UNIVERSAL_DEV_PM_OPS(atomisp_dummy_dev_pm_ops,
							atomisp_dummy_suspend, atomisp_dummy_resume, NULL);

static struct pci_driver atomisp_dummy_driver = {
	.name		= "atomisp-dummy",
	.id_table	= atomisp_dummy_id_table,
	.probe		= atomisp_dummy_probe,
	.remove		= atomisp_dummy_remove,
	.driver		= {
		.pm	= &atomisp_dummy_dev_pm_ops,
	}
};
module_pci_driver(atomisp_dummy_driver);

MODULE_AUTHOR("lambdadroid <lambdadroid@gmail.com>");
MODULE_DESCRIPTION("Intel Atom Image Signal Processor Dummy Driver");
MODULE_LICENSE("GPL");
