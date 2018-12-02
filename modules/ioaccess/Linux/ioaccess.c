/*
 * Driver to allow user-space processes to access PCI and Physical memory
 * Based on uhid module - David Herrmann
 * Copyright (c) <year>, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/list.h>

#include <linux/dma-mapping.h> /* DMA related stuff*/
#include <linux/highmem.h> /* kmap and kunmap*/
#include <linux/pci.h> /* pci stuff */
#include <linux/uaccess.h>  /* copy_to_user() */
#include <linux/compiler.h> /* __must_check */

#include "IoAccessInterface.h"

#define IOACCESS_INFO(args...) printk(KERN_INFO "ioaccess: " args)
#define IOACCESS_ERR(args...)  printk(KERN_ERR  "ioaccess: " args)

/*===========================================================================*/
static loff_t ioaccess_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch (whence) {

	case 0: /* SEEK_SET */
		newpos = off;
		break;

	case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;

	default: /* can't happen */
		return -EINVAL;
	}

	if (newpos < 0)
		return -EINVAL;

	filp->f_pos = newpos;
	return newpos;
}
static ssize_t ioaccess_read(struct file *file, char __user *buffer,
	size_t count, loff_t *ppos)
{
	struct page *p = NULL;
	loff_t i = *ppos;
	size_t initial_offset = 0;
	int to_be_copied = 0;
	void *v = NULL;
	size_t copied = 0;
	char *kernel_buffer = NULL;
	void __iomem *virt_address = 0;
	size_t index = 0;

	IOACCESS_INFO(
		"Device file is read at offset = %lld, read bytes count = %u\n",
		(loff_t)*ppos,
		(unsigned int)count);

	if (i < 0) {
		return -EFAULT;
	}

	virt_address = ioremap(i, count);
	if (virt_address != NULL) {

		IOACCESS_INFO("iomap virt_address = %p\n", virt_address);

		kernel_buffer = kmalloc(count, GFP_KERNEL);
		if (kernel_buffer == NULL) {
			IOACCESS_ERR("kmalloc failed");
			iounmap(virt_address);
			return -EFAULT;
		}

		while (index < count) {
			*(kernel_buffer + index) =
				ioread8(virt_address + index);
			/*IOACCESS_INFO("io byte %d = %d\n",
			index, *(kernel_buffer + index));*/
			++index;
		}

		if (copy_to_user(buffer, kernel_buffer, count) != 0) {

			IOACCESS_ERR("copy_to_user failed");
			kfree(kernel_buffer);
			iounmap(virt_address);
			return -EFAULT;
		}
		iounmap(virt_address);
		kfree(kernel_buffer);

		*ppos += count;
		return count;
	} else {
		IOACCESS_INFO("Trying kernel map");
		while (count > 0) {
			p = pfn_to_page((i) >> PAGE_SHIFT);
			initial_offset = i - (i >> PAGE_SHIFT << PAGE_SHIFT);
			to_be_copied =
				min(
					(size_t)PAGE_SIZE - initial_offset,
					(size_t)count);

			IOACCESS_INFO(
				"Initial offset = %x, to be copied = %x\n",
				initial_offset,
				to_be_copied);

			v = kmap(p);

			IOACCESS_INFO("Virtual Memory = %p, physical = %llx\n",
				v,
				(uint64_t)i);

			if (
				copy_to_user(
					buffer,
					v + initial_offset,
					to_be_copied) != 0) {

				kunmap(p);
				IOACCESS_ERR("copy_to_user failed\n");
				return -EFAULT;
			}

			kunmap(p);

			i += to_be_copied;
			buffer += to_be_copied;
			count -= to_be_copied;
			copied += to_be_copied;
		}

		*ppos += copied;
		return copied;
	}
}
static ssize_t ioaccess_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	struct page *p = NULL;
	loff_t i = *ppos;
	size_t initial_offset = 0;
	int to_be_copied = 0;
	void *v = NULL;
	size_t copied = 0;
	void __iomem *virt_address = 0;
	char *kernel_buffer = NULL;
	size_t index = 0;

	IOACCESS_INFO(
		"Device file is write at offset = %lld, "
		"written bytes count = %u\n",
		(loff_t)*ppos,
		(unsigned int)count);

	if (i < 0) {
		return -EFAULT;
	}

	virt_address = ioremap(i, count);
	if (virt_address != NULL) {

		IOACCESS_INFO("iomap virt_address = %p\n", virt_address);

		kernel_buffer = kmalloc(count, GFP_KERNEL);
		if (kernel_buffer == NULL) {
			IOACCESS_ERR("kmalloc failed");
			iounmap(virt_address);
			return -EFAULT;
		}

		if (copy_from_user(kernel_buffer, buffer, count) != 0) {
			iounmap(virt_address);
			kfree(kernel_buffer);
			return -EFAULT;
		}

		while (index < count) {
			iowrite8(
				*(kernel_buffer + index), virt_address + index);
			++index;
		}

		iounmap(virt_address);
		kfree(kernel_buffer);

		*ppos += count;
		return count;
	} else {
		IOACCESS_INFO("Trying kernel map");
		while (count > 0) {

			p = pfn_to_page((i) >> PAGE_SHIFT);
			initial_offset = i - (i >> PAGE_SHIFT << PAGE_SHIFT);
			to_be_copied =
				min(
					(size_t) PAGE_SIZE - initial_offset,
					(size_t)count);

			IOACCESS_INFO("Initial offset = %x, to be copied = %x\n"
				, initial_offset
				, to_be_copied);

			v = kmap(p);

			IOACCESS_INFO("Virtual Memory = %p, physical = %llx\n"
				, v
				, (uint64_t)i);

			if (copy_from_user(
					v + initial_offset,
					buffer, to_be_copied) != 0) {

				kunmap(p);
				IOACCESS_ERR("copy_from_user failed\n");
				return -EFAULT;
			}

			kunmap(p);

			i += to_be_copied;
			buffer	+= to_be_copied;
			count -= to_be_copied;
			copied += to_be_copied;
		}

		*ppos += copied;
		return copied;
	}
}
/*===========================================================================*/
static int ioaccess_device_count(void __user *argp)
{
	int res = 0;
	uint32_t num_of_devices = 0;
	struct pci_dev *pdev = NULL;

	if (copy_from_user(&num_of_devices, argp, sizeof(num_of_devices))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	while (true) {
		pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pdev);
		if (pdev == NULL) {
			break;
		}
		++num_of_devices;
	}

	if (copy_to_user(argp, &num_of_devices, sizeof(num_of_devices))) {
		IOACCESS_ERR("Unable to copy output to user");
		res = -EFAULT;
		goto ioaccess_device_count_exit;
	}

	res = 0;

ioaccess_device_count_exit:

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	return res;
}
static int ioaccess_device_list(void __user *argp)
{
	int res = 0;
	struct PciDeviceList req;
	struct PciDeviceList *resp = NULL;
	struct pci_dev *pdev = NULL;
	int count = 0;

	if (copy_from_user(&req, argp, sizeof(req))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	resp = (struct PciDeviceList *)kmalloc(
		sizeof(req) + req.listSize * sizeof(struct PciDeviceList),
		GFP_KERNEL);
	if (resp == NULL) {
		IOACCESS_ERR(
			"kmalloc failed. Requested list size %d\n",
			req.listSize);
		return -EFAULT;
	}

	while (count < req.listSize) {

		struct PciDevice_ *dev = resp->list + count;
		pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pdev);
		if (pdev == NULL) {
			break;
		}

		dev->vendorId = pdev->vendor;
		dev->deviceId = pdev->device;
		dev->bus = pdev->bus->number;
		dev->device = PCI_SLOT(pdev->devfn);
	    dev->function = PCI_FUNC(pdev->devfn);

		IOACCESS_INFO(
			"PCI device %x:%x found at bus %x - device %x - function %x.\n",
			dev->vendorId, dev->deviceId,
			dev->bus, dev->device, dev->function);

		++count;
	}

	resp->listSize = count;

	if (copy_to_user(
			argp,
			resp,
			sizeof(req) +
				resp->listSize *
				sizeof(struct PciDevice_))) {

		IOACCESS_ERR("Unable to copy output to user");
		res = -EFAULT;
		goto ioaccess_device_list_exit;
	}

	res = 0;

ioaccess_device_list_exit:

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	if (resp != NULL) {
		kfree(resp);
	}
	return res;
}
static int ioaccess_discover_device(void __user *argp)
{
	int res = 0;
	struct PciDevice_ req;
	struct PciDevice_ resp;
	struct pci_dev *pdev = NULL;

	if (copy_from_user(&req, argp, sizeof(req))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	pdev = pci_get_device(req.vendorId, req.deviceId, pdev);
	if (pdev == NULL) {
		IOACCESS_ERR(
			"PCI device %x:%x not found.\n",
			req.vendorId, req.deviceId);
		res = -ENODEV;
		goto ioaccess_discover_device_exit;
	}

	resp.vendorId = req.vendorId;
	resp.deviceId = req.deviceId;
	resp.bus = pdev->bus->number;
	resp.device = PCI_SLOT(pdev->devfn);
	resp.function = PCI_FUNC(pdev->devfn);

	IOACCESS_INFO(
		"PCI device %x:%x found at bus %x - device %x - function %x.\n",
		req.vendorId, req.deviceId,
		resp.bus, resp.device, resp.function);

	if (copy_to_user(argp, &resp, sizeof(resp))) {
		IOACCESS_ERR("Unable to copy output to user");
		res = -EFAULT;
		goto ioaccess_discover_device_exit;
	}

	res = 0;

ioaccess_discover_device_exit:

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	return res;
}
static int ioaccess_read_pci_config_space(void __user *argp)
{
	int res = 0;
	struct PciData req;
	struct pci_dev *pdev = NULL;
	uint8_t bus, device, function;
	uint16_t offset;

	if (copy_from_user(&req, argp, sizeof(req))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	bus = req.bus;
	device = req.device;
	function = req.function;
	offset = req.offset;

	pdev = pci_get_domain_bus_and_slot(0, bus, PCI_DEVFN(device, function));
	if (pdev == NULL) {
		IOACCESS_ERR(
			"PCI device at bus %x - dev %x - function %x not found.\n",
			bus, device, function);
		res = -ENODEV;
		goto ioaccess_read_pci_config_space_exit;
	}

	res = pci_read_config_dword(pdev, offset, &req.data);
	if (res < 0) {
		IOACCESS_ERR(
			"Error in reading PCI device at bus %x - dev %x - function %x."
			"Config space offset %x\n",
			bus, device, function, offset);
		res = -EFAULT;
		goto ioaccess_read_pci_config_space_exit;
	}

	IOACCESS_INFO(
		"PCI device at bus %x - dev %x - function %x. "
		"Config space offset %x - read data %x\n",
		bus, device, function, offset, req.data);

	if (copy_to_user(argp, &req, sizeof(req))) {
		IOACCESS_ERR("Unable to copy output to user");
		res = -EFAULT;
		goto ioaccess_read_pci_config_space_exit;
	}

	res = 0;

ioaccess_read_pci_config_space_exit:

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	return res;
}
static int ioaccess_write_pci_config_space(void __user *argp)
{
	int res = 0;
	struct PciData req;
	struct pci_dev *pdev = NULL;
	uint8_t bus, device, function;
	uint16_t offset;

	if (copy_from_user(&req, argp, sizeof(req))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	bus = req.bus;
	device = req.device;
	function = req.function;
	offset = req.offset;

	pdev = pci_get_domain_bus_and_slot(0, bus, PCI_DEVFN(device, function));
	if (pdev == NULL) {
		IOACCESS_ERR(
			"PCI device at bus %x - dev %x - function %x not found.\n",
			bus, device, function);
		res = -ENODEV;
		goto ioaccess_write_pci_config_space_exit;
	}


	IOACCESS_INFO(
		"PCI device at bus %x - dev %x - function %x. "
		"Config space offset %x - write data %x\n",
		bus, device, function, offset, req.data);

	res = pci_write_config_dword(pdev, offset, req.data);
	if (res < 0) {
		IOACCESS_ERR(
			"Error in writing PCI device at bus %x - dev %x - function %x. "
			"Config space offset %x - data %x\n",
			bus, device, function, offset, req.data);
		res = -EFAULT;
		goto ioaccess_write_pci_config_space_exit;
	}

	res = 0;

ioaccess_write_pci_config_space_exit:

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	return res;
}
static int ioaccess_alloc_non_cached(void __user *argp)
{
	size_t size = 0;
	void *virtual_kernel_address = NULL;
	struct MemoryAllocation req;

	if (copy_from_user(&req, argp, sizeof(req))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	size = req.size;

	virtual_kernel_address = kmalloc(size, GFP_KERNEL | GFP_DMA);

	if (virtual_kernel_address == NULL) {
		IOACCESS_ERR(
			"Unable to allocate DMA descriptor array with size %u\n",
			size);
		return -ENOMEM;
	}

	req.physicalAddress = virt_to_bus(virtual_kernel_address);

	IOACCESS_INFO(
		"Virtual Kernel Address: %p; Physical Address: %llx, size = %u\n",
		virtual_kernel_address,
		req.physicalAddress,
		size);

	if (copy_to_user(argp, &req, sizeof(req))) {
		IOACCESS_ERR("Unable to copy output to user");
		kfree(virtual_kernel_address);
	    return -EFAULT;
	}

	return 0;
}
static int ioaccess_free_non_cached(void __user *argp)
{
	void *virtual_kernel_address = NULL;
	uint64_t physicalAddress;

	if (copy_from_user(&physicalAddress, argp, sizeof(physicalAddress))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	virtual_kernel_address = bus_to_virt(physicalAddress);

	kfree(virtual_kernel_address);

	IOACCESS_INFO(
		"Freed Virtual Kernel Address: %p; Physical Address: %llx\n",
		virtual_kernel_address,
		physicalAddress);

	return 0;
}

struct pci_memory_allocation_node {
	struct PciMemoryAllocation pci_memory_allocation;
	struct list_head list;
};
LIST_HEAD(pci_memory_allocation_list);

static int ioaccess_alloc_pci_dma(void __user *argp)
{
	dma_addr_t dma_handle = 0;
	struct pci_dev *pdev = NULL;
	size_t size = 0;
	void *virtual_kernel_address = NULL;
	struct PciMemoryAllocation req;
	struct pci_memory_allocation_node *node = NULL;
	int res = 0;

	if (copy_from_user(&req, argp, sizeof(req))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	size = req.size;

	pdev = pci_get_device(req.vendorId, req.deviceId, pdev);
	if (pdev == NULL) {
		IOACCESS_ERR("PCI device %x:%x not found.\n", req.vendorId,
			req.deviceId);
		res = -ENODEV;
		goto ioaccess_alloc_pci_dma_exit;
	}

	virtual_kernel_address = dma_alloc_coherent(
		&pdev->dev,
		size,
	    &dma_handle,
	    GFP_KERNEL);

	if (virtual_kernel_address == NULL) {
		IOACCESS_ERR(
			"Unable to allocate DMA descriptor array with size %u\n",
			size);
		res = -ENOMEM;
		goto ioaccess_alloc_pci_dma_exit;
	}

	req.physicalAddress = dma_handle;

	IOACCESS_INFO(
		"DMA Virtual Kernel Address: %p; Physical Address: %llx, size = %u\n",
		virtual_kernel_address,
		req.physicalAddress,
		size);

	if (copy_to_user(argp, &req, sizeof(req))) {
		IOACCESS_ERR("Unable to copy output to user");
		res = -EFAULT;
		goto ioaccess_alloc_pci_dma_exit;
	}

	/* Keep an internal list of allocated memories */
	node = (struct pci_memory_allocation_node *)
		kmalloc(sizeof(struct pci_memory_allocation_node), GFP_KERNEL);

	if (node == NULL) {
		IOACCESS_ERR("Unable to allocate node entry\n");
		res = -ENOMEM;
		goto ioaccess_alloc_pci_dma_exit;
	}

	node->pci_memory_allocation = req;

	list_add(&node->list, &pci_memory_allocation_list);

	res = 0;

ioaccess_alloc_pci_dma_exit:

	if ((res != 0) && (virtual_kernel_address != NULL)) {
		dma_free_coherent(
			&pdev->dev,
			size,
			virtual_kernel_address,
			dma_handle);
	}

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	return res;
}
static int free_pci_memory_allocation(struct pci_memory_allocation_node *node)
{
	uint32_t size = 0;
	void *virtual_kernel_address = NULL;
	struct pci_dev *pdev = NULL;
	struct PciMemoryAllocation pma;
	int res = 0;

	IOACCESS_INFO("Freeing pci memory allocation\n");
	if (node == NULL) {
		IOACCESS_ERR("node is null\n");
		res = -EINVAL;
		goto free_pci_memory_allocation_node_exit;
	}

	pma = node->pci_memory_allocation;

	pdev = pci_get_device(pma.vendorId, pma.deviceId, pdev);
	if (pdev == NULL) {
		IOACCESS_ERR("PCI device %x:%x not found.\n",
			pma.vendorId, pma.deviceId);
		res = -ENODEV;
		goto free_pci_memory_allocation_node_exit;
	}

	virtual_kernel_address = bus_to_virt(pma.physicalAddress);
	size = pma.size;

	dma_free_coherent(&pdev->dev, size, virtual_kernel_address,
		pma.physicalAddress);

	IOACCESS_INFO(
		"Freed DMA Virtual Kernel Address: %p; Physical Address: %llx\n",
		virtual_kernel_address,
		pma.physicalAddress);

	res = 0;

free_pci_memory_allocation_node_exit:

	if (pdev != NULL) {
		pci_dev_put(pdev);
	}

	return res;
}

static int ioaccess_free_pci_dma(void __user *argp)
{
	uint64_t physicalAddress;
	struct list_head *p, *n;
	struct pci_memory_allocation_node *node = NULL;
	int res = 0;

	if (copy_from_user(&physicalAddress, argp, sizeof(physicalAddress))) {
		IOACCESS_ERR("Unable to copy input from user\n");
		return -EFAULT;
	}

	/* Find the allocate memory from the list */
	list_for_each_safe(p, n, &pci_memory_allocation_list)
	{
		node = list_entry(p, struct pci_memory_allocation_node, list);

		if (
			node->pci_memory_allocation.physicalAddress ==
			physicalAddress) {

			IOACCESS_INFO("Found node for %llx address.\n",
				physicalAddress);
			res = free_pci_memory_allocation(node);
			list_del(p);
			kfree(node);
			return res;
		}
	}

	IOACCESS_ERR("Unable to find a node entry with %llx address\n",
		physicalAddress);
	return -EFAULT;
}
static void free_pci_memory_allocation_list(void)
{
	/* Free the entire list */
	struct list_head *p, *n;
	struct pci_memory_allocation_node *node = NULL;
	list_for_each_safe(p, n, &pci_memory_allocation_list)
	{
		node = list_entry(p, struct pci_memory_allocation_node, list);

		IOACCESS_INFO("Free node associated to %llx address\n",
			node->pci_memory_allocation.physicalAddress);

		free_pci_memory_allocation(node);
		list_del(p);
		kfree(node);
	}
}
/*===========================================================================*/
static long ioaccess_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user *argp = (void __user *) arg;
	int err;

	IOACCESS_INFO("ioctl command %d arg %lu\n", cmd, arg);

	switch (cmd) {

	case IOCTL_PciDeviceCount:
		err = ioaccess_device_count(argp);
		break;
	case IOCTL_PciDeviceList:
		err = ioaccess_device_list(argp);
		break;
	case IOCTL_PciDeviceDiscover:
		err = ioaccess_discover_device(argp);
		break;

	case IOCTL_PciReadCfg:
		err = ioaccess_read_pci_config_space(argp);
		break;

	case IOCTL_PciWriteCfg:
		err = ioaccess_write_pci_config_space(argp);
		break;

	case IOCTL_AllocNonCachedMemory:
		err = ioaccess_alloc_non_cached(argp);
		break;

	case IOCTL_FreeNonCachedMemory:
		err = ioaccess_free_non_cached(argp);
		break;

	case IOCTL_AllocPciDmaMemory:
		err = ioaccess_alloc_pci_dma(argp);
		break;

	case IOCTL_FreePciDmaMemory:
		err = ioaccess_free_pci_dma(argp);
		break;

	default:
		err = -EINVAL;
		break;
	};

	return err;
}
/*===========================================================================*/

static const struct file_operations ioaccess_fops = {
	.owner          = THIS_MODULE,
	.read           = ioaccess_read,
	.write          = ioaccess_write,
	.llseek         = ioaccess_llseek,
	.unlocked_ioctl = ioaccess_ioctl,
	.compat_ioctl	= ioaccess_ioctl,
};

static struct miscdevice ioaccess_misc = {
	.fops           = &ioaccess_fops,
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = "ioaccess",
};

static int __init ioaccess_init(void)
{
	int rc;

	IOACCESS_INFO("Loading");

	rc = misc_register(&ioaccess_misc);
	if (rc < 0) {
		IOACCESS_INFO("Error %d registering device\n", rc);
	}

	IOACCESS_INFO("registered misc device\n");

	return rc;
}

static void __exit ioaccess_exit(void)
{
	IOACCESS_INFO("Unloading");

	free_pci_memory_allocation_list();

	misc_deregister(&ioaccess_misc);
}

module_init(ioaccess_init);
module_exit(ioaccess_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("ioaccess module");
MODULE_VERSION("1.3");
