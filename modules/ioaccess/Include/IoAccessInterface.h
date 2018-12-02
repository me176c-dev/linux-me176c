/*--------------------------------------------------------------------------
 *
 * Copyright (c) Intel Corporation, 2009-2012  All Rights Reserved.
 *
 *  File:       IoAccessDriverInterface.h
 *
 *  Contents:   Io and Memory Api. Received by Steven J Garcia
 *
 *  Notes:
 *                Modified by Angelo Moscati 11/05/2012
 *===========================================================================*/

#ifndef _IOACCESS_DRIVER_INTERFACE_H
#define _IOACCESS_DRIVER_INTERFACE_H

#ifdef XOSA_WINDOWS

	#include <guiddef.h>

	/* TYPE DEFINITIONS */
	typedef signed char int8_t;
	typedef short int16_t;
	typedef int int32_t;

	typedef unsigned char uint8_t;
	typedef unsigned short uint16_t;
	typedef unsigned int uint32_t;

	typedef __int64 int64_t;
	typedef unsigned __int64 uint64_t;

	/*
	 * Define the IOCTL codes we will use.  The IOCTL code contains a command
	 * identifier, plus other information about the device, the type of access
	 * with which the file must have been opened, and the type of buffering.
	 */

	#define IOA_TYPE                40000
	#define FILE_FULL_ACCESS        (FILE_READ_ACCESS | FILE_WRITE_ACCESS)

#elif defined(XOSA_LINUX)
	#include <linux/types.h>
#endif

#define IOACCESS_VERSION 0x00010002
#define VENDER_ID_INTEL 0x8086;

/*===========================================================================*/

struct PciDevice_ {
	uint16_t vendorId;
	uint16_t deviceId;
	uint8_t bus;
	uint8_t device;
	uint8_t function;
};

#ifdef XOSA_WINDOWS
	#pragma warning(push)
	#pragma warning(disable : 4200)
#endif
struct PciDeviceList {
	uint32_t listSize;
	/* Contigous buffer of size listSize to keep device info */
	struct PciDevice_ list[0];
};
#ifdef XOSA_WINDOWS
	#pragma warning(pop)
#endif

struct PciData {
	uint8_t bus;
	uint8_t device;
	uint8_t function;
	uint16_t offset;
	uint32_t data;
};

#ifdef XOSA_WINDOWS
	#define IOCTL_PciDeviceCount	CTL_CODE(IOA_TYPE, 0x801, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_PciDeviceList		CTL_CODE(IOA_TYPE, 0x802, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_PciDeviceDiscover	CTL_CODE(IOA_TYPE, 0x803, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_PciReadCfg		CTL_CODE(IOA_TYPE, 0x804, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_PciWriteCfg		CTL_CODE(IOA_TYPE, 0x805, METHOD_BUFFERED, FILE_FULL_ACCESS)
#elif defined(XOSA_LINUX)
	#define IOCTL_PciDeviceCount	_IOW('M', 101, uint32_t)
	#define IOCTL_PciDeviceList		_IOW('M', 102, uint32_t)
	#define IOCTL_PciDeviceDiscover	_IOW('M', 103, uint32_t)
	#define IOCTL_PciReadCfg		_IOW('M', 104, uint32_t)
	#define IOCTL_PciWriteCfg		_IOW('M', 105, uint32_t)
#endif

/*===========================================================================*/

struct MemoryMap {
	uint64_t physicalAddress;
	uint32_t length;
};

struct MemoryAllocation {
	uint32_t size;
	uint64_t physicalAddress;
	uint64_t maxPhysicalAddress;
	uint64_t minPhysicalAddress;
};

struct PciMemoryAllocation {
	uint32_t vendorId;
	uint32_t deviceId;
	uint32_t size;
	uint64_t physicalAddress;
	uint64_t maxPhysicalAddress;
	uint64_t minPhysicalAddress;
};

#ifdef XOSA_WINDOWS
	#define IOCTL_MapPhysical		CTL_CODE(IOA_TYPE, 0x820, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_UnmapPhysical		CTL_CODE(IOA_TYPE, 0x821, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_AllocNonCachedMemory	CTL_CODE(IOA_TYPE, 0x822, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_FreeNonCachedMemory	CTL_CODE(IOA_TYPE, 0x823, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_AllocPciDmaMemory		CTL_CODE(IOA_TYPE, 0x824, METHOD_BUFFERED, FILE_FULL_ACCESS)
	#define IOCTL_FreePciDmaMemory		CTL_CODE(IOA_TYPE, 0x825, METHOD_BUFFERED, FILE_FULL_ACCESS)
#elif defined(XOSA_LINUX)
	#define IOCTL_MapPhysical		_IOW('M', 120, uint32_t)
	#define IOCTL_UnmapPhysical		_IOW('M', 121, uint32_t)
	#define IOCTL_AllocNonCachedMemory  _IOW('M', 122, uint32_t)
	#define IOCTL_FreeNonCachedMemory   _IOW('M', 123, uint32_t)
	#define IOCTL_AllocPciDmaMemory		_IOW('M', 124, uint32_t)
	#define IOCTL_FreePciDmaMemory		_IOW('M', 125, uint32_t)
#endif

/*===========================================================================*/

#endif /*_IOACCESS_DRIVER_INTERFACE_H */
