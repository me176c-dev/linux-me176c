// SPDX-License-Identifier: MIT
// Copyright (C) 2019 lambdadroid

#ifndef ME176C_FACTORY_H
#define ME176C_FACTORY_H
#ifdef __cplusplus
extern "C" {
#endif

struct me176c_factory_info {
    /** The serial number of the device. */
    char serialno[32];
    /** The Bluetooth device address of the device (format XX:XX:XX:XX:XX:XX) */
    char bdaddr[18];
    /** The WiFi hardware address of the device (format XX:XX:XX:XX:XX:XX) */
    char wifiaddr[18];
};

/**
 * Read the factory info from the mounted factory partition.
 *
 * @param mount The mount directory of the factory partition
 * @param info Pointer to the struct where the result is stored
 */
int me176c_factory_read(const char *mount, struct me176c_factory_info *info);

/**
 * Load the factory info from the block device pointed by "source".
 * Mounts the block device at "target", reads the factory info and unmounts
 * the block device again.
 *
 * @param mount The path to the block device (of the factory partition)
 * @param target The path to mount to (temporarily)
 * @param info Pointer to the struct where the result is stored
 */
int me176c_factory_load(const char *source, const char *target, struct me176c_factory_info *info);

#ifdef __cplusplus
}
#endif
#endif
