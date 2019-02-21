// SPDX-License-Identifier: MIT
// Copyright (C) 2019 lambdadroid

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include "me176c-factory.h"

#define PHONE_INFO_FILE  "PhoneInfodisk/PhoneInfo_inf"

#define BDADDR_OFFSET     0x99
#define WIFI_ADDR_OFFSET  0xcc

static void format_addr(const char *buf, char *addr) {
    // Copy MAC address and add separators
    for (int i = 0; i < 12; i += 2) {
        if (i)
            *addr++ = ':';
        *addr++ = buf[i];
        *addr++ = buf[i+1];
    }
    *addr = 0;
}

int me176c_factory_read(const char *mount, struct me176c_factory_info *info) {
    char buf[256];
    int i = snprintf(buf, sizeof(buf), "%s/%s", mount, PHONE_INFO_FILE);
    if (i < 0 || i >= (int) sizeof(buf)) {
        errno = ENAMETOOLONG;
        return -1;
    }

    int fd = open(buf, O_RDONLY);
    if (fd < 0)
        return fd;

    i = read(fd, buf, sizeof(buf));
    close(fd);
    if (i != sizeof(buf))
        return i;

    memcpy(info->serialno, buf, sizeof(info->serialno));
    info->serialno[sizeof(info->serialno)-1] = 0;

    format_addr(&buf[BDADDR_OFFSET], info->bdaddr);
    format_addr(&buf[WIFI_ADDR_OFFSET], info->wifiaddr);
    return 0;
}

int me176c_factory_load(const char *source, const char *target, struct me176c_factory_info *info) {
    int ret = mkdir(target, 0770);
    if (ret && errno != EEXIST)
        return ret;

    ret = mount(source, target, "ext4", MS_RDONLY | MS_NOSUID | MS_NODEV | MS_NOEXEC, "");
    if (ret)
        return ret;

    ret = me176c_factory_read(target, info);

    umount(target);
    rmdir(target);
    return ret;
}
