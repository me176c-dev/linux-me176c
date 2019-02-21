// SPDX-License-Identifier: MIT
// Copyright (C) 2019 lambdadroid

#define _POSIX_C_SOURCE  200809L

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include "me176c-factory.h"

#define MOUNT_PATH        RUNTIME_DIRECTORY "/factory"
#define SERIALNO_FILE     RUNTIME_DIRECTORY "/serialno"
#define BDADDR_FILE       RUNTIME_DIRECTORY "/bdaddr"
#define WIFIADDR_FILE     RUNTIME_DIRECTORY "/wifiaddr"
#define ENVIRONMENT_FILE  RUNTIME_DIRECTORY "/environment"

static int write_file(char *p, char *s) {
    int fd = creat(p, 0660);
    if (fd < 0)
        return 1;

    int len = strlen(s);
    s[len] = '\n';
    len = write(fd, s, len + 1);
    close(fd);

    if (len < 0) {
        fprintf(stderr, "Failed to write to %s: %s\n", p, strerror(errno));
        return 1;
    }

    return 0;
}

static int write_environment(const struct me176c_factory_info *info) {
    int fd = creat(ENVIRONMENT_FILE, 0660);
    if (fd < 0)
        return 1;

    int written = dprintf(fd, "SERIALNO=%s\nBDADDR=%s\nWIFIADDR=%s\n",
        info->serialno, info->bdaddr, info->wifiaddr);
    close(fd);
    return written < 0;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: me176c-factory <block-device>\n");
        return 1;
    }

    int ret = mkdir(RUNTIME_DIRECTORY, 0777);
    if (ret && errno != EEXIST) {
        perror("Failed to create runtime directory '" RUNTIME_DIRECTORY "'");
        return 1;
    }

    const char *source = argv[1];
    struct me176c_factory_info info;
    ret = me176c_factory_load(source, MOUNT_PATH, &info);
    if (ret) {
        fprintf(stderr, "Failed to load factory info from '%s': %s\n",
            source, strerror(errno));
        return 1;
    }

    printf("Loaded factory info from '%s': serialno: %s, bdaddr: %s, wifiaddr: %s\n",
        source, info.serialno, info.bdaddr, info.wifiaddr);

    return write_environment(&info)
        | write_file(SERIALNO_FILE, info.serialno)
        | write_file(BDADDR_FILE, info.bdaddr)
        | write_file(WIFIADDR_FILE, info.wifiaddr);
}
