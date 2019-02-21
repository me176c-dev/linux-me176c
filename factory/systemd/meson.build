# SPDX-License-Identifier: MIT
# Copyright (C) 2019 lambdadroid

systemd = dependency('systemd')
systemdsystemunitdir = systemd.get_pkgconfig_variable('systemdsystemunitdir')

configure_file(configuration: conf,
    input: 'me176c-factory.service.in',
    output: 'me176c-factory.service',
    install_dir: systemdsystemunitdir,
)

if get_option('udev')
    subdir('udev')
endif
