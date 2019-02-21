# SPDX-License-Identifier: MIT
# Copyright (C) 2019 lambdadroid

udev = dependency('udev')

configure_file(configuration: conf,
    input: 'me176c-factory-wifiaddr@.service.in',
    output: 'me176c-factory-wifiaddr@.service',
    install_dir: systemdsystemunitdir,
)

configure_file(configuration: conf,
    input: 'me176c-factory-bdaddr@.service.in',
    output: 'me176c-factory-bdaddr@.service',
    install_dir: systemdsystemunitdir,
)

install_data('99-me176c-factory.rules',
  install_dir: join_paths(udev.get_pkgconfig_variable('udevdir'), 'rules.d'))
