# SPDX-License-Identifier: MIT
# Copyright (C) 2019 lambdadroid

udev = dependency('udev')

configure_file(configuration: conf,
    input: '99-me176c-factory.rules.in',
    output: '99-me176c-factory.rules',
    install_dir: join_paths(udev.get_pkgconfig_variable('udevdir'), 'rules.d'),
)

configure_file(configuration: conf,
    input: 'setaddr.sh.in',
    output: 'setaddr.sh',
    install_dir: me176c_libdir,
)
