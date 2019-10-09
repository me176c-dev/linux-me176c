# SPDX-License-Identifier: MIT
# Copyright (C) 2019 lambdadroid

configure_file(configuration: conf,
    input: 'me176c-factory.initd.in',
    output: 'me176c-factory',
    install_dir: join_paths(get_option('prefix'), get_option('sysconfdir'), 'init.d'),
)

if get_option('udev')
    subdir('udev')
endif
