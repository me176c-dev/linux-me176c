# SPDX-License-Identifier: MIT
# Copyright (C) 2019 lambdadroid

project('me176c-factory', 'c',
    version: '0.1.0',
    license: 'MIT',
    default_options: [
        'c_std=c11',
        'warning_level=3',
        'werror=true'
    ]
)

me176c_libdir = join_paths(get_option('libdir'), 'me176c')
me176c_rundir = join_paths(get_option('rundir'), 'me176c')

executable('me176c-factory',
    'me176c-factory.c',
    'service.c',
    c_args: ['-DRUNTIME_DIRECTORY="' + me176c_rundir + '"'],
    install: true,
    install_dir: me176c_libdir,
)

conf = configuration_data()
conf.set('BLOCK_DEVICE', get_option('blockdevice'))
conf.set('BINARY_DIRECTORY', join_paths(get_option('prefix'), get_option('bindir')))
conf.set('LIBRARY_DIRECTORY', join_paths(get_option('prefix'), me176c_libdir))
conf.set('RUNTIME_DIRECTORY', me176c_rundir)

if get_option('systemd')
    subdir('systemd')
endif
if get_option('openrc')
    subdir('openrc')
endif
