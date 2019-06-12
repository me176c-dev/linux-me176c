<!-- SPDX-License-Identifier: CC-BY-SA-4.0 OR GFDL-1.3-or-later -->

# thermald-me176c
This directory contains a custom configuration for [thermald]
for the ASUS MeMO Pad 7 (ME176C(X)). It is based on similar trip points that
were used on the stock (Android) system of the tablet. They begin throttling
quite early based on the device (not CPU) temperature.

Feel free to contribute if you think these values can be improved.

## Usage
Install [thermald]. Copy `thermal-conf-me176c.xml` to `/etc/thermald/thermal-conf.xml`.
With the `--config-file=` parameter, it can also be copied to a non-generic location
like `/etc/thermald/thermal-conf-me176c.xml`. An updated systemd service for this
is provided in `thermald-me176c.service`.

[thermald]: https://github.com/intel/thermal_daemon

## Packaging
- **Arch Linux:** `thermald-me176c` AUR package
