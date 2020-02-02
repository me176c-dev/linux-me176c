<!-- SPDX-License-Identifier: CC-BY-SA-4.0 OR GFDL-1.3-or-later -->

# me176c-factory
A simple library, and optionally service to load the factory information
(serial number, BT/WiFi mac address) from the `factory` partition on the
ASUS MeMO Pad 7 (ME176C(X)).

## Library
`me176c-factory.c/h` provides methods to load the factory information.
See [`me176c-factory.h`](me176c-factory.h) for details.

## Service
There is a simple service implementation suitable for most Linux distributions.
It mounts the `factory` partition temporarily on boot, reads the data and stores
it in the runtime directory `/run/me176c` (compile-time configurable).

### Building
Using [Meson]:

```
meson . build
ninja -C build
```

### Usage
Run `/usr/lib/me176c/me176c-factory <block-device>`. In most cases,
`<block-device>` should be `/dev/disk/by-partlabel/factory`.

The service then writes the information to:
  - `/run/me176c/serialno`: Serial number
  - `/run/me176c/wifiaddr`: WiFi hardware (MAC) address
  - `/run/me176c/bdaddr`: Bluetooth device (MAC) address
  - `/run/me176c/environment`: Above information as `KEY=VALUE` pairs

### systemd
systemd and udev integration is enabled by default. It provides `me176c-factory.service`
that starts the service on boot. Optionally, udev rules together with
`me176c-factory-bdaddr@<dev>` and `me176c-factory-wifiaddr@<dev>` are provided
to automatically set the WiFi/Bluetooth MAC address on boot.

### OpenRC
OpenRC + udev integration can be enabled with `-Dsystemd=false -Dopenrc=true`.
It provides the `me176c-factory` service. Optionally, udev rules are provided
to automatically set the WiFi/Bluetooth MAC address on boot.

## Packaging
- **Arch Linux:** `me176c-factory` AUR package

[Meson]: http://mesonbuild.com
