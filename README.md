<!-- SPDX-License-Identifier: CC-BY-SA-4.0 OR GFDL-1.3-or-later -->

# Linux on ASUS MeMO Pad 7 (ME176C(X))
This repository contains source code for using Linux on the ASUS MeMO Pad 7 (ME176C(X))),
including a Linux kernel fork and userspace modules shared between different Linux distributions.

**IMPORTANT NOTE:** Recent Linux versions (5.19+) work without the need for any device specific
patches thanks to the new `x86-android-tablets` module with support for the ME176C. The Linux
kernel fork is no longer maintained. Just use the standard Linux kernel as provided by your
Linux distribution!

The [me176c-factory](/factory) module is still maintained to allow setting the
correct MAC addresses for WiFi and Bluetooth.

## Kernel branches
- Archived:
  - [Linux 5.4](https://github.com/me176c-dev/linux-me176c/tree/5.4): `5.4`
  - [Android (Linux 4.19)](https://github.com/me176c-dev/linux-me176c/tree/android-4.19): `android-4.19`
  - [Linux 4.19](https://github.com/me176c-dev/linux-me176c/tree/4.19): `4.19`
  - [Linux 4.14](https://github.com/me176c-dev/linux-me176c/tree/4.14): `4.14`
  - [LineageOS 14.1 (Linux 4.14)](https://github.com/me176c-dev/linux-me176c/tree/cm-14.1): `cm-14.1`

## Userspace modules
- [me176c-factory](/factory)
- [thermald-me176c](/thermal)

## Documentation
- [Porting - Using linux-me176c in other distributions](/porting.md)
- [me176c-boot](https://github.com/me176c-dev/me176c-boot#readme)
- [Dual/multi boot example (me176c-boot)](https://github.com/me176c-dev/me176c-boot/tree/master/examples/multi-boot)
- [Tips and tricks (ArchWiki)](https://wiki.archlinux.org/index.php/ASUS_MeMO_Pad_7_(ME176C(X))#Tips_and_tricks)
