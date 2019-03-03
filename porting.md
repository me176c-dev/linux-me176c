<!-- SPDX-License-Identifier: CC-BY-SA-4.0 OR GFDL-1.3-or-later -->

## Porting - Using linux-me176c in other distributions
Most of the code in linux-me176c is not specific to one Linux distribution.
In fact it is already being used for Android, Arch Linux and postmarketOS.
This page explains how to apply it to an other Linux distribution.

### Linux kernel fork (linux-me176c)
Most of the necessary patches for full functionality have been submitted upstream.
However, the battery/charging drivers have been ported from the downstream stock
(3.10) kernel. Even though they went through major cleanup already, they are not
even closely in a state that is suitable for upstreaming.

This is why it is necessary to run a Linux kernel fork ("linux-me176c") for
full functionality. Compared to other downstream kernel it only carries a couple
of patches that can be easily rebased on top of newer kernel versions.

Usually, linux-me176c tracks the latest LTS kernel and is available from one
of the branches in this repository.

### Dependencies
Unfortunately, there are a few closed-source components required for full
functionality. Some of them (e.g. me176c-acpi and CPU Microcode) will be running
anyway through the BIOS. Therefore, replacing them with updated/fixed versions
will not add new unfree code.

- **[me176c-acpi]:** For full functionality (e.g. touchscreen, Bluetooth, ...)
- **Firmware:**
  - [linux-firmware](https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/)
      (available as package for almost all distributions):
    - **WiFI:** `brcm/brcmfmac43362-sdio.bin`
    - **Sound:** `intel/fw_sst_0f28.bin`
  - Additional firmware from the ASUS stock ROM ([`UL-K013-WW-12.10.1.36-user.zip`](https://dlcdnets.asus.com/pub/ASUS/EeePAD/ME176C/UL-K013-WW-12.10.1.36-user.zip)):
    - **WiFI:** `/system/etc/nvram.txt` -> `brcm/brcmfmac43362-sdio.txt`
    - **Bluetooth:** `/system/etc/firmware/BCM2076B1_002.002.004.0132.0141_reduced_2dB.hcd` -> `brcm/BCM2076B1.hcd`
- **Userspace:**
  - Battery Daemon (`/sbin/upi_ug31xx` in the `boot.img` ramdisk of the ASUS Stock ROM):  
    May be needed for battery driver to work correctly. Appears to be used for calibration/backup purposes.
    - Running it avoids `User space daemon no response` spam in the kernel log.
    - Requires mounting the `config` partition at `/config`, although the mount point can be changed by using `sed` on the binary.
    - Example extraction: [Arch Linux (AUR) `me176c-firmware` package](https://github.com/me176c-dev/archlinux-me176c/tree/master/me176c-firmware)
    - Example systemd service: [Arch Linux (AUR) `me176c-battery` package](https://github.com/me176c-dev/archlinux-me176c/tree/master/me176c-battery)
- **CPU Microcode Update:** Not included in Intel's Linux microcode update package.
  - **Download:** From [platomav/CPUMicrocodes](https://github.com/platomav/CPUMicrocodes/tree/master/Intel/cpu30678_plat02_ver00000837_2018-01-25_PRD_F0D56486.bin)
  - Pack it into an initrd for use with the Linux CPU microcode updater.
  - Example package: [Arch Linux (AUR) `intel-ucode-byt-t-c0` package](https://aur.archlinux.org/packages/intel-ucode-byt-t-c0/)

### Notes
See also: [Tips and tricks (ArchWiki)](https://wiki.archlinux.org/index.php/ASUS_MeMO_Pad_7_(ME176C(X))#Tips_and_tricks)

#### WiFi / Bluetooth
- WiFi / Bluetooth do not use the same MAC address as the stock (ASUS) system by default
- Bluetooth needs to be configured with an unique MAC address from userspace to make it work
- [me176c-factory](/factory) can be used to apply the ASUS WiFi/BT MAC address from the factory partition

#### Thermal
Consider setting up a Thermal Daemon (e.g. [thermald](https://github.com/intel/thermal_daemon)) to monitor device
temperature and to throttle the CPU performance early when the device gets too hot.
  - [Custom configuration for thermald](/thermal) is provided in this repository.

[me176c-acpi]: https://github.com/me176c-dev/me176c-acpi
