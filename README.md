<!-- SPDX-License-Identifier: CC-BY-SA-4.0 OR GFDL-1.3-or-later -->

# Linux on ASUS MeMO Pad 7 (ME176C/X)
This repository contains the source code for the Linux kernel fork for the ASUS MeMO Pad 7 (ME176C/X).  
This README contains instructions and (random) notes for using Linux on this device.

## Usage
### Branches
- **[Linux 4.19](https://github.com/me176c-dev/linux-me176c/tree/4.19):** `4.19`
- **[Android (Linux 4.19)](https://github.com/me176c-dev/linux-me176c/tree/android-4.19):** `android-4.19`
- Archived:
  - [Linux 4.14](https://github.com/me176c-dev/linux-me176c/tree/4.14): `4.14`
  - [LineageOS 14.1 (Linux 4.14)](https://github.com/me176c-dev/linux-me176c/tree/cm-14.1): `cm-14.1`

### (Optional) Dependencies (Proprietary)
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
    - Example extraction: [Arch Linux `me176c-firmware` package](https://github.com/me176c-dev/archlinux-me176c/tree/master/me176c-firmware)
    - Example systemd service: [Arch Linux `me176c-battery` package](https://github.com/me176c-dev/archlinux-me176c/tree/master/me176c-battery)
- **CPU Microcode Update:** Not included in Intel's Linux microcode update package.
  - **Download:** From [platomav/CPUMicrocodes](https://github.com/platomav/CPUMicrocodes/tree/master/Intel/cpu30678_plat02_ver00000837_2018-01-25_PRD_F0D56486.bin)
  - Pack it into an initrd for use with the Linux CPU microcode updater.
  - Example package: [Arch Linux `intel-ucode-byt-t-c0` package](https://github.com/me176c-dev/archlinux-me176c/blob/master/intel-ucode-byt-t-c0/PKGBUILD)

## (Random) Notes
### Partitioning
- Avoid repartitioning the internal storage, or only shrink the `data` partition. Some partitions on the internal storage
  contain important data (e.g. MAC addresses) and may be needed for the device to boot.
- Multi-boot (or dual-boot) setup: See me176c-boot guide (TODO)
- Encryption: Optional, but possible options to enter a boot password to unlock the partition:
  - USB-OTG keyboard or a hardware token like Yubikey (if you don't want to type the password)
  - [osk-sdl](https://wiki.postmarketos.org/wiki/Osk-sdl) to enter it via touchscreen

### Swap
- This tablet has only very little RAM, so you almost certainly want to set up some kind of memory swap space.
- Flash storage has only a limited amount of write cycles, so it is not appropriate as swap in the long term.
- ZRAM can be used to compress parts of the RAM as swap space, gaining a little bit more RAM at the cost of CPU time.

### Audio
- ALSA mixers need to be configured using ALSA UCM. The configurations (`bytcr-rt5640`) are available upstream,
  but may need to be applied manually using `alsaucm`.
  - PulseAudio applies them automatically
- The speaker volume can be set higher than supported by the hardware. Setting it to very high values may damage the speakers
  or cause distortions.
- Using ALSA, some audio files (especially MP3 and AAC) are not played correctly when converted to `float` instead of `s16`
  format. This does not happen when using PulseAudio.

### Hardware-accelerated Video Codecs
- Software codecs require higher CPU usage
- Hardware-accelerated (video) codecs are provided by [VA-API/libva](https://github.com/intel/libva) and [intel-vaapi-driver](https://github.com/intel/intel-vaapi-driver).

### Sensors
- Desktop environment specific, most support them through [iio-sensor-proxy](https://github.com/hadess/iio-sensor-proxy).
- Hall Sensor / Smart Cover / Lid: If you have a case that closes magnetically and automatically wakes/puts the device to sleep,
  it may trigger too early/often and suspend your device unexpectedly.
  - For systemd, this can be disabled in `/etc/systemd/logind.conf` (`HandleLidSwitch=ignore`)

### Thermal
Consider setting up a Thermal Daemon (e.g. [thermald](https://github.com/intel/thermal_daemon)) to monitor device
temperature and to throttle the CPU performance early when the device gets too hot.
  - Example setup: [Arch Linux `me176c-thermal` package](https://github.com/me176c-dev/archlinux-me176c/tree/master/me176c-thermal)

[me176c-acpi]: https://github.com/me176c-dev/me176c-acpi
