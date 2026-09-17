#!/usr/bin/env bash
# Flash an ELF to the sensor board over USB DFU via the STM32WB55 ROM bootloader.
# Used as the cargo runner (`cargo run --release`), or call it with an ELF path.
#
# Works from either state:
#  - app running: dfu-util sends DFU_DETACH to the app's DFU runtime interface,
#    the app reboots into the ROM bootloader, dfu-util continues with it;
#  - already in the ROM bootloader (blank chip, JP1 shorted at plug-in,
#    button held 2 s, or after a panic): dfu-util flashes directly.
set -euo pipefail

elf=${1:?usage: flash.sh path/to/elf}
bin="${elf}.bin"

host=$(rustc -vV | sed -n 's/^host: //p')
objcopy="$(rustc --print sysroot)/lib/rustlib/${host}/bin/llvm-objcopy"
"$objcopy" -O binary "$elf" "$bin"
echo "image: $bin ($(stat -c %s "$bin") bytes)"

DFU_UTIL=${DFU_UTIL:-dfu-util}
# With several boards attached, pick one by USB port path (lsusb -t / dmesg),
# e.g. DFU_PATH=1-4.1. The path survives the detach/re-enumerate cycle; the
# USB serial does not (the app reports the chip UID, the ROM bootloader its own).
path_arg=()
[ -n "${DFU_PATH:-}" ] && path_arg=(-p "$DFU_PATH")
# -a 0 is "@Internal Flash". Never use alt 1 (option bytes) or alt 2 (OTP).
exec "$DFU_UTIL" "${path_arg[@]}" -d 1209:0001,0483:df11 -a 0 -s 0x08000000:leave -D "$bin"
