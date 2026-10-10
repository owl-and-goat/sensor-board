# ST's firmware for the radio coprocessor, from the STM32CubeWB release that
# our firmware is written against.
coprocessor_release := "v1.24.0"
coprocessor_url := "https://raw.githubusercontent.com/STMicroelectronics/STM32CubeWB/" + coprocessor_release + "/Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x"
coprocessor_images := ".cache/coprocessor-" + coprocessor_release
fus_image := coprocessor_images / "stm32wb5x_FUS_fw.bin"
stack_image := coprocessor_images / "stm32wb5x_Thread_FTD_fw.bin"

@cli *args:
  ( cd cli && cargo build --release )
  cli/target/release/sensor-board-cli {{args}}

# Fetch ST's coprocessor images unless they are already cached, and verify their checksums
coprocessor-images: (_image fus_image "bbab9b42d92c31f8a04e2f04ef9361c3f6f01714c14408cd34fb1583a6edf30d") (_image stack_image "53babb15ecb9da0961aacba612be0b1273e79271232348c545f1bb87c73b9e84")

_image file sha256:
  @mkdir -p {{parent_directory(file)}}
  @[ -f {{file}} ] || { echo "Fetching {{file_name(file)}}"; curl -fL --remove-on-error -o {{file}} {{coprocessor_url}}/{{file_name(file)}}; }
  @echo "{{sha256}}  {{file}}" | sha256sum --check --quiet

# Two images: the firmware alone, which is what an update sends to a board,
# and the full flash image, with the bootloader, its blank state page, and
# the firmware.
firmware_image := "firmware/target/sensor-board-firmware.bin"
flash_image := "firmware/target/sensor-board-flash.bin"
objcopy := '"$(rustc --print sysroot)"/lib/rustlib/*/bin/llvm-objcopy'

# Set up a new board over USB: flash our bootloader and firmware to CPU1, then install FUS and the Thread stack on CPU2. The board has to be in its ROM bootloader, as a blank chip is
new-board: coprocessor-images _flash-image _dfu-download
  {{just_executable()}} cli coprocessor install {{fus_image}}
  {{just_executable()}} cli coprocessor install {{stack_image}}

# Flash the bootloader and the firmware through the debug probe
flash: _flash-image
    probe-rs download --chip STM32WB55CG --binary-format bin --base-address 0x08000000 --verify {{flash_image}}
    probe-rs reset --chip STM32WB55CG

# Update a board's firmware over USB, through its running firmware. With several boards attached, select one: just update --board <serial>
update *board: firmware-image
    {{just_executable()}} cli firmware update {{firmware_image}} {{board}}

# Update every attached board over USB, one after the other
update-all: firmware-image
    #!/usr/bin/env bash
    set -euo pipefail
    for serial in $({{just_executable()}} cli list | cut -d' ' -f1); do
        {{just_executable()}} cli firmware update {{firmware_image}} --board "$serial"
    done

# Update every board on the network. The attached board receives the image over USB and offers it to the others, which fetch it over the network. With several boards attached, select one: just push --board <serial>
push *board: firmware-image
    {{just_executable()}} cli firmware push {{firmware_image}} {{board}}

# Flash the bootloader and the firmware over USB through the ROM bootloader. The board has to be running our firmware. With several boards attached, select one: just dfu-flash --board <serial>
dfu-flash *board: _flash-image
    {{just_executable()}} cli bootloader {{board}}
    {{just_executable()}} _dfu-download -w

# Flash every attached board through the ROM bootloader, one after the other
dfu-flash-all: _flash-image
    #!/usr/bin/env bash
    set -euo pipefail
    for serial in $({{just_executable()}} cli list | cut -d' ' -f1); do
        echo "Board $serial"
        {{just_executable()}} cli bootloader --board "$serial"
        {{just_executable()}} _dfu-download -w
        # Give the board time to leave the ROM bootloader before the next one enters it.
        sleep 3
    done

# Build the firmware as a raw image. Every recipe that touches a board builds it first
firmware-image:
    ( cd firmware && cargo build --release && {{objcopy}} -O binary target/thumbv7em-none-eabihf/release/sensor-board-firmware ../{{firmware_image}} )

# Build the full flash image: the bootloader, padded with 0xff up to where the firmware starts, then the firmware
_flash-image: firmware-image
    ( cd bootloader && cargo build --release && {{objcopy}} -O binary --gap-fill 0xff --pad-to 0x08007000 target/thumbv7em-none-eabihf/release/sensor-board-bootloader target/sensor-board-bootloader.bin )
    cat bootloader/target/sensor-board-bootloader.bin {{firmware_image}} > {{flash_image}}

# Write the flash image to a board in its ROM bootloader, and start it. With -w, wait for a board to enter the ROM bootloader (dfu-util 0.11 lists the long form, --wait, but does not accept it)
_dfu-download *flags:
    # dfu-util exits with an error even when the download works: at ":leave" the chip starts the firmware, so it is gone when dfu-util asks for the final status. Check dfu-util's own download message instead.
    dfu-util {{flags}} -d 0483:df11 -a 0 -s 0x08000000:leave -D {{flash_image}} | tee /dev/stderr | grep "File downloaded successfully" > /dev/null

attach:
    probe-rs attach --chip STM32WB55CG --no-catch-reset firmware/target/thumbv7em-none-eabihf/release/sensor-board-firmware

build *args:
    ( cd protocol && cargo build {{args}} )
    ( cd firmware && cargo build {{args}} )
    ( cd bootloader && cargo build {{args}} )
    ( cd cli && cargo build {{args}} )

fmt *args:
    ( cd protocol && cargo fmt {{args}} )
    ( cd firmware && cargo fmt {{args}} )
    ( cd bootloader && cargo fmt {{args}} )
    ( cd cli && cargo fmt {{args}} )
