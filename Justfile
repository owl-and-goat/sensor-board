# -*- eval: (whitespace-mode -1) -*-

# ST's firmware for the radio coprocessor, from the STM32CubeWB release that
# our firmware is written against.
coprocessor_release := "v1.24.0"
coprocessor_url := "https://raw.githubusercontent.com/STMicroelectronics/STM32CubeWB/" + coprocessor_release + "/Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x"
coprocessor_images := ".cache/coprocessor-" + coprocessor_release
fus_image := coprocessor_images / "stm32wb5x_FUS_fw.bin"
stack_image := coprocessor_images / "stm32wb5x_Thread_FTD_fw.bin"
chip := "STM32WB55CG"

@cli *args:
    ( cd cli && cargo build --release )
    cli/target/release/sensor-board-cli {{ args }}

# Fetch ST's coprocessor images unless they are already cached, and verify their checksums
coprocessor-images: (_image fus_image "bbab9b42d92c31f8a04e2f04ef9361c3f6f01714c14408cd34fb1583a6edf30d") (_image stack_image "53babb15ecb9da0961aacba612be0b1273e79271232348c545f1bb87c73b9e84")

_image file sha256:
    @mkdir -p {{ parent_directory(file) }}
    @[ -f {{ file }} ] || { echo "Fetching {{ file_name(file) }}"; curl -fL --remove-on-error -o {{ file }} {{ coprocessor_url }}/{{ file_name(file) }}; }
    @echo "{{ sha256 }}  {{ file }}" | sha256sum --check --quiet

# embassy-stm32 as released, with the patch that makes Stop mode work on this
# chip. firmware/Cargo.toml takes the crate from this directory.
embassy_stm32_crate := "embassy-stm32-0.6.0"
embassy_stm32 := ".cache/embassy-stm32"
embassy_stm32_patch := "firmware/patches/embassy-stm32-stm32wb-stop.patch"

# Unpack embassy-stm32 and apply our patch to it, unless that has been done with
# the patch as it is now. The firmware does not build without this
embassy-stm32:
    #!/usr/bin/env bash
    set -euo pipefail
    stamp={{ embassy_stm32 }}/.patch.sha256
    if sha256sum --check --status "$stamp" 2>/dev/null; then exit 0; fi
    crate=.cache/{{ embassy_stm32_crate }}.crate
    mkdir -p .cache
    [ -f "$crate" ] || { echo "Fetching {{ embassy_stm32_crate }}"; curl -fL --remove-on-error -o "$crate" https://static.crates.io/crates/embassy-stm32/{{ embassy_stm32_crate }}.crate; }
    echo "486c0622deb5a519fc4d2cb8e3ef324f7568fcdfff201ff8fcab46557d663ceb  $crate" | sha256sum --check --quiet
    rm -rf {{ embassy_stm32 }}
    mkdir {{ embassy_stm32 }}
    tar -xzf "$crate" -C {{ embassy_stm32 }} --strip-components 1
    patch --quiet -d {{ embassy_stm32 }} -p1 < {{ embassy_stm32_patch }}
    sha256sum {{ embassy_stm32_patch }} > "$stamp"

# Two images: the firmware alone, which is what is sent to the board by an OTA
# update, and the full flash image, with the bootloader, its blank state page,
# and the firmware.
firmware_image := "firmware/target/sensor-board-firmware.bin"
flash_image := "firmware/target/sensor-board-flash.bin"
objcopy := '"$(rustc --print sysroot)"/lib/rustlib/*/bin/llvm-objcopy'
elf_firmware := "firmware/target/thumbv7em-none-eabihf/release/sensor-board-firmware"

# Set up a new board over USB: flash our bootloader and firmware to CPU1, then
# install FUS and the Thread stack on CPU2.
new-board: coprocessor-images _flash-image _dfu-download
    {{ just_executable() }} cli coprocessor install {{ fus_image }}
    {{ just_executable() }} cli coprocessor install {{ stack_image }}

# Flash the bootloader and the firmware through the debug probe
flash: _flash-image
    probe-rs download --chip {{ chip }} --binary-format bin --base-address 0x08000000 --verify {{ flash_image }}
    probe-rs reset --chip {{ chip }}

# Update a board's firmware over USB, through its running firmware. With several
# boards attached, select one: just update --board <serial>
update *board: firmware-image
    {{ just_executable() }} cli firmware update {{ firmware_image }} {{ board }}

# Update every attached board over USB, one after the other
update-all: firmware-image
    #!/usr/bin/env bash
    set -euo pipefail
    for serial in $({{ just_executable() }} cli list | cut -d' ' -f1); do
        {{ just_executable() }} cli firmware update {{ firmware_image }} --board "$serial"
    done

# Update every board on the network
push *board: firmware-image
    {{ just_executable() }} cli firmware push {{ firmware_image }} {{ board }}

# Flash the bootloader and the firmware over USB through the ROM bootloader.
dfu-flash *board: _flash-image
    {{ just_executable() }} cli bootloader {{ board }}
    {{ just_executable() }} _dfu-download -w

# Flash every attached board through the ROM bootloader, one after the other
dfu-flash-all: _flash-image
    #!/usr/bin/env bash
    set -euo pipefail
    for serial in $({{ just_executable() }} cli list | cut -d' ' -f1); do
        echo "Board $serial"
        {{ just_executable() }} cli bootloader --board "$serial"
        {{ just_executable() }} _dfu-download -w
        # Give the board time to leave the ROM bootloader before the next one enters it.
        sleep 3
    done

# Build the firmware as a raw image. Every recipe that touches a board builds it first
firmware-image: embassy-stm32
    ( cd firmware && cargo build --release && {{ objcopy }} -O binary target/thumbv7em-none-eabihf/release/sensor-board-firmware ../{{ firmware_image }} )

# Build the full flash image: the bootloader, padded with 0xff up to where the firmware starts, then the firmware
_flash-image: firmware-image
    ( cd bootloader && cargo build --release && {{ objcopy }} -O binary --gap-fill 0xff --pad-to 0x08007000 target/thumbv7em-none-eabihf/release/sensor-board-bootloader target/sensor-board-bootloader.bin )
    cat bootloader/target/sensor-board-bootloader.bin {{ firmware_image }} > {{ flash_image }}

# Write the flash image to a board in its ROM bootloader, and start it.
_dfu-download *flags:
    # dfu-util exits with an error even when the download works: at ":leave" the
    # chip starts the firmware, so it is gone when dfu-util asks for the final
    # status. Check dfu-util's own download message instead.
    dfu-util {{ flags }} -d 0483:df11 -a 0 -s 0x08000000:leave -D {{ flash_image }} | tee /dev/stderr | grep "File downloaded successfully" > /dev/null

# Attach to a running board to follow its logs, using the debug probe.
attach: firmware-image
    probe-rs attach --chip {{ chip }} --no-catch-reset {{ elf_firmware }}

gdb:
    probe-rs gdb --chip STM32WB55CG {{ elf_firmware }} 2>&1 &
    trap 'kill $(jobs -p)' EXIT
    gdb {{ elf_firmware }} -ex 'target extended-remote :1337'

build *args:
    ( cd protocol && cargo build {{ args }} )
    ( cd firmware && cargo build {{ args }} )
    ( cd bootloader && cargo build {{ args }} )
    ( cd cli && cargo build {{ args }} )

fmt *args:
    ( cd protocol && cargo fmt {{ args }} )
    ( cd firmware && cargo fmt {{ args }} )
    ( cd bootloader && cargo fmt {{ args }} )
    ( cd cli && cargo fmt {{ args }} )
