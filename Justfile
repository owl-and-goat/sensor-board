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

# Fetch ST's coprocessor images, unless they are here already, and check them
coprocessor-images: (_image fus_image "bbab9b42d92c31f8a04e2f04ef9361c3f6f01714c14408cd34fb1583a6edf30d") (_image stack_image "53babb15ecb9da0961aacba612be0b1273e79271232348c545f1bb87c73b9e84")

_image file sha256:
  @mkdir -p {{parent_directory(file)}}
  @[ -f {{file}} ] || { echo "Fetching {{file_name(file)}}"; curl -fL --remove-on-error -o {{file}} {{coprocessor_url}}/{{file_name(file)}}; }
  @echo "{{sha256}}  {{file}}" | sha256sum --check --quiet

# Set up a new board over USB: our firmware on CPU1, then FUS and the Thread stack on CPU2. A blank chip sits in its ROM bootloader, which is where this starts
new-board: coprocessor-images
  ( cd firmware && cargo build --release && "$(rustc --print sysroot)"/lib/rustlib/*/bin/llvm-objcopy -O binary target/thumbv7em-none-eabihf/release/sensor-board-firmware target/sensor-board-firmware.bin )
  # dfu-util ends in an error whether this works or not: at ":leave" the chip starts the firmware, and is gone when dfu-util asks it how that went. So go by its own report of the download.
  dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D firmware/target/sensor-board-firmware.bin | tee /dev/stderr | grep "File downloaded successfully" > /dev/null
  {{just_executable()}} cli coprocessor install {{fus_image}}
  {{just_executable()}} cli coprocessor install {{stack_image}}

attach:
    probe-rs attach --chip STM32WB55CG --no-catch-reset firmware/target/thumbv7em-none-eabihf/release/sensor-board-firmware

build *args:
    ( cd protocol && cargo build {{args}} )
    ( cd firmware && cargo build {{args}} )
    ( cd cli && cargo build {{args}} )
