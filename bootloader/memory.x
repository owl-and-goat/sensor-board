/* The flash below ST's radio stack (which starts at 0x0808C000) is divided
   between this bootloader, the firmware, and the staging area for a firmware
   update. firmware/memory.x defines the same regions: keep the two files in
   sync. */
MEMORY
{
  FLASH            : ORIGIN = 0x08000000, LENGTH = 24K
  /* The bootloader's instruction for the next reset, and its swap progress. */
  BOOTLOADER_STATE : ORIGIN = 0x08006000, LENGTH = 4K
  /* The running firmware. It is linked to start here. */
  ACTIVE           : ORIGIN = 0x08007000, LENGTH = 224K
  /* 0x0803F000, 4K: the firmware's stored configuration. An update never
     moves or changes it. */
  /* The staging area: a staged update, and after a swap the firmware that
     was replaced. One page longer than ACTIVE, because the swap needs a
     spare page. */
  DFU              : ORIGIN = 0x08040000, LENGTH = 228K

  /* The top of the firmware's RAM. Only the firmware's stack is there, and
     none of the words that it keeps across a reset. */
  RAM        (rwx) : ORIGIN = 0x20020000, LENGTH = 16K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(FLASH);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(FLASH);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(FLASH);
