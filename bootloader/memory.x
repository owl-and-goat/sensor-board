/* The flash below ST's radio stack (which starts at 0x0808C000), shared out
   between this bootloader, the firmware, and the slot a firmware update is
   staged in. firmware/memory.x names the same regions: keep the two in step. */
MEMORY
{
  FLASH            : ORIGIN = 0x08000000, LENGTH = 24K
  /* What the bootloader is to do at the next reset, and how far it got. */
  BOOTLOADER_STATE : ORIGIN = 0x08006000, LENGTH = 4K
  /* The firmware that runs. It is linked to start here. */
  ACTIVE           : ORIGIN = 0x08007000, LENGTH = 224K
  /* 0x0803F000, 4K: the firmware's stored configuration, which no update
     moves or touches. */
  /* The staged update, and after a swap the firmware it replaced. One page
     longer than ACTIVE: the swap needs the room. */
  DFU              : ORIGIN = 0x08040000, LENGTH = 228K

  /* The top of what the firmware has for RAM: its stack is there, and none
     of the words that it keeps across a reset. */
  RAM        (rwx) : ORIGIN = 0x20020000, LENGTH = 16K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(FLASH);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(FLASH);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(FLASH);
