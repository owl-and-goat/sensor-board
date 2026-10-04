/* STM32WB55CGU6 (the part on the BOM, LCSC C404023): 1 MB flash, 256 KB SRAM.

   The top of flash belongs to FUS and the CPU2 wireless stack (secure area):
   ST's Thread stack starts at 0x0808C000. What is below it is shared out
   between the bootloader, this firmware, and the slot a firmware update is
   staged in. bootloader/memory.x names the same regions: keep the two in
   step. */
MEMORY
{
  BOOTLOADER       : ORIGIN = 0x08000000, LENGTH = 24K
  /* What the bootloader is to do at the next reset, and how far it got. */
  BOOTLOADER_STATE : ORIGIN = 0x08006000, LENGTH = 4K
  /* The firmware. The page after it, at 0x0803F000, is its stored
     configuration (persistent_config.rs), which no update moves or touches. */
  FLASH            : ORIGIN = 0x08007000, LENGTH = 224K
  /* The staged update, and after a swap the firmware it replaced. One page
     longer than the firmware's own region: the swap needs the room. */
  DFU              : ORIGIN = 0x08040000, LENGTH = 228K

  /* SRAM1 below 0x20024000 only: the CPU2 Thread stack uses the top 48 KB of
     SRAM1 (see _estack in ST's Thread examples). The mailbox lives in SRAM2a
     via RAM_SHARED from embassy-stm32-wpan's tl_mbox.x. The bootloader runs
     in the top 16 KB of this, where the firmware has only its stack. */
  RAM              : ORIGIN = 0x20000000, LENGTH = 144K
}

/* Where update.rs finds the two regions it writes, as offsets into flash. */
__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOTLOADER);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOTLOADER);

/* The vector table is 79 words, so .text would land at 0x...13c, four bytes
   off the 8-byte alignment LLVM asks for it. cortex-m-rt PROVIDEs _stext, so
   round it up and spend four bytes of padding instead. */
_stext = ORIGIN(FLASH) + 0x140;
