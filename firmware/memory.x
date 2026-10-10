/* STM32WB55CGU6 (the part on the BOM, LCSC C404023): 1 MB flash, 256 KB SRAM.

   The top of flash belongs to FUS and the CPU2 wireless stack (secure area):
   ST's Thread stack starts at 0x0808C000. The flash below it is divided
   between the bootloader, this firmware, and the staging area for a firmware
   update. bootloader/memory.x defines the same regions: keep the two files
   in sync. */
MEMORY
{
  BOOTLOADER       : ORIGIN = 0x08000000, LENGTH = 24K
  /* The bootloader's instruction for the next reset, and its swap progress. */
  BOOTLOADER_STATE : ORIGIN = 0x08006000, LENGTH = 4K
  /* The firmware. The page after it, at 0x0803F000, holds the stored
     configuration (persistent_config.rs). An update never moves or changes
     that page. */
  FLASH            : ORIGIN = 0x08007000, LENGTH = 224K
  /* The staging area: a staged update, and after a swap the firmware that
     was replaced. One page longer than the firmware's own region, because
     the swap needs a spare page. */
  DFU              : ORIGIN = 0x08040000, LENGTH = 228K

  /* SRAM1 below 0x20024000 only: the CPU2 Thread stack uses the top 48 KB of
     SRAM1 (see _estack in ST's Thread examples). The mailbox lives in SRAM2a
     via RAM_SHARED from embassy-stm32-wpan's tl_mbox.x. The bootloader uses
     the top 16 KB of this region, where the firmware only has its stack. */
  RAM              : ORIGIN = 0x20000000, LENGTH = 144K
}

/* The two regions that update.rs writes, as offsets into flash. */
__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOTLOADER);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOTLOADER);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOTLOADER);

/* The vector table is 79 words, so .text would land at 0x...13c, four bytes
   off the 8-byte alignment LLVM asks for it. cortex-m-rt PROVIDEs _stext, so
   round it up and spend four bytes of padding instead. */
_stext = ORIGIN(FLASH) + 0x140;
