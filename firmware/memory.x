/* STM32WB55CGU6 (the part on the BOM, LCSC C404023): 1 MB flash, 256 KB SRAM. */
MEMORY
{
  /* The top of flash belongs to FUS and the CPU2 wireless stack (secure area).
     Keep the CPU1 app well below the stack install address from the
     STM32CubeWB release notes; 256 KB is under every 1 MB-device stack. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 252K   /* last 4 KB page = config (config.rs) */
  /* SRAM1 below 0x20024000 only: the CPU2 Thread stack uses the top 48 KB of
     SRAM1 (see _estack in ST's Thread examples). The mailbox lives in SRAM2a
     via RAM_SHARED from embassy-stm32-wpan's tl_mbox.x. */
  RAM   : ORIGIN = 0x20000000, LENGTH = 144K
}
