/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x2C0000, LENGTH = 0x20000 /* Inside MRAM */
  RAM (rwx) : ORIGIN = 0x200E0000, LENGTH = 0x20000
  RAM1 (rwx) : ORIGIN = 0x200C0000, LENGTH = 0x20000
}


INCLUDE "nrf_common_riscv.ld"
