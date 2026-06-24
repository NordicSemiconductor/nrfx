/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x2E0000, LENGTH = 0x10000 /* Inside MRAM */
  RAM (rwx) : ORIGIN = 0x28000000, LENGTH = 0x20000
}


INCLUDE "nrf_common_riscv.ld"
