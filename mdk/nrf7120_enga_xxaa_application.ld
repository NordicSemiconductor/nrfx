/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x2C0000 /* Inside MRAM */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0xE0000
  RAM1 (rwx) : ORIGIN = 0x200E0000, LENGTH = 0x20000
}


INCLUDE "nrf_common.ld"
