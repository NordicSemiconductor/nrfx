/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x2C0000 /* Inside MRAM */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x80000
  RAM1 (rwx) : ORIGIN = 0x20080000, LENGTH = 0x40000
  RAM2 (rwx) : ORIGIN = 0x200C0000, LENGTH = 0x20000
  RAM3 (rwx) : ORIGIN = 0x200E0000, LENGTH = 0x1E000
}


INCLUDE "nrf_common.ld"
