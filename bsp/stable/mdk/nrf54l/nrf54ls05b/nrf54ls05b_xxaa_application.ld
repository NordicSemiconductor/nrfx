/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x7F000 /* Inside global RRAM0 */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x8000
  RAM1 (rwx) : ORIGIN = 0x20008000, LENGTH = 0x10000
}


INCLUDE "nrf_common.ld"
