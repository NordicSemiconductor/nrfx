/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x7D000 /* Inside global RRAM0 */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x18000
}


INCLUDE "nrf_common.ld"
