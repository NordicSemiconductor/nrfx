/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0xE312000, LENGTH = 0x19D000 /* Application FW + storage + Additional space for App DFU */
  FLASH1 (rx) : ORIGIN = 0xE400000, LENGTH = 0xAF000 /* Common DFU area */
  FLASH2 (rx) : ORIGIN = 0x2F840000, LENGTH = 0x4000 /* OTP0 */
  RAM (rwx) : ORIGIN = 0x22000000, LENGTH = 0x8000 /* Local TCM RAM */
  RAM1 (rwx) : ORIGIN = 0x2F000000, LENGTH = 0x60000
  RAM2 (rwx) : ORIGIN = 0x2F890000, LENGTH = 0x10000
  RAM3 (rwx) : ORIGIN = 0x2FC00000, LENGTH = 0x4000
  RAM4 (rwx) : ORIGIN = 0x2F000000, LENGTH = 0x40000 /* global RAM00 */
  RAM5 (rwx) : ORIGIN = 0x2F040000, LENGTH = 0x40000 /* global RAM01 */
  RAM6 (rwx) : ORIGIN = 0x2F880000, LENGTH = 0x10000 /* global RAM20 */
  RAM7 (rwx) : ORIGIN = 0x2F890000, LENGTH = 0x10000 /* global RAM21 */
  RAM8 (rwx) : ORIGIN = 0x2FC00000, LENGTH = 0x10000 /* global RAM30 (low-speed) */
  RAM9 (rwx) : ORIGIN = 0x2FC10000, LENGTH = 0x4000 /* global RAM31 (low-speed) */
}


INCLUDE "nrf_common.ld"
