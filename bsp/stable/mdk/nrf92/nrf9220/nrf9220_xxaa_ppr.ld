/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0xE0E0000, LENGTH = 0x20000 /* Inside global MRAM0 */
  FLASH1 (rx) : ORIGIN = 0x2F840000, LENGTH = 0x4000 /* OTP0 */
  RAM (rwx) : ORIGIN = 0x2FC00000, LENGTH = 0x3C00 /* Inside global RAM30 */
  RAM1 (rwx) : ORIGIN = 0x2F000000, LENGTH = 0x40000 /* global RAM00 */
  RAM2 (rwx) : ORIGIN = 0x2F040000, LENGTH = 0x40000 /* global RAM01 */
  RAM3 (rwx) : ORIGIN = 0x2F880000, LENGTH = 0x10000 /* global RAM20 */
  RAM4 (rwx) : ORIGIN = 0x2F890000, LENGTH = 0x10000 /* global RAM21 */
  RAM5 (rwx) : ORIGIN = 0x2FC00000, LENGTH = 0x10000 /* global RAM30 (low-speed) */
  RAM6 (rwx) : ORIGIN = 0x2FC10000, LENGTH = 0x4000 /* global RAM31 (low-speed) */
}


INCLUDE "nrf_common_riscv.ld"
