/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1E0E0000, LENGTH = 0x20000 /* Inside global MRAM0 (secure) */
  FLASH1 (rx) : ORIGIN = 0xE0E0000, LENGTH = 0x20000 /* Inside global MRAM0 (non-secure) */
  FLASH2 (rx) : ORIGIN = 0x3F840000, LENGTH = 0x4000 /* OTP0 (secure) */
  FLASH3 (rx) : ORIGIN = 0x2F840000, LENGTH = 0x4000 /* OTP0 (non-secure) */
  EXTFLASH (rx) : ORIGIN = 0x70000000, LENGTH = 0x20000000
  RAM (rwx) : ORIGIN = 0x3FC00000, LENGTH = 0x4000 /* All of GRAM30 (secure) */
  RAM1 (rwx) : ORIGIN = 0x2FC00000, LENGTH = 0x4000 /* All of GRAM30 (non-secure) */
  RAM2 (rwx) : ORIGIN = 0x3F000000, LENGTH = 0x80000 /* GRAM00 (secure) */
  RAM3 (rwx) : ORIGIN = 0x2F000000, LENGTH = 0x80000 /* GRAM00 (non-secure) */
  RAM4 (rwx) : ORIGIN = 0x3F080000, LENGTH = 0x60000 /* GRAM01 (secure) */
  RAM5 (rwx) : ORIGIN = 0x2F080000, LENGTH = 0x60000 /* GRAM01 (non-secure) */
  RAM6 (rwx) : ORIGIN = 0x3FBF0000, LENGTH = 0x8000 /* GRAM20 (secure) */
  RAM7 (rwx) : ORIGIN = 0x2FBF0000, LENGTH = 0x8000 /* GRAM20 (non-secure) */
  RAM8 (rwx) : ORIGIN = 0x3FBF8000, LENGTH = 0x8000 /* GRAM21 (secure) */
  RAM9 (rwx) : ORIGIN = 0x2FBF8000, LENGTH = 0x8000 /* GRAM21 (non-secure) */
  RAM10 (rwx) : ORIGIN = 0x3FC00000, LENGTH = 0x4000 /* GRAM30 (low-speed) (secure) */
  RAM11 (rwx) : ORIGIN = 0x2FC00000, LENGTH = 0x4000 /* GRAM30 (low-speed) (non-secure) */
  RAM12 (rwx) : ORIGIN = 0x3FC04000, LENGTH = 0x4000 /* GRAM31 (low-speed) (secure) */
  RAM13 (rwx) : ORIGIN = 0x2FC04000, LENGTH = 0x4000 /* GRAM31 (low-speed) (non-secure) */
}


INCLUDE "nrf_common_riscv.ld"
