/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1B000, LENGTH = 0x1F800
  SERIAL_NUMBER (r) : ORIGIN = 0x3A800, LENGTH = 0x400
  /*RAM (rwx) :  ORIGIN = 0x20002078, LENGTH = 0x5f88*/
  RAM (rwx) :  ORIGIN = 0x20001ff8, LENGTH = 0x6008

  /** Location of bootloader setting in at the last flash page. */
  BOOTLOADER_SETTINGS (rw) : ORIGIN = 0x0003FC00, LENGTH = 0x0400

  /** Location in UICR where bootloader start address is stored. */
  UICR_BOOTLOADER (r) : ORIGIN = 0x10001014, LENGTH = 0x04
}


SECTIONS
{
  .serialNumber(NOLOAD) :
  {
    PROVIDE(__start_serial_number = .);
    KEEP(*(.serialNumber))
    PROVIDE(__stop_serial_number = .);
  } > SERIAL_NUMBER
  /* Ensures the bootloader settings are placed at the last flash page. */
  .bootloaderSettings(NOLOAD) :
  {

  } > BOOTLOADER_SETTINGS

  /* Ensures the Bootloader start address in flash is written to UICR when flashing the image. */
  .uicrBootStartAddress :
  {
    KEEP(*(.uicrBootStartAddress))
  } > UICR_BOOTLOADER

  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(.pwr_mgmt_data))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "sdk/components/toolchain/gcc/nrf51_common.ld"
