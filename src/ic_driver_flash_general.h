/**
 * @file    ic_driver_flash_general.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_DRIVER_FLASH_GENERAL_H
#define _IC_DRIVER_FLASH_GENERAL_H

/** @defgroup LOW_LEVEL_FLASH_GENERAL_INFO general information of flash memory
 *  @{
 */

/**
 * @brief Define enumeration for number of bytes to send via SPI interface
 */
enum
{
  FLASH_SEND_1BYTE   =  1,    //!< FLASH_SEND_1BYTE
  FLASH_SEND_2BYTES  =  2,    //!< FLASH_SEND_2BYTES
  FLASH_SEND_4BYTES  =  4,    //!< FLASH_SEND_4BYTES
  FLASH_SEND_6BYTES  =  6,    //!< FLASH_SEND_6BYTES
  FLASH_SEND_PAGE    =  256,  //!< FLASH_SEND_PAGE
};

/**
 * @brief Flash return type
 *
 * Enumeration contains type of data returning from flash functions
 **/
typedef enum
{
  Flash_Success,
  Flash_Init_Success,
  Flash_Init_Failed,
  Flash_Write_Enabled,
  Flash_AddressInvalid,
  Flash_PageEraseFailed,
  Flash_SectorNrInvalid,
  Flash_OperationOngoing,
  Flash_OperationTimeOut,
  Flash_ProgramFailed,
  Flash_WrongType,
  Flash_StatRegBlocked,
  Flash_WrongBlockNum,
  Flash_WrongDataSize,
  Flash_Error
} FlashReturnType;

/**
 * Macros for flash addressing mode
 **/
typedef enum
{
  FLASH_3_BYTE_ADDR_MODE  = 0x03,     /* 3 byte address */
  FLASH_4_BYTE_ADDR_MODE  = 0x04      /* 4 byte address */
} AddressMode;

	/*	Structure for flash object - description and operation	*/
struct _FLASH_DEVICE_OBJECT;

typedef struct _FLASH_DEVICE_OBJECT ic_flash_FLASH_DEVICE_OBJECT;
typedef struct _FLASH_DESCRIPTION   ic_flash_FLASH_DESCRIPTION;
typedef struct _FLASH_OPERATION     ic_flash_FLASH_OPERATION;

/**
 * @brief Flash description
 *
 * Structure for description data of used flash
 **/
struct _FLASH_DESCRIPTION
{
  uint32_t  FlashID;
  uint8_t   FlashType;
  uint32_t  StartingAddress;
  uint32_t  FlashAddressMask;
  uint32_t  FlashBlockCount;
  uint32_t  FlashBlockSize_bit;
  uint32_t  FlashSectorCount;
  uint32_t  FlashSubSectorCount;
  uint32_t  FlashSubSectorSize_bit;
  uint32_t  FlashPageSize;
  uint8_t   FlashPageSize_bit;
  uint32_t  FlashPageCount;
  uint32_t  FlashSectorSize;
  uint32_t  FlashSectorSize_bit;
  uint32_t  FlashSubSectorSize;
  uint32_t  FlashSize;
  uint32_t  FlashOTPSize;
  uint8_t   FlashDieCount;
  uint32_t  FlashDieSize;
  uint32_t  FlashDieSize_bit;
  uint32_t  Size;
  uint32_t  BufferSize;
  uint8_t   DataWidth;
  AddressMode NumAddrByte;
};

/**
 * @brief Flash operation
 *
 * Structure for flash operations functions
 */
struct _FLASH_OPERATION
{
  FlashReturnType (*FlashReadDeviceIdentification) (uint32_t *);
  FlashReturnType (*FlashWriteEnable) (ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashWriteDisable) (void);
  FlashReturnType (*FlashReadStatusRegister) (uint8_t *);
  FlashReturnType (*FlashWriteStatusRegister) (uint8_t, ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashPageProgram) (uint32_t, uint8_t *, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashDataRead) (uint32_t, uint8_t *, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashDataReadHS) (uint32_t);
  FlashReturnType (*FlashSectorErase) (uint32_t, ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashBlockErase) (uint16_t, ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashReadInformationReg) (uint8_t *);
  FlashReturnType (*FlashChipErase) (ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashEnableQuadIO) (void);
  FlashReturnType (*FlasResetEnable) (void);
  FlashReturnType (*FlashEnter4ByteMode) (ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashExit4ByteMode) (ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*EnterHighBankLatchMode) (void);
  FlashReturnType (*ExitHighBankLatchMode) (void);
  FlashReturnType (*FlashEnterDeepPwrDown) (void);
  FlashReturnType (*FlashExitDeepPwrDown) (void);
  FlashReturnType (*FlashReadDeviceID) (uint32_t *);
  FlashReturnType (*FlashGenWrite) (uint16_t, uint8_t, uint8_t, uint8_t *, size_t, ic_flash_FLASH_DEVICE_OBJECT *fdo);
  FlashReturnType (*FlashUninit) (void);
  FlashReturnType (*FlashSoftReset) (void);
  /**
   * Describe here more operations if you are using them
   **/
};

/**
 * @brief Flash device object
 *
 * Device object structure holding device description and operations
 */
struct _FLASH_DEVICE_OBJECT
{
  ic_flash_FLASH_DESCRIPTION   Desc;
  ic_flash_FLASH_OPERATION     GenOp;
//  char not_used[72];
};

/**
 * @brief Spi package to send
 *
 * Structure for pushing instruction, address and data to flash
 */
typedef struct __packed
{
  uint8_t inst;				 // 1 byte of instruction
  uint8_t address[4];  // 3 or 4 bytes of address
}s_spiSendToFlash;

/**
 *  @}
 */
#endif /*  __IC_DRIVER_FLASH_GENERAL_H  */
