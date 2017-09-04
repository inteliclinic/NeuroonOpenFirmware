/**
 * @file    ic_driver_micron.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_DRIVER_FLASH_MICRON_H
#define _IC_DRIVER_FLASH_MICRON_H

#include "ic_driver_flash_general.h"

/** @defgroup LOW_LEVEL_DRIVER_MICRON low level driver for N25Q256QA flash memory
 *  @{
 */

/** @defgroup FLASH_MICRON_REGISTERS_ADDRESS registers address in N25Q256A flash memory
 *  @ingroup LOW_LEVEL_DRIVER_MICRO
 *  @{
 *
 *  @brief N25Q256A register addresses
 *
 * Enumeration contains addresses of N25Q256A flash memory registers
 */
enum
{
	N25Q256A_RESET_EN										 =		0x66,  //!< N25Q256A_RESET_EN
	N25Q256A_RESET_MEM									 =		0x99,  //!< N25Q256A_RESET_MEM
	N25Q256A_READ_ID										 =		0x9F,  //!< N25Q256A_READ_ID
	N25Q256A_READ_ID2										 =		0xAF,  //!< N25Q256A_READ_ID2
	N25Q256A_READ_SERIAL_FLASH					 =		0x5A,  //!< N25Q256A_READ_SERIAL_FLASH
	N25Q256A_READ												 =		0x03,  //!< N25Q256A_READ
	N25Q256A_FAST_READ									 =		0x0B,  //!< N25Q256A_FAST_READ
	N25Q256A_FAST_READ_DTR							 =		0x0D,  //!< N25Q256A_FAST_READ_DTR
	N25Q256A_4BYTE_READ									 =		0x13,  //!< N25Q256A_4BYTE_READ
	N25Q256A_4BYTE_FAST_READ						 =		0x0C,  //!< N25Q256A_4BYTE_FAST_READ
	N25Q256A_WRITE_EN										 =		0x06,  //!< N25Q256A_WRITE_EN
	N25Q256A_WRITE_DIS									 =		0x04,  //!< N25Q256A_WRITE_DIS
	N25Q256A_READ_STAT_REG							 =		0x05,  //!< N25Q256A_READ_STAT_REG
	N25Q256A_WRITE_STAT_REG							 =		0x01,  //!< N25Q256A_WRITE_STAT_REG
	N25Q256A_READ_LOCK_REG							 =		0xE8,  //!< N25Q256A_READ_LOCK_REG
	N25Q256A_WRITE_LOCK_REG							 =		0xE5,  //!< N25Q256A_WRITE_LOCK_REG
	N25Q256A_READ_FLAG_STATUS_REG				 =		0x70,  //!< N25Q256A_READ_FLAG_STATUS_REG
	N25Q256A_CLEAR_FLAG_STATUS_REG			 =		0x50,  //!< N25Q256A_CLEAR_FLAG_STATUS_REG
	N25Q256A_READ_NONVOLATILE_CONF_REG	 =		0xB5,  //!< N25Q256A_READ_NONVOLATILE_CONF_REG
	N25Q256A_WRITE_NONVOLATILE_CONF_REG	 =		0xB1,  //!< N25Q256A_WRITE_NONVOLATILE_CONF_REG
	N25Q256A_READ_VOLATILE_CONF_REG			 =		0x85,  //!< N25Q256A_READ_VOLATILE_CONF_REG
	N25Q256A_WRITE_VOLATILE_CONF_REG		 =		0x81,  //!< N25Q256A_WRITE_VOLATILE_CONF_REG
	N25Q256A_READ_EXT_ADDR_REG					 =		0xC8,  //!< N25Q256A_READ_EXT_ADDR_REG
	N25Q256A_WRITE_EXT_ADDR_REG					 =		0xC5,  //!< N25Q256A_WRITE_EXT_ADDR_REG
	N25Q256A_PAGE_PROG									 =		0x02,  //!< N25Q256A_PAGE_PROG
	N25Q256A_4BYTE_PAGE_PROG						 =		0x12,  //!< N25Q256A_4BYTE_PAGE_PROG
	N25Q256A_SUBSECTOR_ERASE						 =		0x20,  //!< N25Q256A_SUBSECTOR_ERASE
	N25Q256A_4BYTE_SUBSECTOR_ERASE			 =		0x21,  //!< N25Q256A_4BYTE_SUBSECTOR_ERASE
	N25Q256A_SECTOR_ERASE								 =		0xD8,  //!< N25Q256A_SECTOR_ERASE
	N25Q256A_4BYTE_SECTOR_ERASE					 =		0xDC,  //!< N25Q256A_4BYTE_SECTOR_ERASE
	N25Q256A_BULK_ERASE									 =		0xC7,  //!< N25Q256A_BULK_ERASE
	N25Q256A_PROG_ERASE_RESUME					 =		0x7A,  //!< N25Q256A_PROG_ERASE_RESUME
	N25Q256A_PROG_ERASE_SUSPEND					 =		0x75,  //!< N25Q256A_PROG_ERASE_SUSPEND
	N25Q256A_READ_OTP_ARRAY							 =		0x4B,  //!< N25Q256A_READ_OTP_ARRAY
	N25Q256A_PROGRAM_OTP_ARRAY					 =		0x42,  //!< N25Q256A_PROGRAM_OTP_ARRAY
	N25Q256A_ENTER_4BYTE_ADDR_MODE			 =		0xB7,  //!< N25Q256A_ENTER_4BYTE_ADDR_MODE
	N25Q256A_EXIT_4BYTE_ADDR_MODE				 =		0xE9   //!< N25Q256A_EXIT_4BYTE_ADDR_MODE
};

	/*	STATUS REGISTER BIT LOCATIONS	*/
#define MICRON_SR_SRP_BIT        (1<<7)
#define MICRON_SR_OTP_LOCK_BIT   (1<<6)
#define MICRON_SR_QE_BIT         (1<<5)
#define MICRON_SR_BP3_BIT        (1<<4)
#define MICRON_SR_BP2_BIT        (1<<3)
#define MICRON_SR_BP1_BIT        (1<<2)
#define MICRON_SR_WEL_BIT		  	 (1<<1)
#define MICRON_SR_WIP_BIT        (1<<0)

	/*	INFORMATION REGISTER BIT LOCATIONS	*/
#define MICRON_IR_HBL_BIT				 (1<<7)
#define MICRON_IR_EFF_BIT				 (1<<6)
#define MICRON_IR_PFF_BIT				 (1<<5)
#define MICRON_IR_OTP_LOCK_BIT	 (1<<1)
#define MICRON_IR_4BYTE_BIT			 (1<<0)

/**
 * @brief Read device identification number
 * @param device_id
 *
 * Example:
 * @code
 * uint32_t device_id = 0;
 * FlashReadDeviceIdentification2(&device_id);
 * printf("Device ID is: 0x%02lX", device_id);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashReadDeviceIdentification(uint32_t *device_id);
/***************************************************************************************/
/**
 * @brief Uninit flash module
 *
 * @return
 */
FlashReturnType MICRON_FlashUninit(void);
/***************************************************************************************/
/**
 * @brief Enbaling WEN latch in Status Register
 *
 * @param fdo
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashWriteEnable(ic_flash_FLASH_DEVICE_OBJECT *fdo);
/***************************************************************************************/
/**
 * @brief Disabling WEN latch in Status Register
 *
 * @return
 */
FlashReturnType MICRON_FlashWriteDisable(void);
/***************************************************************************************/
/**
 * @brief Read Statug Register
 *
 * @param stat_reg - pointer to read data from status register
 *
 * Example:
 * @code
 * uint8_t stat_reg = 0;
 * FlashReadStatusRegister(&stat_reg);
 * printf("Value of the status register is: 0x%02X\r\n", stat_reg);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashReadStatusRegister(uint8_t *stat_reg);
/***************************************************************************************/
/**
 * @brief Write to status register
 *
 * @param write_data - data to write to status register
 *
 * Example:
 * @code
 * uint8_t write_data = 0;
 * FlashWriteStatusRegister(write_data);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashWriteStatusRegister(uint8_t write_data, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/***************************************************************************************/

/*
 *                                                GENERIC INFORMATION
 *
 *  We've got 512 blocks in which we've got 16 sectors in each. In each sector we can use 16 pages, 256 data bytes each.
 *  Address mode is 3 or 4 byte. So you have to know exactly to what address you want to write!
 *
 */
/**
 * @brief Program page in flash
 *
 * Write data to the page in flash memory
 *
 *
 * @param address - address to which you want to write data
 * @param data		- data you want to write
 * @param len			- length of data you want to write
 *
 * Example:
 * @code
 * 	uint32_t address = 0x009C00;
 * 	uint8_t data_to_write[] = { Hello world! };
 *
 * 	FlashPageProgram(address, data_to_write, sizeof(data_to_write));
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashPageProgram(uint32_t address, uint8_t *data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/*******************************************************************************************************************************/
/**
 * @brief Program page in flash but in more safety way
 * 				(checking while getting inside the function whether flash is busy)
 *
 * 				More safety but takes more time
 *
 * @param address
 * @param data
 * @param len
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashPageProgramSafety(uint32_t address, uint8_t *data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/*********************************************************************************************************************************/
/**
 * @brief Read data from flash
 *
 * @param address - address from which you want to read data
 * @param read_data - pointer to read data
 *
 * Example:
 * @code
 * uint32_t address = 0x009C00;
 * uint8_t read_data[100] = {0};
 * size_t len = 100;
 *
 * FlashDataRead(address, read_data, len);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashDataRead(uint32_t address, uint8_t *read_data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/********************************************************************************************************************************/
FlashReturnType MICRON_FlashDataReadHS(uint32_t);
/***************************************************************************************/

/**
 * @brief Erase sector from flash
 *
 * @param sector_address - address of sector you want to erase (please refer to datasheet page 7 - 14)
 *
 * Example:
 * @code
 * uint32_t sector_address = 0x01;
 * FlashSectorErase(sector_address);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashSectorErase(uint32_t sector_address, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/********************************************************************************************************************************/
/**
 *
 * @param block_address - address of block you want to erase from flash (please refer to pages 7 - 14)
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashBlockErase(uint16_t block_address, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/********************************************************************************************************************************/
/**
 * @brief Read information register
 *
 * @param info_reg - data stored in information register
 *
 * Example:
 * @code
 * uint8_t info_reg = 0;
 * FlashReadInformationReg(&info_reg);
 * printf("Information register value: 0x%02X", info_reg);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashReadInformationReg(uint8_t *info_reg);
/********************************************************************************************************************************/
/**
 * @brief Erase all flash chip
 *
 * Example:
 * @code
 * FlashChipErase();
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashChipErase(ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType MICRON_FlashEnableQuadIO(void);
FlashReturnType MICRON_FlasResetEnable(void);
FlashReturnType MICRON_FlashEnter4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType MICRON_FlashExit4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType MICRON_EnterHighBankLatchMode(void);
FlashReturnType MICRON_ExitHighBankLatchMode(void);
//FlashReturnType MICRON_FlashEnterDeepPwrDown(void);

/**
 * @brief Read device Identification
 *
 * @param deviceID - stored value of device ID
 *
 * Example:
 * @code
 * uint32_t deviceID = 0;
 * FlashReadDeviceID(&deviceID);
 * printf("Device ID: 0x%06X", deviceID);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashReadDeviceID(uint32_t *deviceID);
/********************************************************************************************************************************/
/**
 * @brief Write data to flash in specific way by giving the block, sector and page address
 *
 * @param block_addr
 * @param sector_addr
 * @param page_addr
 * @param data_input
 * @param len
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashGenWrite(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr, uint8_t *data_input, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/********************************************************************************************************************************/
/**
 * @brief Write to flash like in FlashGenWrite but more safety (takes more time)
 *
 * @param block_addr
 * @param sector_addr
 * @param page_addr
 * @param data_input
 * @param len
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType MICRON_FlashGenWriteSafety(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr, uint8_t *data_input, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
/********************************************************************************************************************************/
/**
 * @}
 */
#endif	/*	IC_DRIVER_FLASH_MICRON_H	*/
