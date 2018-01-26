/**
 * @file    ic_driver_flash_eon.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_DRIVER_FLASH_EON_H
#define _IC_DRIVER_FLASH_EON_H

#include "ic_driver_flash_general.h"

	/***
	 *
	 * if you are not using polling, you may want to use timeout function
	 * with typical and maximum operation times for EN25QH256 flash memory
	 *
	 ***/
#define EN25QH256_PAGE_PROGRAM_TIME					1
#define EN25QH256_SECTOR_ERASE_TIME					50
#define EN25QH256_BLOCK_ERASE_TIME					400
#define EN25QH256_CHIP_ERASE_TIME						100000

#define EN25QH256_MAX_PAGE_PROGRAM_TIME			5
#define EN25QH256_MAX_SECTOR_ERASE_TIME			300
#define EN25QH256_MAX_BLOCK_ERASE_TIME			2000
#define EN25QH256_MAX_CHIP_ERASE_TIME				28000000

/**
 * @brief EN25QH256 register addresses map
 *
 * Declare enum for registers addresses of EON25QH256
 */
enum
{
	EN25QH256_EQPI_ADDR		                =							0x38,  //!< EN25QH256_EQPI_ADDR
	EN25QH256_RSTQIO_ADDR		              =							0xFF,  //!< EN25QH256_RSTQIO_ADDR
	EN25QH256_RSTEN_ADDR		              =							0x66,  //!< EN25QH256_RSTEN_ADDR
	EN25QH256_RST_ADDR		                =							0x99,  //!< EN25QH256_RST_ADDR
	EN25QH256_W_EN_ADDR		                =							0x06,  //!< EN25QH256_W_EN_ADDR
	EN25QH256_W_DIS_ADDR		              =							0x04,  //!< EN25QH256_W_DIS_ADDR
	EN25QH256_R_STATREG_ADDR			        =							0x05,  //!< EN25QH256_R_STATREG_ADDR
	EN25QH256_R_INFOREG_ADDR              =							0x2B,  //!< EN25QH256_R_INFOREG_ADDR
	EN25QH256_W_STATREG_ADDR	            =							0x01,  //!< EN25QH256_W_STATREG_ADDR
	EN25QH256_ENTER_4BYTE_MODE_ADDR	      =							0xB7,  //!< EN25QH256_ENTER_4BYTE_MODE_ADDR
	EN25QH256_EXIT_4BYTE_MODE_ADDR	      =							0xE9,  //!< EN25QH256_EXIT_4BYTE_MODE_ADDR
	EN25QH256_EN_HIGH_BANK_LATCH_ADDR     =							0x67,  //!< EN25QH256_EN_HIGH_BANK_LATCH_ADDR
	EN25QH256_EXIT_HIGH_BANK_LATCH_ADDR	  =							0x98,  //!< EN25QH256_EXIT_HIGH_BANK_LATCH_ADDR
	EN25QH256_PAGE_PROGRAM_ADDR	          =							0x02,  //!< EN25QH256_PAGE_PROGRAM_ADDR
	EN25QH256_SECTOR_ERASE_ADDR	          =							0x20,  //!< EN25QH256_SECTOR_ERASE_ADDR
	EN25QH256_BLOCK_ERASE_ADDR	          =							0xD8,  //!< EN25QH256_BLOCK_ERASE_ADDR
	EN25QH256_CHIP_ERASE_ADDR	            =							0xC7,  //!< EN25QH256_CHIP_ERASE_ADDR
	EN25QH256_DEPP_PWR_DOWN_ADDR	        =							0xB9,  //!< EN25QH256_DEPP_PWR_DOWN_ADDR
	EN25QH256_RELEASE_DEEP_PWR_DOWN_ADDR  =							0xAB,  //!< EN25QH256_RELEASE_DEEP_PWR_DOWN_ADDR
	EN25QH256_R_DEVICE_ID_ADDR  			    =							0x90,  //!< EN25QH256_R_DEVICE_ID_ADDR
	EN25QH256_R_IDENTIFICATION	          =							0x9F,  //!< EN25QH256_R_IDENTIFICATION
	EN25QH256_ENTER_OTP_ADDR			        =							0x3A,  //!< EN25QH256_ENTER_OTP_ADDR
	EN25QH256_R_SFDP_ADDR				      	  =							0x5A,  //!< EN25QH256_R_SFDP_ADDR
	EN25QH256_R_DATA_ADDR				      	  =							0x03,  //!< EN25QH256_R_DATA_ADDR
	EN25QH256_FAST_READ_ADDR			        =							0x0B,  //!< EN25QH256_FAST_READ_ADDR
	EN25QH256_DUAL_OUT_FAST_READ_ADDR	    =							0x3B,  //!< EN25QH256_DUAL_OUT_FAST_READ_ADDR
	EN25QH256_DUAL_IO_FASTE_READ_ADDR	    =							0xBB,  //!< EN25QH256_DUAL_IO_FASTE_READ_ADDR
	EN25QH256_QUAD_IO_FAST_READ_ADDR	    =							0xEB,  //!< EN25QH256_QUAD_IO_FAST_READ_ADDR
};

  /*	STATUS REGISTER BIT LOCATIONS	 */
#define EON_SR_SRP_BIT        (1<<7)
#define EON_SR_OTP_LOCK_BIT   (1<<6)
#define EON_SR_QE_BIT         (1<<5)
#define EON_SR_BP3_BIT        (1<<4)
#define EON_SR_BP2_BIT        (1<<3)
#define EON_SR_BP1_BIT        (1<<2)
#define EON_SR_WEL_BIT	  		(1<<1)
#define EON_SR_WIP_BIT        (1<<0)


  /*	INFORMATION REGISTER BIT LOCATIONS	*/
#define EON_IR_HBL_BIT				(1<<7)
#define EON_IR_EFF_BIT				(1<<6)
#define EON_IR_PFF_BIT				(1<<5)
#define EON_IR_4BYTE_BIT			(1<<2)
#define EON_IR_OTP_LOCK_BIT		(1<<1)


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
FlashReturnType EON_FlashReadDeviceIdentification(uint32_t *device_id);

/**
 * @brief Uninit flash module
 *
 * @return
 */
FlashReturnType EON_FlashUninit(void);

/**
 * @brief Enbaling WEN latch in Status Register
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType EON_FlashWriteEnable(ic_flash_FLASH_DEVICE_OBJECT *fdo);

/**
 * @brief Disabling WEN latch in Status Register
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType EON_FlashWriteDisable(void);

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
FlashReturnType EON_FlashReadStatusRegister(uint8_t *stat_reg);

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
FlashReturnType EON_FlashWriteStatusRegister(uint8_t write_data, ic_flash_FLASH_DEVICE_OBJECT *fdo);


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
FlashReturnType EON_FlashPageProgram(uint32_t address, uint8_t *data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);

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
FlashReturnType EON_FlashPageProgramSafety(uint32_t address, uint8_t *data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);

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
FlashReturnType EON_FlashDataRead(uint32_t address, uint8_t *read_data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType EON_FlashDataReadHS(uint32_t);

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
FlashReturnType EON_FlashSectorErase(uint32_t sector_address, ic_flash_FLASH_DEVICE_OBJECT *fdo);

/**
 *
 * @param block_address - address of block you want to erase from flash (please refer to pages 7 - 14)
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType EON_FlashBlockErase(uint16_t block_address, ic_flash_FLASH_DEVICE_OBJECT *fdo);

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
FlashReturnType EON_FlashReadInformationReg(uint8_t *info_reg);

/**
 * @brief Erase all flash chip
 *
 * Example:
 * @code
 * FlashChipErase();
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType EON_FlashChipErase(ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType EON_FlashEnableQuadIO(void);
FlashReturnType EON_FlasResetEnable(void);
FlashReturnType EON_FlashEnter4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType EON_FlashExit4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo);
FlashReturnType EON_EnterHighBankLatchMode(void);
FlashReturnType EON_ExitHighBankLatchMode(void);
FlashReturnType EON_FlashEnterDeepPwrDown(void);
FlashReturnType EON_FlashExitDeepPwrDown(void);

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
FlashReturnType EON_FlashReadDeviceID(uint32_t *deviceID);

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
FlashReturnType EON_FlashGenWrite(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr, uint8_t *data_input, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);

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
FlashReturnType EON_FlashGenWriteSafety(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr, uint8_t *data_input, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo);

FlashReturnType EON_FlashSoftReset(void);
#endif /*  IC_DRIVER_FLASH_EON_H  */
