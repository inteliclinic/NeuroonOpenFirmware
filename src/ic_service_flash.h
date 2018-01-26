/**
 * @file    ic_flash.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_FLASH_H
#define IC_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include  "ic_common_types.h"

#include  "ic_driver_flash.h"

/**
 *
 */
#define _FLASH_TEST
//#define _FLASH_FILESYSTEM_TEST



/**
 * @brief Initialize flash memory device
 *
 * To properly initialize flash you have to call some functions like:
 * 	-> ic_uart_init() - for being able to print data via uart interface
 * 	-> configure_gpio() - gpio configuration for flash spi interface
 *
 * 	!!! if you want to use fatfs, you need to allocate 4kB space in freeRTOS system !!!
 *
 * 	-> flash_init_task()
 * 		- initialize flash driver
 * 		- create and start timer
 *
 *  -> create waiting task, which will be resuming when flash is busy (for example sector erase time is relatively long)
 *  	 In waiting task you have to polling checking is flash busy -
 *  	 if yes, user can specify what to do in this time,
 *  	 if no, you can suspend waiting task and return to the latest operation
 */
ic_return_val_e ic_flash_init(void);

/**
 * @brief Deinitialize flash memory device
 *
 *
 */
ic_return_val_e ic_flash_deinit(void);

#ifdef _FLASH_TEST

ic_return_val_e ic_flash_test();

#endif

#ifdef _FLASH_FILESYSTEM_TEST
/**
 * @brief Testing flash function
 *
 * Function which is testing basic flash operations like:
 * 	-> erase_sector_from_flash()
 * 	-> write_to_flash()
 * 	-> read_from_flash() / read_from_flash_specific()
 *
 * Firstly, it checks these functions by erasing specific sector from flash,
 * then writes your defined signature characters and finally read it from that address
 * If read value is the same as defined signature, first step is correct
 *
 * Secondly, it erases sector 0 from flash, writes 256 bytes of data and reads it.
 * After that, it checks byte by byte if writing data and read from flash are the same.
 * If yes, flash_test_done variable is set to true
 */
ic_return_val_e flash_test_write(void);

/**
 * @brief Flash task
 *
 * Function for testing fatfs_write_sector and fatfs_read_sector (without any controlling mechanism)
 */
void flash_test_read(void);

#endif
/**
 * @brief Write to flash
 *
 * @param address
 * @param data_to_write
 * @param data_len
 * @param cb
 */
ic_return_val_e ic_flash_write(uint32_t address, uint8_t *data_to_write, size_t data_len, func_finished cb);

/**
 * @brief Erase sector from flash
 *
 * @param address
 * @param cb
 */
ic_return_val_e ic_flash_sector_erase(uint32_t address, func_finished cb);

/**
 * @brief Read from flash
 *
 * @param address
 * @param read_data
 * @param len
 * @param cb
 */
ic_return_val_e ic_flash_read(uint32_t address, uint8_t *read_data, size_t len, func_finished cb);

/**
 * @brief Read from flash by giving specific address
 *
 * @param block_address
 * @param sector_address
 * @param page_address
 * @param read_data
 * @param len
 * @param cb
 *
 * Address is divided into block, sector and page address
 *
 */
ic_return_val_e ic_flash_read_specific(uint16_t block_address, uint8_t sector_address, uint8_t page_address, uint8_t *read_data, size_t len,
		func_finished cb);

/**
 * @brief Erase flash chip
 *
 * @param cb
 * @return
 */
ic_return_val_e ic_flash_erase_chip(func_finished cb);

	#ifdef _FATFS_USE

/** @defgroup FATFS_FLASH_API fatfs functions connected with flash module
 *  @ingroup HIGH_LEVEL_FLASH_API
 *
 */
	/**
	 * @brief Write fatfs sector
	 *
	 *
	 * @param sector	- address of fatfs sector you want to write to
	 * @param buff		- input buffer to write
	 *
	 * @return true if sector was written correctly
	 */
	bool fatfs_write_sector(uint32_t sector, uint8_t *buff);

	/**
	 * @brief Read fatfs sector
	 *
	 *
	 * @param sector 	- address of fatfs sector you want to read
	 * @param buff 		- buffer to store data
	 *
	 * @return true if secotr was read correctly
	 */
	bool fatfs_read_sector(uint32_t sector, uint8_t *buff);

	/**
	 * @brief Check whether fatfs sector is empty
	 *
	 * @param sector - address of fatfs sector
	 *
	 * @return true if sector is empty
	 */
	bool fatfs_is_sector_empty(uint32_t sector);

	/**
	 * @brief Init fatfs
	 *
	 * @return true if initialization is okay
	 */
	bool fatfs_init(void);
			/**
			 * @brief Fatfs example function
			 *
			 * Function for testing writing to flash memory using fatfs.
			 */
		void fat_example(void);
	#endif /* !_FATFS_USE  */
#endif /* !IC_FLASH_H */
