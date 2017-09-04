/**
 * @file    ic_driver_flash.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_DRIVER_FLASH_H
#define _IC_DRIVER_FLASH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "ic_driver_flash_general.h"

/* manufacturer id + mem type + mem capacity  */
#define MEM_TYPE_EN25Q256  					0x1C7019  /* Manufacturer ID  for EN25QH256 (eon) device */
#define MEM_TYPE_N25Q256A	 					0x20BA19	 /* Manufacturer ID for N25Q256A device	*/

#define FLASH_BUFFER_SIZE_MAX       256
#define FLASH_BUFFER_SIZE_TO_SEND   256

#define FATFS_SECTOR_LENGTH					512

typedef unsigned char BOOL;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL_PTR
#define NULL_PTR 0x0
#endif

/** @defgroup HIGH_LEVEL_FLASH_API high level API for flash memory
 *  @{
 */
	/* manufacturer id + mem type + mem capacity  */
#define MEM_TYPE_EN25Q256  0x1C7019  /* Manufacturer ID  for EN25QH256 (eon) device */
#define MEM_TYPE_N25Q256A  0x20BA19	/* Manufacturer ID for N25Q256A device	*/

	/*  Max flash page's and sector's sizes  */
/** @brief Flash's defined sizes
 *
 * Flash memory management:
 * 	-> page   - 256 bytes
 * 	-> sector - 4kB
 * 	-> block  -
 * 	*/
#define FLASH_BUFFER_SIZE_MAX   256
#define FLASH_SECOTR_SIZE	4096

/**	enable using fatfs functions
	*		Enabled  (1)
	*		Disabled (0)
	**/
#define _USE_FATFS  1
#if _USE_FATFS
	#define FATFS_SECTOR_LENGTH  512
#endif


typedef volatile struct _FLASH_STATE ic_flash_state;
typedef void (*func_finished)(ic_flash_state *m_flash_state);

/**
 * @brief Flash state structure
 *
 * Structure for holding flash state information
 */
struct _FLASH_STATE
{
  enum
  {
    write_state,
    read_state,
    erase_state,
    nop
  }state;
  func_finished callback;
};

/**
 * @brief SPI callback function
 *
 * @param context
 *
 * To correctly use ic_spi_driver SPI_SEND_DATA functions, you need to declare callback function
 * Callback is called when spi transaction is done
 */
void callback_spi(void *context);
/**
 * @brief Callback function for checking flash busy flag
 *
 * User has to declare callback function for proper flash memory working.
 * For simplify user's work, you can use IsFlashBusy function,
 * which is checking WIP bit in status register
 *
 * @example
 *
 * BOOL busy_flag = FALSE;
 *
 * do
 * {
 *    busy_flag = IsFlashBusy();
 * }while(busy_flag);
 * @return
 */
void callback_function(ic_flash_state *m_flash_state);

/**
 * @brief Check is flash busy
 *
 * @return true if flash is busy
 */
bool IsFlashBusy(void);


/**
 * @}
 */
/** @defgroup HIGH_LEVEL_FLASH_FUNCTIONS_DECLARATION
 *  @ingroup HIGH_LEVEL_FLASH_API
 *  @{
 */
					/****************************************************************************************************************************************************************/
					/****************************************************************************************************************************************************************/
					/**************************************************                              ********************************************************************************/
					/**************************************************     FUNCTIONS PROTOTYPES     ********************************************************************************/
					/**************************************************                              ********************************************************************************/
					/****************************************************************************************************************************************************************/
					/****************************************************************************************************************************************************************/


					/***************************************************************************************************************************************************************/
					/*********************************************           	HIGHER LEVEL FUNCTIONS                   *************************************************************/
					/***************************************************************************************************************************************************************/

/**
 * @brief Driver Initialization
 *
 * @param flash_device_object
 *
 * Example:
 * @code
 * static ic_flash_FLASH_DEVICE_OBJECT flash_eon;
 * ReturnType ret;
 *
 * ret = Flash_Driver_Init(&flash_eon);
 *
 * if (ret == Flash_WrongType)
 * {
 *   NRF_LOG_INFO("No device detected!\r\n");
 *   printf("No device detected!\r\n");
 *
 *   return FALSE;
 * }
 * else
 * {
 * 	 NRF_LOG_INFO("Device detected properly!\r\n");
 *	 printf("Device detected properly!\r\n");
 * }
 * @return Flash_Success if everything was done okay
 */
FlashReturnType init_flash_driver(ic_flash_FLASH_DEVICE_OBJECT *flash_device_object);
/********************************************************************************************************************************/
/**
 * @brief Write data to flash in specific address
 *
 * @param address 	- address of flash you want to write to
 * @param data_to_write - buffer of data you want to store in flash
 * @param data_len	- length of the data you want to write
 * @param m_flash_state - pointer to flash's state structure
 * @param cb		- function's callback
 *
 * Example:
 * @code
 *
 * static ic_flash_state flash_state;
 *
 * uint8_t data_to_write[] = {
 *	"This text is for checking writing data to flash\r\n"
 *	"I hope everything works fine and you can see all of the message\r\n"
 *	"KG!!!"
 *	};
 *
 * write_to_flash(0x00, data_to_write, sizeof data_to_write, &flash_state, callback_function);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType write_to_flash(uint32_t address, uint8_t *data_to_write, size_t data_len, ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
/**
 * @brief Write data to flash in a specific way, giving block, sector and page address
 *
 * @param block_addr    - give the block address from datasheet (block address range 0 - 512)(page 7-14 eon)
 * @param sector_addr   - give the sector address from datasheet (sector addr range 0 - 15)
 * @param page_addr     - give the page address from datasheet (page addr range 0 - 15)
 * @param data_to_write - buffer with data to send via SPI interface (max bufer size 256 bytes)
 * @param len           - length of the data to send
 * @param m_flash_state - pointer to flash's state structure
 * @param cb		- function's callback
 *
 * Example:
 * @code
 *
 * static ic_flash_state flash_state;
 *
 * uint8_t data_to_write[] = {
 *	"This text is for checking writing data to flash\r\n"
 *	"I hope everything works fine and you can see all of the message\r\n"
 *	"KG!!!"
 *	};
 *
 * write_to_flash_specific(0x00, data_to_write, sizeof data_to_write, &flash_state, callback_function);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType write_to_flash_specific(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr, uint8_t *data_to_write, size_t len,
		ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
/**
 * @brief Erase sector from flash
 *
 * @param secotr_address - address of flash sector you want to erase
 * @param m_flash_state  - pointer to flash's state structure
 * @param cb		 - function's callback
 *
 * Example:
 * @code
 *
 * static ic_flash_state flash_state;
 *
 * erase_sector_from_flash(0x0F,&flash_state, callback_function);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType erase_sector_from_flash (uint32_t secotr_address,
		ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
/**
 * @brief Erase block from flash
 *
 * @param block_address - address of flash block you want to erase
 * @param cb		- function's callback
 *
 * Example:
 * @code
 *
 * static ic_flash_state flash_state;
 *
 * erase_block_from_flash(0x01,callback_function);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType erase_block_from_flash(uint16_t block_address, ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
FlashReturnType erase_chip(ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
/**
 * @brief Read from flash
 *
 * @param block_address		- address of block you want to read from
 * @param sector_address	- address of sector you want to erase
 * @param page_address		- address of page you want to erase
 * @param read_data		- stored data
 * @param len			- data length
 * @param m_flash_state 	- pointer to flash's state structure
 * @param cb			- function's callback
 *
 * Example:
 * @code
 *
 * static ic_flash_state flash_state;
 *
 * static volatile BOOL timer_busy_flag = FALSE;
 * uint8_t output_buffer[FLASH_BUFFER_SIZE_TO_SEND] = {0};
 *
 * read_from_flash(0xFA, output_buffer, sizeof output_buffer, &flash_state, callback_function);
 * printf("Output buffer: %s", output_buffer);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType read_from_flash(uint32_t start_address, uint8_t *data_output, size_t len,
		ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
/**
 * @brief Read from flash by giving a specific address
 *
 * @param block_address		- address of block you want to read from
 * @param sector_address	- address of sector you want to erase
 * @param page_address		- address of page you want to erase
 * @param read_data		- stored data
 * @param len			- data length
 * @param m_flash_state 	- pointer to flash's state structure
 * @param cb			- function's callback
 *
 * Example:
 * @code
 *
 * static ic_flash_state flash_state;
 *
 * uint8_t output_buffer[FLASH_BUFFER_SIZE_TO_SEND] = {0};
 *
 * read_from_flash_specific(0xFA, 0x0C, 0x0E, output_buffer, sizeof (output_buffer), &flash_state, callback_function);
 * printf("Output buffer: %s", output_buffer);
 *
 * @return Flash_Success if everything was done okay
 */
FlashReturnType read_from_flash_specific(uint16_t block_address, uint8_t sector_address, uint8_t page_address, uint8_t *read_data, size_t len,
		ic_flash_state *m_flash_state, func_finished cb);
/********************************************************************************************************************************/
/**
 * @brief Flash deinit
 *
 * @return
 */
FlashReturnType deinit_flash();
/********************************************************************************************************************************/
/**
 * @brief Convert data to vector for sending via spi
 *
 * @param data 			- data you will be converting
 * @param vec_data 		- your data vector
 * @param num_address_byte 	- length of converting data
 */
void convert_vector_addr(uint32_t data, uint8_t *vec_data, size_t num_address_byte);

/**
 *  @}
 */


	/**
	 * @}
	 */
#endif /* !IC_DRIVER_FLASH_H */
