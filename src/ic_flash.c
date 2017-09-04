	/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */


/*
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *                                                              1. Correct ic_driver_spi
 *
 *  It has to be able to send only specific instructions and address
 *  - SEND_INSTRUCTION()                              // for example SEND_INSTRUCTION(write_enable) - 1B
 *  - SEND (register_to_read_from)                    // for example SEND(read_from_status_register) -
 *                                                       1B instruction + 1B output data
 *  - SEND(INSTRUCTION+ADDR)                          // for example SEND_ADDRESS(my_address_to_write) -
 *                                                       2B (1B instruction register + 1B output data)
 *  - SEND (read_bytes_from_addr/write_bytes_to_addr) // max 256B
 *
 *  THE MOST IMPORTANT THING IS TO CHANGE SPI_SEND TO BE ABLE TO CONTROL #CS AND THEN
 *  YOU WILL BE ABLE TO REMOVE STRUCTURE TO WHICH DATA ARE COPIED AND SENDING BY PACKAGE
 *
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

 /*
  *       																											           FLASH_ALL
  *       																										     EON              MICRON
  *


																									/ ****************** MEMORY ORGANIZATION ************************/
/*
 * 																																		512 block of 64-Kbyte
 * 																																	8,192 sectors of 4-Kbyte
 * 																																131,072 pages (256 bytes each)
 */


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "app_timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "nrf_gpio.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_driver_uart.h"
#include "ic_driver_button.h"

#include "ic_config.h"

#include "ic_driver_spi.h"
#include "ic_driver_flash.h"

					/*		for FatFS module		*/
#include "nrf.h"
#include "bsp.h"
#if _USE_FATFS
	#include "ff.h"
#endif

#include  "nrf_drv_swi.h"




#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


#define FLASH_MAX_PAGE_PROGRAM_TIME   	5
#define FLASH_MAX_SECTOR_ERASE_TIME   	300
#define FLASH_MAX_BLOCK_ERASE_TIME    	2000
#define FLASH_MAX_CHIP_ERASE_TIME     	280000


#define FREERTOS_TIMER_INTERVAL					5000

#define TEST_SIGNATURE									"KG"
#define TEST_ADDR_SIGNATURE							0xFACE00

#define FF_MAX_SS												512


#define TASK_PRIORITY										3
#define MAIN_TASK_STACK_DEPTH						620
#define WAIT_TASK_STACK_DEPTH						128


#define FLASH_NRF_DEBUG


																								/*	FUNCTIONS DECLARATION	*/
void spi_flash_task_test(void);
void fat_example(void);
void spi_flash_task(void *arg);


								/**************************************************************************************************************************************/
								/**************************************************************************************************************************************/
								/******************************************						VARIABLES				*****************************************************************/
								/**************************************************************************************************************************************/
								/**************************************************************************************************************************************/

static volatile 	BOOL busy_flag = FALSE;
static volatile 	BOOL is_driver_init = FALSE;
static 						BOOL flash_test_done = FALSE;

ret_code_t err_code = 0;

	/*	Declare flash object  */
static ic_flash_FLASH_DEVICE_OBJECT flash_eon;
	/*	Declare pointer to allocated memory	*/
uint8_t *pvMy_memory;


	/*    declare all semaphores for handling   */
static volatile SemaphoreHandle_t wait_semaphore  = NULL;

	/*   variables for handling the tasks    */
//static TaskHandle_t m_init_thread 		 = NULL;
static TaskHandle_t m_wait_thread = NULL;

	/*	define timer handle	*/
static TimerHandle_t xTimer;

//static int task = 0;


	/*    Timer callback function     */
void vTimerCallback (TimerHandle_t xTimer)
{
	configASSERT(xTimer);
	printf("\r\n{%s}\r\n", __func__);
	busy_flag = IsFlashBusy();
	printf("BUSY FLAG: %d\r\n", busy_flag);
}

				/***************************************		ISR FUNCTIONS		***************************************************/
	/*	on press ISR function	*/
//static void on_press(void)
//{
//#ifdef FLASH_DEBUG
//  printf("\n\rButton pressed!\n\r");
//#endif
//#ifdef FLASH_NRF_DEBUG
//  NRF_LOG_INFO("\r\n Button pressed\r\n");
//#endif
//
//  /*	resume MAIN task when button is pressed	*/
//  xTaskResumeFromISR(m_init_thread);
//}
//
//static void on_release(void)
//{
//#ifdef FLASH_DEBUG
//  printf("\n\rButton released!\n\r");
//#endif
//}
//
//static void on_long_press(void)
//{
//  printf("\n\rButton long pressed!\n\r");
//}

						/**************************************************************************************************************************************/
						/**************************************************************************************************************************************/
						/******************************************						CALLBACKS				*****************************************************************/
						/**************************************************************************************************************************************/
						/**************************************************************************************************************************************/

	/*	callback for read flash function	*/
//typedef void (*flash_cb)(void);
//
//void test_cb(void)
//{
//	NRF_LOG_INFO("In test_cb\r\n");
//}
//
//void call_cb_func(flash_cb callback)
//{
//	NRF_LOG_INFO("\r\nIn call_cb_func\r\n");
//}

	/*	callback function for dealing with flash time operations	*/
BOOL callback_function()
{
	//printf("Function callback!\r\n");

	busy_flag = IsFlashBusy();
	if (busy_flag)
	{
		xSemaphoreGive(wait_semaphore);
		vTaskResume(m_wait_thread);
	}

	/***
	 *  if you do not use freeRTOS,
	 *  you can do polling like that
	 *  but, that way, you are blocking
	 *  processor
	 *
	 *
	 *	do
	 *	{
	 *		busy_flag = IsFlashBusy();
	 *	}	while(busy_flag);
	 *
	***/
	return busy_flag;
}

    																								/*    GPIO configuration function     */
void configure_gpio()
{
  	/*	set HOLD pin and PowerDistr pins as output (for proper behaving of flash)	*/
  nrf_gpio_cfg_output(18);
  nrf_gpio_cfg_output(15);
  nrf_gpio_cfg_output(16);
  	/*	set HIGH state to declared pins	*/
  nrf_gpio_pin_set(18);
  nrf_gpio_pin_set(15);
  nrf_gpio_pin_set(16);
}

<<<<<<< HEAD
=======
	for (;;)
	{
	//		printf("\r\n{%s}\r\n", __func__);
	//			/*	You must check the busy flag	*/
	//		if (!IsFlashBusy())
	//		{
	//			printf("Not busy anymore\r\n");
	//				/*	if not busy, suspend waiting task	*/
	//			vTaskSuspend(m_wait_thread);
	//			taskYIELD();
	//		}
	//		else
	//			printf("I have to wait...\r\n");

		switch(flash_state.state)
		{
			case write_state:
//				printf("Write operation\r\n");
				if(! IsFlashBusy())
				{
					flash_state.state = nop;
					flash_state.callback();
				}
				break;
			case erase_state:
//				printf("Erase operation\r\n");
				if(! IsFlashBusy())
				{
					flash_state.state = nop;
					flash_state.callback();
				}
				break;
			case read_state:
//				printf("Read operation\r\n");
				if(! IsFlashBusy())
				{
					flash_state.state = nop;
					flash_state.callback();
				}
				break;
			case nop:
//				printf("Not busy\r\n");
				taskYIELD();
//				break;
		}

	/*	Do whatever you want in here while
	* waiting for dealing with flash issues	*/

	}
}
/********************************************************************************************/
>>>>>>> d62fb84... Small changes
																										/*	Flash driver initialization function	*/
BOOL flash_driver_init(void)
{
  /*	Initialize your object - check what type of flash is connected and fill flash description structure	*/
  ReturnType ret;
  ret = init_flash_driver(&flash_eon, pvMy_memory);

  if (ret == Flash_WrongType)
    {
      NRF_LOG_INFO("No device detected!\r\n");
      printf("No device detected!\r\n");

      return FALSE;
    }

  return TRUE;
}


//								/*******************************  Variables for input and output buffer    *********************************************/
uint8_t output_buffer[FLASH_BUFFER_SIZE_TO_SEND] = {0};
//
//uint8_t data_to_write[] =
//    	 {"\r\n0123456789\r\n0123456789\r\n"
//    			 "0123456789\r\n0123456789\r\n"
//    		"\r\n0123456789\r\n0123456789\r\n"
//    			 "0123456789\r\n0123456789\r\n"
//    		"\r\n0123456789\r\n0123456789\r\n"
//    			 "0123456789\r\n0123456789\r\n"
//    		"Randomowe 20 znakow!"
//    		"Kolejne randomy..A teraz jeszcze wiecej\r\n"
//    			 	 "dalej i wypelniamy na maxa i patrzymy!!!!!KG"
//    			 };
//
uint8_t fatfs_data_to_write[] =
{
		"\r\n0123456789\r\n0123456789\r\n"
		    			 "0123456789\r\n0123456789\r\n"
		    		"\r\n0123456789\r\n0123456789\r\n"
		    			 "0123456789\r\n0123456789\r\n"
		    		"\r\n0123456789\r\n0123456789\r\n"
		    			 "0123456789\r\n0123456789\r\n"
		    		"Randomowe 20 znakow!"
		    		"Kolejne randomy..A teraz jeszcze wiecej\r\n"
		    			 	 "zmieniona tablica zeby zobaczyc czy PENIS!\r\n"
		"\r\n9999999999\r\n0123456789\r\n"
		    			 "0123456789\r\n0123456789\r\n"
		    		"\r\n0123456789\r\n0123456789\r\n"
		    			 "0123456789\r\n0123456789\r\n"
		    		"\r\n0123456789\r\n0123456789\r\n"
		    			 "0123456789\r\n0123456789\r\n"
		    		"Randomowe 20 znakow!"
		    		"Kolejne randomy..A teraz jeszcze wiecej\r\n"
		    			 	 "dalej i wypelniamy na maxa i patrzymy!!!PENIS"
};

//size_t data_to_write_size = sizeof(data_to_write);

	/******************************************************************************************************************************************************************/


																										/*	FATFS EXAMPLE TASK	*/
static int num=0;

void fat_example()
{

  printf("\r\n{%s}\r\n", __func__);

  FATFS fs;
//  DIR dir;
  FIL file;
  FILINFO fno;
  UINT bw;
  BYTE *work = (BYTE*) pvPortMalloc(FATFS_SECTOR_LENGTH);
  //BYTE work[1024];
  FRESULT res = 0;

	busy_flag = IsFlashBusy();

	//xSemaphoreTake(spi_test_semaphore2, 0) == pdTRUE &&
	if (busy_flag == FALSE && is_driver_init == TRUE)
	{
		if(num==0)
		{
			/*	format your flash memory to work as a fatfs disk	*/
			res = f_mkfs("", FM_ANY, 65536, work, FATFS_SECTOR_LENGTH);
			if (res == FR_OK)
				printf("F_MKFS OKAY\r\n");
			else
			{
				printf("F_MKFS failed\r\n");
			}

			/*	mount volume on your disk	*/
			printf("Mounting volume...\r\n");
			res = f_mount(&fs, "", 1);
			if (res == FR_OK)
				printf("Mount OKAY\r\n");
			else
				printf("Mount failed\r\n");
		}

		if (res == FR_OK && num == 0)
		{
			res = f_open (&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
			printf("RES: %d\r\n", res);
			if(!res)
			{
				printf("f_open correct\r\n");
				for (int i=0;i<2048;i++)
//					res = f_write(&file, fatfs_data_to_write, sizeof (fatfs_data_to_write), &bw);

				printf("RES: %d\r\n", res);
				res = f_close(&file);

				res = f_open (&file, "test.txt", FA_READ);
				if (!res)
				{
					for (int i = 0; i < 2048; i++)
					{
						res = f_read(&file, work, 512, &bw);
						printf("FATFS read sector %d: %s\r\n", 97 + i, work);
					}
				}
			}
			res = f_close(&file);
		}
		else
		{
			printf("Couldn't open file\r\n");
		}

		printf("Test for file...\n");

		res = f_stat("TEST.txt", &fno);
		switch (res)
		{
		case FR_OK:
				printf("Size: %lu\r\n", fno.fsize);
				printf("Timestamp: %u/%02u/%02u, %02u:%02u\r\n",
							 (fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
							 fno.ftime >> 11, fno.ftime >> 5 & 63);
				printf("Attributes: %c%c%c%c%c\r\n",
							 (fno.fattrib & AM_DIR) ? 'D' : '-',
							 (fno.fattrib & AM_RDO) ? 'R' : '-',
							 (fno.fattrib & AM_HID) ? 'H' : '-',
							 (fno.fattrib & AM_SYS) ? 'S' : '-',
							 (fno.fattrib & AM_ARC) ? 'A' : '-');
				break;

		case FR_NO_FILE:
				printf("It is not exist\r\n");
				break;

		default:
				printf("An error occured. (%d)\r\n", res);
		}
		num++;
	}	// MAIN IF END
}

		/**************************************       TESTING TASK FUNCTIONS      ******************************************************/


void spi_flash_task_test(void)
{
//  UNUSED_VARIABLE(arg);
  printf("\r\n{%s}\r\n", __func__);

  IsFlashBusy();
	if (is_driver_init == TRUE)
	{
		erase_sector_from_flash(0xFAC, callback_function);
		write_to_flash(TEST_ADDR_SIGNATURE, (uint8_t*)TEST_SIGNATURE, sizeof (TEST_SIGNATURE), callback_function);
		read_from_flash_specific(0xFA, 0x0C, 0x0E, output_buffer, sizeof (TEST_SIGNATURE), callback_function);

		printf("WHAT I'VE GOT: %s\r\n", output_buffer);

		for (int i = 0; i < sizeof (TEST_SIGNATURE); i++)
		{
			if (!(output_buffer[i] == TEST_SIGNATURE[i]))
			{
				printf("WRONG DATA WRITE AND READ\r\n");
				flash_test_done = FALSE;
				break;
			}
		}

		flash_test_done = TRUE;
//		erase_sector_from_flash(0x00, callback_function);
//		write_to_flash_specific(0x00,0x00,0x00, data_to_write, sizeof data_to_write, callback_function);
//		read_from_flash_specific(0x00, 0x00, 0x00, output_buffer, sizeof data_to_write, callback_function);

//		for (int i = 0; i < sizeof data_to_write; i++)
//		{
//			if (!(output_buffer[i] == data_to_write[i]))
//			{
//				printf("WRONG DATA WRITE AND READ\r\n");
//				flash_test_done = FALSE;
//				break;
//			}
//		}
		if (flash_test_done)
		{
			printf("Flash test is done. Everything is OK\r\n");
		}
	}
	else
	{
		printf("Couldn't do flash test because driver is not correct\r\n");
		flash_test_done = FALSE;
	}
}
/*************************************************       END OF TESTING TASK           *************************************************/

							//****************************************************************************************************//
							//*************************************      TASK FOR FLASH           ********************************//
							//****************************************************************************************************//

void spi_flash_task(void *arg){

	UNUSED_VARIABLE(arg);
  printf("\r\n{%s}\r\n", __func__);

  uint8_t output_buffer2[FATFS_SECTOR_LENGTH] = {0};

  IsFlashBusy();
	if((flash_test_done == TRUE))
	{
		printf("INSIDE\r\n");

		for (int i = 0; i < 8; i++)
			fatfs_write_sector(i, fatfs_data_to_write);

		for (int i= 0 ; i < 8; i++)
		{
			fatfs_read_sector(i, output_buffer2);
			printf("READING SECTOR %d: %s\r\n", i, output_buffer2);
		}
	} // END OF IF STATEMENT
} // END OF SPI_FLASH_TASK

																							/*********************	WAITING TASK	**********************/
void WAITING(void *arg)
{
	UNUSED_VARIABLE(arg);
  printf("\r\n{%s}\r\n", __func__);

	for (;;)
	{
		if(xSemaphoreTake(wait_semaphore, 0) == pdTRUE)
		{
			/*	You must check the busy flag	*/
			busy_flag = IsFlashBusy();
			if (!busy_flag)
			{
				printf("Not busy anymore\r\n");
				/*	if not busy, suspend waiting task	*/
		    vTaskSuspend(m_wait_thread);
		    taskYIELD();
			}
			printf("I have to wait...\r\n");
			/*	give semaphore back to repeat waiting task	*/
			xSemaphoreGive(wait_semaphore);

																						/*	Do whatever you want in here while
																						 * waiting for dealing with flash issues	*/


		}
    taskYIELD();
	}
}

												/**************************************   INIT TASK   ****************************************/

void flash_init_task ()
{
//  neuroon_exti_init();
//  ic_btn_pwr_press_handle_init(on_press);
//  ic_btn_pwr_release_handle_init(on_release);
//  ic_btn_pwr_long_press_handle_init(on_long_press);

  	/*	initialize flash driver	*/
  is_driver_init = flash_driver_init();
  	/*	create FreeRTOS timer	*/
  xTimer = xTimerCreate("Timer", FREERTOS_TIMER_INTERVAL, pdTRUE, (void *) 0, vTimerCallback);

  	/*	create and set freeRTOS timer	*/
  if (xTimer == NULL)
  {
    NRF_LOG_INFO("\r\nTimer not created\r\n");
  }
  else
	{
		// start the timer
		if ( xTimerStart(xTimer, 0) != pdPASS)
		{
			NRF_LOG_INFO("Couldn't start timer\r\n");
		}
	}
}

																		/********************		MAIN TASK		****************************/
//void MAIN_TASK(void *arg)
//{
//	UNUSED_VARIABLE(arg);
////  NRF_LOG_INFO("\r\n{%s}\r\n", (uint32_t)__func__);
//
//  size_t free;
//  	/*	check free heap size of freeRTOS	*/
//  free = xPortGetFreeHeapSize();
//  NRF_LOG_INFO("FREE SPACE: %d\r\n", free);
//
//  	/*	allocoate 4kB space for fatfs	functions	*/
//	pvMy_memory = (uint8_t *)pvPortMalloc(300);
//
////	NRF_LOG_INFO("MY ALLOC MEM: %p\r\n", (uint32_t)pvMy_memory);
//	if (pvMy_memory == NULL)
//		printf("Pointer ERROR\r\n");
//
//		/*	check again free heap size	*/
////  free = xPortGetFreeHeapSize();
////  NRF_LOG_INFO("FREE SPACE: %d\r\n", free);
//
//  																			/**********	TASK MAIN LOOP	***********/
//  for (;;)
//  {
//  	switch(task)
//  	{
//  	case 0:
//
//      flash_init_task();
//  		/*	enter in 4byte addressing mode, so you can use all of the flash memory	*/
////  		flash_eon.GenOp.FlashEnter4ByteMode(&flash_eon);
//  		/*	set waiting task's priority higher than MAIN for proper working	*/
//  		vTaskPrioritySet(m_spi_flash_thread, TASK_PRIORITY + 1);
//  		task++;
//
//  		break;
//
//  	case 1:
//  		spi_flash_task_test();
//  		task++;
//
//  		break;
//
//  	case 2:
//  		spi_flash_task(arg);
//  		task++;
//
//  		break;
//
//  	case 3:
//  		fat_example();
//
//  		break;
//
//  	default:
//  		printf("No task to do\r\n");
//  	}
//  	vTaskSuspend(m_init_thread);
//  	taskYIELD();
//  }
//}


			/******************************************************************************************************************************************/
			/******************************************************************************************************************************************/
			/****************************************************				MAIN				*******************************************************************/
			/******************************************************************************************************************************************/
			/******************************************************************************************************************************************/
/**@brief Function for application main entry.
 */
void ic_flash_init(void)
{
  	NRF_LOG_INFO("Starting flash initialization\r\n");

    /*	initialize uart	interface	*/
    ic_uart_init();
    printf("UART after initialization\r\n");

    /*  GPIO CONFIGURE   */
    configure_gpio();
    NRF_LOG_INFO("GPIO for SPI configured\r\n");

  	/*	create binary semaphore	*/
    wait_semaphore = xSemaphoreCreateBinary();
    /*	give the semaphore	*/
    xSemaphoreGive(wait_semaphore);
//    xSemaphoreTake(wait_semaphore, 0);

    /*	create two tasks
     *
     * MAIN_TASK - the main task in which you handle your functions
     *
     * WAITING - task for dealing with flash operations timing
     * 		You have to check whether flash is busy or not (polling),
     * 		and if so, you can do whatever you want to
     *
     * 		If flash is not busy anymore, the waiting task is suspended
     * 		and you return to last operation
     */
//    if(pdPASS != xTaskCreate(MAIN_TASK, "INIT", 256, NULL, TASK_PRIORITY, &m_init_thread))
//    {
//      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }

    if(pdPASS != xTaskCreate(WAITING, "WAIT", 128, NULL, TASK_PRIORITY | portPRIVILEGE_BIT, &m_wait_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    flash_init_task();
    taskYIELD();

}	// MAIN END

