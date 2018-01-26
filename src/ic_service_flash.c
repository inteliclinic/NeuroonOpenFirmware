/**
 * @file    ic_flash.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Flash service file
 *
 * Description
 */

 /*
  *								        FLASH_ALL
  *								  EON              MICRON
  *			  		      / ****************** MEMORY ORGANIZATION ********************/
/*								512 block of 64-Kbyte
 *								8,192 sectors of 4-Kbyte
 *							     131,072 pages (256 bytes each)
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
#define NRF_LOG_MODULE_NAME 		 "SERVICE_FLASH"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_driver_uart.h"
#include "ic_driver_button.h"

#include "ic_config.h"

#include "ic_driver_spi.h"
#include "ic_service_flash.h"
#include "ic_driver_flash.h"
#include "ic_driver_flash_micron.h"
#include "ic_flash_filesystem.h"
#include "ic_flash_address_map.h"

#include  "nrf_drv_swi.h"

#define UART_DEBUG
#undef UART_DEBUG

#define TASK_PRIORITY            3
#define WAIT_TASK_STACK_DEPTH    128

#define FLASH_NRF_DEBUG


ic_return_val_e m_is_driver_init;

  /*	Declare flash object  */
static ic_flash_FLASH_DEVICE_OBJECT m_flash_object;

  /*   variables for handling the tasks    */
static TaskHandle_t m_wait_thread = NULL;

/*******************************************************************/
void flash_service_cb(ic_flash_state *flash_state);

static ic_flash_state m_flash_state =
{
		.callback = flash_service_cb,
		.state = nop
};
/*******************************************************************/
	/*	callback function for dealing with flash time operations	*/
void flash_service_cb(ic_flash_state *flash_state)
{
  NRF_LOG_INFO("{ %s } from %d\r\n", (uint32_t)__func__, flash_state->state);
    /*	flash ended doing operation, going back to nop state  */
  m_flash_state.state = nop;
}
/*******************************************************  IC_FILE_SYSTEM  **********************************************************************/
uint32_t temp_file_addr = FLASH_STARTING_ADDRESS;

static s_mbr_info m_mbr_info =
{
  .neuroon_sig = 0x1234,
  .erasal_num = 0x00,
  .num_of_files = 0,
  .source_info = {},
  .not_used = {}
};
/*******************************************************************/
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
/*******************************************************************/
void deconfigure_gpio()
{
  nrf_gpio_pin_clear(18);
  nrf_gpio_pin_clear(15);
  nrf_gpio_pin_clear(16);
  
  nrf_gpio_cfg_default(18);
  nrf_gpio_cfg_default(15);
  nrf_gpio_cfg_default(16);
}
/*********************	WAITING TASK	**********************/
void flash_waiting(void *arg)
{
  UNUSED_VARIABLE(arg);
  NRF_LOG_INFO("\r\n{%s}\r\n", (uint32_t)__func__);

  for (;;)
  {
    switch(m_flash_state.state)
    {
      case write_state:
        /*NRF_LOG_INFO("Write operation\r\n");*/
        if(! IsFlashBusy())
        {
          if(m_flash_state.callback != NULL)
            m_flash_state.callback(&m_flash_state);
          else
            m_flash_state.state = nop;
        }
        break;
      case erase_state:
        /*NRF_LOG_INFO("Erase operation\r\n");*/
        if(! IsFlashBusy())
        {
          if(m_flash_state.callback != NULL)
            m_flash_state.callback(&m_flash_state);
          else
            m_flash_state.state = nop;
        }
        break;
      case read_state:
        /*NRF_LOG_INFO("Read operation\r\n");*/
        if(! IsFlashBusy())
        {
          if(m_flash_state.callback != NULL)
            m_flash_state.callback(&m_flash_state);
          else
            m_flash_state.state = nop;
        }
        break;
      case nop:
        /*NRF_LOG_INFO("Not busy\r\n");*/
        vTaskDelay(1);
        break;
      default:
        NRF_LOG_INFO("Error in %s\r\n", (uint32_t)__func__);
    }

	/*	Do whatever you want in here while
	* waiting for dealing with flash issues	*/

  }
}
/********************************************************************************************/
	/*	Flash driver initialization function	*/
ic_return_val_e ic_flash_driver_init(void)
{
    /*	Initialize your object - check what type of flash is connected and fill flash description structure	*/
  __auto_type _ret_val = init_flash_driver(&m_flash_object);

  if (_ret_val != Flash_Init_Success)
  {
    NRF_LOG_INFO("No device detected!\r\n");

    return IC_ERROR;
  }
  m_flash_object.GenOp.FlashSoftReset();

    /*	enter 4 byte addressing mode	*/
  _ret_val = m_flash_object.GenOp.FlashWriteEnable(&m_flash_object);
  if (_ret_val == Flash_Success)
    _ret_val = m_flash_object.GenOp.FlashEnter4ByteMode(&m_flash_object);

  return IC_SUCCESS;
}
/*******************************************************************/
	/*  Variables for input and output buffer    */
uint8_t output_buffer[FLASH_BUFFER_SIZE_MAX] = {0};

uint8_t data_to_write[] =
   {    "\r\n0123456789\r\n0123456789\r\n"
    		"0123456789\r\n0123456789\r\n"
    		"\r\n0123456789\r\n0123456789\r\n"
    		"0123456789\r\n0123456789\r\n"
    		"\r\n0123456789\r\n0123456789\r\n"
    		"0123456789\r\n0123456789\r\n"
    		"Randomowe 20 znakow!"
    		"Kolejne randomy..A teraz jeszcze wiecej\r\n"
    		"dalej i wypelniamy na maxa i patrzymy!!!!!KG"
	 };
/******************************************************************************************************************************************************************/
ic_return_val_e ic_flash_write(uint32_t address, uint8_t *data_to_write, size_t data_len, func_finished cb)
{
  if (write_to_flash(address, data_to_write, data_len, &m_flash_state, cb) != Flash_Success)
    return IC_ERROR;
  if(cb == NULL)
  {
    while(IsFlashBusy() == true);
    m_flash_state.state = nop;
  }
  else
    while(m_flash_state.state != nop){ vTaskDelay(1); };
  return IC_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_return_val_e ic_flash_sector_erase(uint32_t address, func_finished cb)
{
  ++(m_mbr_info.erasal_num);
  if (erase_sector_from_flash(address, &m_flash_state, cb) != Flash_Success)
  {
    --(m_mbr_info.erasal_num);
    return IC_ERROR;
  }
  if(cb == NULL)
  {
    while(IsFlashBusy() == true);
    m_flash_state.state = nop;
  }
  else
    while(m_flash_state.state != nop){ vTaskDelay(1); };
  return IC_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_return_val_e ic_flash_read(uint32_t address, uint8_t *read_data, size_t len, func_finished cb)
{
  if (read_from_flash(address, read_data, len, &m_flash_state, cb) != Flash_Success)
    return IC_ERROR;
  if(cb == NULL)
  {
    while(IsFlashBusy() == true);
    m_flash_state.state = nop;
  }
  else
    while(m_flash_state.state != nop){ vTaskDelay(1); };
  return IC_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_return_val_e ic_flash_read_specific(uint16_t block_address, uint8_t sector_address, uint8_t page_address, uint8_t *read_data, size_t len,
		func_finished cb)
{
  if (read_from_flash_specific(block_address, sector_address, page_address, read_data, len, &m_flash_state, cb) != Flash_Success)
    return IC_ERROR;
  if(cb == NULL)
  {
    while(IsFlashBusy()== true);
    m_flash_state.state = nop;
  }
  else
    while(m_flash_state.state != nop){ vTaskDelay(1); };
  return IC_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_return_val_e ic_flash_erase_chip(func_finished cb)
{
  if (erase_chip(&m_flash_state, cb) != Flash_Success)
    return IC_ERROR;
  if(cb == NULL)
  {
    while(IsFlashBusy() == true);
    m_flash_state.state = nop;
  }
  else
    while(m_flash_state.state != nop){ vTaskDelay(1); };
  return IC_SUCCESS;
}
/******************************************************************************************************************************************************************/
		/**************************************       TESTING TASK FUNCTIONS      ******************************************************/
#ifdef _FLASH_FILESYSTEM_TEST

ic_return_val_e flash_test_write(void)
{
  /*NRF_LOG_INFO("\r\n{%s}\r\n", (uint32_t)__func__);*/
  if (m_is_driver_init == IC_SUCCESS)
  {
    bool _test_flag = true;
//    ic_flash_erase_chip(callback_function);
    ic_flash_sector_erase(0x01, flash_service_cb);
    while(m_flash_state.state != nop) {vTaskDelay(1);}
      /*  format mbr sector  */
    if (ic_mbr_format(&m_mbr_info, true) == IC_FILE_SUCCESS)
    {
        /*  open stream with filesystem  */
      if (ic_open_source(&m_mbr_info) != IC_FILE_SUCCESS)
        _test_flag = false;
  #ifdef UART_DEBUG
      printf("SIG VAL: 0x%lX\r\n", m_mbr_info.neuroon_sig);
      printf("ERASAL NUM: %lu\r\n", m_mbr_info.erasal_num);
      printf("FILES NUM: %d\r\n", m_mbr_info.num_of_files);
      for (int i = 0; i < m_mbr_info.num_of_files; i++)
      {
        printf("[%d]NAME: %s\r\n", i, m_mbr_info.source_info[i].source_name);
        printf("[%d]START ADDR: %lu\r\n", i, m_mbr_info.source_info[i].start_addr);
        printf("[%d]END ADDR: %lu\r\n", i, m_mbr_info.source_info[i].end_addr);
        printf("[%d]ATTRIBUTE: %d\r\n", i, m_mbr_info.source_info[i].source_flag);
      }
  #endif
        /*  create new file  */
      if (ic_create_file(&m_mbr_info, "LEL.TXT", 7) == IC_FILE_CREATED)
      {
          /*  open file with IC_FILE_W flag for writing  */
        if (ic_open_file(&m_mbr_info, "LEL.TXT", IC_FILE_W) == IC_FILE_SUCCESS)
        {
          /*  write some testing chars to your file  */
        for (int i = 0; i < 51; i++)
          ic_write_file(&m_mbr_info, "LEL.TXT", data_to_write);
          /*  close file  */
        ic_close_file(&m_mbr_info, "LEL.TXT");
        }
      }
      else
        _test_flag = false;

        /*  create second file  */
      if (ic_create_file(&m_mbr_info, "LEL2.TXT", 8) == IC_FILE_CREATED)
      {
        if (ic_open_file(&m_mbr_info, "LEL2.TXT", IC_FILE_W) == IC_FILE_SUCCESS)
        {
          ic_write_file(&m_mbr_info, "LEL2.TXT", data_to_write);
          ic_write_file(&m_mbr_info, "LEL2.TXT", data_to_write);
          ic_close_file(&m_mbr_info, "LEL2.TXT");
        }
      }
      else
        _test_flag = false;

      NRF_LOG_INFO("{ %s done!}\r\n", (uint32_t)__func__);
    }
    return (_test_flag == true) ? IC_SUCCESS : IC_ERROR;
	}
  else
  {
    NRF_LOG_INFO("Flash module not initialized\r\n");
    return IC_ERROR;
  }
}
/********************************************************************************/
									/************************************      TASK FOR FLASH           ********************************/
void flash_test_read()
{
  /*NRF_LOG_INFO("\r\n{%s}\r\n", (uint32_t)__func__);*/

  s_mbr_info _test;

  ic_open_source(&_test);
#ifdef UART_DEBUG
  printf("SIG VAL: 0x%lX\r\n", _test.neuroon_sig);
  printf("ERASAL NUM: %lu\r\n", _test.erasal_num);
  printf("FILES NUM: %d\r\n", _test.num_of_files);
  for (int i = 0; i < _test.num_of_files; i++)
  {
    printf("[%d]NAME: %s\r\n", i, _test.source_info[i].source_name);
    printf("[%d]START ADDR: %lu\r\n", i, _test.source_info[i].start_addr);
    printf("[%d]END ADDR: %lu\r\n", i, _test.source_info[i].end_addr);
    printf("[%d]ATTRIBUTE: %d\r\n", i, _test.source_info[i].source_flag);
    printf("[%d]LENGTH: %lu\r\n", i, _test.source_info[i].end_addr - _test.source_info[i].start_addr);
  }
#endif


  uint8_t *_file_data = (uint8_t *)pvPortMalloc(512);
  if (_file_data != NULL)
  {
    ic_read_file(&_test, "LEL2.TXT", _file_data);
#ifdef UART_DEBUG
  printf("CHECK DATA: %s\r\n", file_data);
#endif
    vPortFree(_file_data);
  }
} // END OF SPI_FLASH_TASK

#endif
/*************************************************************************************************************************/
/************************************************* FLASH TESTING *********************************************************/
ic_return_val_e ic_flash_test(void)
{
  ic_flash_sector_erase(0xFAC, flash_service_cb);
  ic_flash_write(0xFACE00, (uint8_t*)"TEST_FLASH_NEUROON_OPEN", 23, flash_service_cb);

  uint8_t *_file_data = (uint8_t *)pvPortMalloc(23);
  if (_file_data != NULL)
  {
    ic_flash_read(0xFACE00, _file_data, 23, flash_service_cb);
    NRF_LOG_INFO("{%s}: %s\r\n", (uint32_t)__func__, (uint32_t)_file_data);
  }
  vPortFree(_file_data);

  if ((strcmp((const char*)_file_data, (const char*)"TEST_FLASH_NEUROON_OPEN")) != 0)
    return IC_ERROR;
  else
    return IC_SUCCESS;
}
/*************************************************************************************************************************/
ic_return_val_e ic_flash_init(void)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/

    /*  GPIO CONFIGURE   */
  configure_gpio();
    /*	initialize flash driver	*/
  m_is_driver_init = ic_flash_driver_init();

    /*
     *
     * flash_waiting - task for dealing with flash operations timing
     * 		You have to check whether flash is busy or not (polling),
     * 		and if so, you can do whatever you want to
     *
     * 		If flash is not busy anymore, the waiting task is suspended
     * 		and you return to last operation
     */
  if(m_wait_thread == NULL)
    if(xTaskCreate(flash_waiting, "WAIT", WAIT_TASK_STACK_DEPTH, NULL, TASK_PRIORITY, &m_wait_thread) != pdPASS)
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

  return IC_SUCCESS;
}	// MAIN END
/*******************************************************************************************************************************/
ic_return_val_e ic_flash_deinit(void)
{
    /*  deinit spi used by flash  */
  deinit_flash();
    /*  deconfigurate gpio used by flash module  */
  deconfigure_gpio();

  if (m_wait_thread != NULL)
    vTaskSuspend(m_wait_thread);

  /*NRF_LOG_INFO("Flash device deinitialized\r\n");*/

  return IC_SUCCESS;
}
/********************************************************************************************************************************/
/********************************************************************************************************************************/
