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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
/*#include "boards.h"*/
#include "softdevice_handler.h"
#include "app_timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*#include "bsp.h"*/
/*#include "bsp_btn_ble.h"*/
#include "nrf_gpio.h"
#include "nrf_drv_clock.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_bluetooth.h"
#include "ic_driver_uart.h"
#include "ic_driver_button.h"
#include "ic_driver_twi.h"

#include "ic_config.h"

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

static TaskHandle_t m_init_thread;
/*uint32_t m_app_ticks_per_100ms =0; [> EXTERN!!! <]*/


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}




/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
  __auto_type err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

/*
 *static void logger_thread(void * arg)
 *{
 *    UNUSED_PARAMETER(arg);
 *
 *    while(1)
 *    {
 *        NRF_LOG_FLUSH();
 *
 *        vTaskDelay(32000); // Suspend myself
 *    }
 *}
 */

void vApplicationIdleHook( void )
{
  NRF_LOG_FLUSH();
  power_manage();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  __auto_type _err = (error_info_t *)info;

  NRF_LOG_ERROR("{%s}[%d](%s)\r\n", (uint32_t)_err->p_file_name, _err->line_num,
      (uint32_t)ERR_TO_STR(_err->err_code));
  NRF_LOG_FINAL_FLUSH();
  // On assert, the system can only recover with a reset.
#ifndef DEBUG
  NVIC_SystemReset();
#else
  app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}

ALLOCK_SEMAPHORE(twi_test_semphr);

static void on_pwr_press(){
  GIVE_SEMAPHORE(twi_test_semphr, NULL);
}

void twi_test_task(void *arg){
  UNUSED_PARAMETER(arg);

  TWI_REGISTER(twi_test, ({
        printf("sent data\n\r");
            })
        );

  INIT_SEMAPHORE_BINARY(twi_test_semphr);
  TAKE_SEMAPHORE(twi_test_semphr, portMAX_DELAY, NULL);
  ic_btn_pwr_press_handle_init(on_pwr_press);
  for(;;){
    TAKE_SEMAPHORE(twi_test_semphr, portMAX_DELAY, NULL);
  }
}

void init_task (void *arg){
  UNUSED_PARAMETER(arg);
  ble_module_init();
  neuroon_exti_init();
  vTaskDelete(NULL);
  taskYIELD();
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    __auto_type err_code = NRF_LOG_INIT(xTaskGetTickCount);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    if(pdPASS != xTaskCreate(init_task, "INIT", 256, NULL, 4, &m_init_thread)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if(pdPASS != xTaskCreate(init_task, "INIT", 256, NULL, 4, &m_init_thread)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    ic_uart_init();

    /*SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;*/
    NRF_LOG_INFO("starting scheduler\n");
    vTaskStartScheduler();

    for (;;)
    {
      APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
      power_manage();
    }
}

/**
 * @}
 */
