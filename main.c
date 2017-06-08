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
#define OSTIMER_WAIT_FOR_QUEUE           2                                /**< Number of ticks to wait for the timer queue to be ready */

/*static TaskHandle_t m_logger_thread;*/
static TimerHandle_t m_dummy_timer;
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

APP_TIMER_DEF(m_app_timer_id);


static void dummy_timer(TimerHandle_t xTimer){
  UNUSED_VARIABLE(xTimer);
  /*static int val = 0;*/
  /*NRF_LOG_INFO("hi! . My val is: %d\n", ++val);*/
  nrf_gpio_pin_toggle(20);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
  // Initialize timer module.
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

  m_dummy_timer = xTimerCreate("DUMM", 1000, pdTRUE, NULL, dummy_timer);

  // Create timers.


  /* YOUR_JOB: Create any timers to be used by the application.
     Below is an example of how to create a timer.
     For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
     one.
     uint32_t err_code;
     err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
     APP_ERROR_CHECK(err_code); */

}



/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
  NRF_LOG_INFO("application_timers_start. m_app_timer_id: 0x%X\n", (int)m_app_timer_id);
  /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
     uint32_t err_code;
     err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
     APP_ERROR_CHECK(err_code); */
  if (pdPASS != xTimerStart(m_dummy_timer, OSTIMER_WAIT_FOR_QUEUE))
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
}


#if 0
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
  //[TODO] delete it
/*
 *    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
 *
 *    APP_ERROR_CHECK(err_code);
 *
 *    // Prepare wakeup buttons.
 *    err_code = bsp_btn_ble_sleep_mode_prepare();
 *    APP_ERROR_CHECK(err_code);
 *
 */
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint32_t err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
#endif


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
  /*vTaskResume(m_logger_thread);*/
  NRF_LOG_FLUSH();
}

void init_task (void *arg){
  UNUSED_PARAMETER(arg);
  timers_init();
  ble_module_init();
  application_timers_start();
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

    nrf_gpio_cfg_output(20);
    nrf_gpio_pin_clear(20);

    /*
     *if (pdPASS != xTaskCreate(logger_thread, "LOG", 128, NULL, 1, &m_logger_thread))
     *{
     *    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
     *}
     */

    if(pdPASS != xTaskCreate(init_task, "INIT", 256, NULL, 1, &m_init_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Start execution.
    /*NRF_LOG_INFO("Template started\r\n");*/

    /*SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;*/
    vTaskStartScheduler();
    // Enter main loop.
    for (;;)
    {
      APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
      power_manage();
    }
}

/**
 * @}
 */
