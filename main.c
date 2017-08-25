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
#include "app_scheduler.h"
#include "app_timer_appsh.h"

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

#include "ic_config.h"

#include "ic_driver_twi.h"

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

#define SCHED_MAX_EVENT_DATA_SIZE      MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                           BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE               20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */

TWI_REGISTER(m_twi_handle);
APP_TIMER_DEF(m_ltc_timer);

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
/*
 *static void power_manage(void)
 *{
 *  __auto_type err_code = sd_app_evt_wait();
 *  APP_ERROR_CHECK(err_code);
 *}
 */

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
  /*power_manage();*/
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
  __auto_type _err = (error_info_t *)info;

  NRF_LOG_ERROR("{%s}[%d]\r\n", (uint32_t)_err->p_file_name, _err->line_num);
  NRF_LOG_FINAL_FLUSH();
  // On assert, the system can only recover with a reset.
#ifndef DEBUG
  NVIC_SystemReset();
#else
  app_error_save_and_stop(id, pc, info);
#endif // DEBUG
}

void init_task (void *arg){
  UNUSED_PARAMETER(arg);
  ble_module_init();
  neuroon_exti_init();
}

/*static void twi_finished_cb(void *p_context){*/
  /*UNUSED_PARAMETER(p_context);*/
/*}*/

static void ltc_timer_to(void *p_context){
  static uint8_t _send_buf[2] = {0x1, 30};
  static int8_t _inc = 1;
  UNUSED_PARAMETER(p_context);

  TWI_SEND_DATA(m_twi_handle, 0x1C, _send_buf, sizeof(_send_buf), NULL);
  _send_buf[1] += _inc;
  if (_send_buf[1]>62 || _send_buf[1]<1){
    _inc *= -1;
  }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    __auto_type err_code = NRF_LOG_INIT(app_timer_cnt_get);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    nrf_gpio_cfg_output(15);
    nrf_gpio_pin_set(15);
    nrf_gpio_cfg_output(16);
    nrf_gpio_pin_set(16);
    nrf_gpio_cfg_output(25);
    nrf_gpio_pin_set(25);

    #ifdef DEBUG
    ic_uart_init();
    #endif

    /*SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;*/
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
    NRF_LOG_INFO("starting scheduler\n");
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    neuroon_exti_init();
    ble_module_init();
    TWI_INIT(m_twi_handle);
    app_timer_create(&m_ltc_timer, APP_TIMER_MODE_REPEATED, ltc_timer_to);
    app_timer_start(m_ltc_timer, APP_TIMER_TICKS(4, 0), NULL);

    for (;;)
    {
      /*APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);*/
      app_sched_execute();
      /*power_manage();*/
      NRF_LOG_FLUSH();
    }
}

/**
 * @}
 */
