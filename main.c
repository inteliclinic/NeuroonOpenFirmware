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
#include "ic_command_task.h"

#include "ic_service_ads.h"

#include "ic_acc_service.h"
#include "ic_afe_service.h"

#include "ic_config.h"
#include "ic_easy_ltc_driver.h"

#include "ic_driver_ltc.h"
#include "ic_driver_actuators.h"
#include "ic_service_ltc.h"

#include "ic_ble_service.h"

#include "ic_service_time.h"

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

static TaskHandle_t m_init_task;
static TaskHandle_t m_cleanup_task;
static void (*m_welcome)(void);

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
  /*__auto_type err_code = sd_app_evt_wait();*/
  __SEV();
  __WFE();
  __WFE();
  /*APP_ERROR_CHECK(err_code);*/
}

void vApplicationIdleHook( void )
{
  NRF_LOG_FLUSH();
  power_manage();
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

static void power_down_all_systems(void){
  nrf_gpio_cfg_output(15);
  nrf_gpio_cfg_output(16);
  nrf_gpio_cfg_output(IC_LTC_POWER_PIN);
  nrf_gpio_pin_clear(15);
  nrf_gpio_pin_clear(16);
  nrf_gpio_pin_clear(IC_LTC_POWER_PIN);
  /*nrf_gpio_cfg_default(15);*/
  /*nrf_gpio_cfg_default(16);*/
  /*nrf_gpio_cfg_default(IC_LTC_POWER_PIN);*/
  nrf_gpio_cfg_default(IC_SPI_FLASH_SS_PIN);
  nrf_gpio_cfg_default(IC_SPI_MISO_PIN);
  nrf_gpio_cfg_default(IC_SPI_MOSI_PIN);
  nrf_gpio_cfg_default(IC_SPI_SCK_PIN);
  nrf_gpio_cfg_default(IC_UART_RX_PIN);
  nrf_gpio_cfg_default(IC_UART_TX_PIN);
  nrf_gpio_cfg_default(IC_SPI_AFE_SS_PIN);
  nrf_gpio_cfg_default(IC_SPI_AFE_RESET_PIN);
  nrf_gpio_cfg_default(IC_SPI_AFE_PDN_PIN);
}

static void power_up_all_systems(void){
  nrf_gpio_cfg_output(15);
  nrf_gpio_pin_set(15);
  nrf_gpio_cfg_output(16);
  nrf_gpio_pin_set(16);
  nrf_gpio_cfg_output(IC_LTC_POWER_PIN);
  nrf_gpio_pin_set(IC_LTC_POWER_PIN);
}

/*ALLOCK_SEMAPHORE(m_fade_lock);*/

/*static void m_fade_callback(void){*/
  /*NRF_LOG_INFO("{%s}\n", (uint32_t)__func__);*/
  /*GIVE_SEMAPHORE(m_fade_lock);*/
/*}*/

/*static void m_reinit(void){*/
  /*vTaskResume(m_init_task);*/
  /*ic_bluetooth_enable();*/
/*}*/


/*
 *static void m_deinit(void){
 *  if (CHECK_INIT_SEMAPHORE(m_fade_lock))
 *    INIT_SEMAPHORE_BINARY(m_fade_lock);
 *
 *  __auto_type _ret_val = pdFAIL;
 *
 *  TAKE_SEMAPHORE(m_fade_lock, 0, _ret_val);
 *  ic_btn_pwr_long_press_handle_init(m_reinit);
 *  ic_ads_service_deinit();
 *  ic_bluetooth_disable();
 *
 *
 *  ic_actuator_init();
 *
 *  ic_actuator_set(LEFT_GREEN_LED, 63, NULL);
 *
 *  TAKE_SEMAPHORE(m_fade_lock, pdMS_TO_TICKS(4000), _ret_val);
 *  GIVE_SEMAPHORE(m_fade_lock);
 *  ic_ez_ltc_module_deinit();
 *  power_down_all_systems();
 *  [>NVIC_DisableIRQ(RTC1_IRQn);<]
 *  [>NRF_POWER->SYSTEMOFF = 1;<]
 *}
 *
 */


  /*UNUSED_PARAMETER(arg);*/
  /*int8_t val = 0;*/
  /*for(;;){*/

    /*ic_actuator_set(ACTUATOR_LEFT_RED_LED, val&0x3F, NULL);*/
    /*ic_actuator_set(ACTUATOR_LEFT_GREEN_LED, val&0x3F, NULL);*/
    /*ic_actuator_set(ACTUATOR_LEFT_BLUE_LED, (val&0x3F)>>3, NULL);*/

    /*ic_actuator_set(ACTUATOR_RIGHT_RED_LED, val&0x3F, NULL);*/
    /*ic_actuator_set(ACTUATOR_RIGHT_GREEN_LED, val&0x3F, NULL);*/
    /*ic_actuator_set(ACTUATOR_RIGHT_BLUE_LED, (val&0x3F)>>3, NULL);*/

    /*val++;*/
    /*vTaskDelay(16);*/
  /*}*/
/*}*/

#define WELCOME_PERIOD pdMS_TO_TICKS(500)
#define PERIOD pdMS_TO_TICKS(2000)

static void on_connect(void){
  ic_actuator_set_off_func(IC_LEFT_BLUE_LED, 0, 0, 0);
  ic_actuator_set_off_func(IC_RIGHT_BLUE_LED, 0, 0, 0);
  ic_actuator_set_off_func(IC_LEFT_RED_LED, 0, 0, 0);
  ic_actuator_set_off_func(IC_RIGHT_RED_LED, 0, 0, 0);
  ic_actuator_set_off_func(IC_LEFT_GREEN_LED, 0, 0, 0);
  ic_actuator_set_off_func(IC_RIGHT_GREEN_LED, 0, 0, 0);
}

static void on_disconnect(void){
  ic_actuator_set_off_func(IC_LEFT_RED_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_RIGHT_RED_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_GREEN_LED, 0, 0, 30);
  ic_actuator_set_off_func(IC_RIGHT_GREEN_LED, 0, 0, 30);
  ic_actuator_set_off_func(IC_VIBRATOR, 0, 0, 0);
  ic_actuator_set_off_func(IC_POWER_LEDS, 0, 0, 0);

  ic_actuator_set_triangle_func(IC_LEFT_BLUE_LED, PERIOD, 0, 63);
  ic_actuator_set_triangle_func(IC_RIGHT_BLUE_LED, PERIOD, 0, 63);
}

 void welcome(void){
  ic_actuator_set_triangle_func(IC_LEFT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_LEFT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_LEFT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_RIGHT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_RIGHT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_RIGHT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
}

void showoff(void){
  ic_actuator_set_triangle_func(IC_POWER_LEDS, WELCOME_PERIOD>>2, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_LEFT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_LEFT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_LEFT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_RIGHT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_RIGHT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_RIGHT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));
  ic_actuator_set_triangle_func(IC_VIBRATOR, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD));

  power_down_all_systems();

  NRF_POWER->SYSTEMOFF = 1;
}

void bye_bye(void){
  ic_actuator_set_off_func(IC_LEFT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  ic_actuator_set_off_func(IC_LEFT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  ic_actuator_set_off_func(IC_LEFT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  ic_actuator_set_off_func(IC_RIGHT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  ic_actuator_set_off_func(IC_RIGHT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  ic_actuator_set_off_func(IC_RIGHT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);

  ic_actuator_set_triangle_func(IC_LEFT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_LEFT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_LEFT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_RIGHT_RED_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_RIGHT_GREEN_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
  ic_actuator_set_triangle_func(IC_RIGHT_BLUE_LED, WELCOME_PERIOD, WELCOME_PERIOD, 63);
  vTaskDelay(pdMS_TO_TICKS(WELCOME_PERIOD>>2));
}

static void m_deep_sleep(void){
  NRF_LOG_INFO("{%s}\n", (uint32_t)__func__);
  RESUME_TASK(m_cleanup_task);
}

static void cleanup_task (void *arg){
  NRF_LOG_INFO("{%s}\n", (uint32_t)__func__);

  sd_power_reset_reason_clr(NRF_POWER->RESETREAS);

  ic_bluetooth_disable();
  ic_ads_service_deinit();
  ic_acc_deinit();
  ic_afe_deinit();

  bye_bye();


  vTaskDelay(1024);


  power_down_all_systems();

  NRF_POWER->SYSTEMOFF = 1;
}

/*********************/
/* AFE&ACC module */
#if 1
TaskHandle_t m_stream1_handle = NULL;

static volatile bool m_send_to_stream1 = false;

static void on_stream1_state_change(bool active){
  NRF_LOG_INFO("{%s}\n",(uint32_t)__func__);
  m_send_to_stream1 = active;
}

static u_otherDataFrameContainer m_stream1_output_frame;

static volatile bool m_acc_measured = false;

static void m_acc_measured_callback(acc_data_s data){

  m_stream1_output_frame.frame.time_stamp = GET_TICK_COUNT();
  m_stream1_output_frame.frame.acc[0] = data.x;
  m_stream1_output_frame.frame.acc[1] = data.y;
  m_stream1_output_frame.frame.acc[2] = data.z;

  m_acc_measured = true;

  /*NRF_LOG_INFO("x: %d, y: %d, z: %d\n",data.x, data.y, data.z)*/

}

static void m_afe_measured_callback(s_led_val data){

  m_stream1_output_frame.frame.ir_sample = data.diff_led1;
  m_stream1_output_frame.frame.red_sample = data.diff_led2;

  if(m_acc_measured){
    m_acc_measured = !m_acc_measured;
    if(m_send_to_stream1){
      RESUME_TASK(m_stream1_handle);
    }
  }
}

void stream1_task(void *arg){
  for(;;){
    __auto_type _ret_val = ble_iccs_send_to_stream1(
        m_stream1_output_frame.raw_data,
        sizeof(u_otherDataFrameContainer),
        NULL);

    switch(_ret_val)
    {
      case IC_SUCCESS:
        break;
      default:
        NRF_LOG_INFO("Stream1 error: %s\n", (uint32_t)g_return_val_string[_ret_val]);
        continue;
    }
    vTaskSuspend(NULL);
  }
}

void init_acc_afe(void){
  ble_iccs_connect_to_stream1(on_stream1_state_change);

  if(pdPASS != xTaskCreate(stream1_task, "STR1", 128, NULL, 2, &m_stream1_handle)){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  vTaskSuspend(m_stream1_handle);

  ic_acc_module_init(m_acc_measured_callback);
  ic_afe_init(m_afe_measured_callback);
}

TaskHandle_t m_stream2_handle = NULL;
static volatile bool m_send_to_stream2 = false;
uint32_t g_twi_err_cnt = 0;

static void on_stream2_state_change(bool active){
  NRF_LOG_INFO("{%s}\n",(uint32_t)__func__);
  active ? ({RESUME_TASK(m_stream2_handle);}) : vTaskSuspend(m_stream2_handle);
  m_send_to_stream2 = active;
}

void stream2_task(void *arg){
  static char _buffer[20];
  for(;;){
    snprintf(_buffer, sizeof(_buffer), "%ld: %ld", GET_TICK_COUNT(), g_twi_err_cnt);
    if(m_send_to_stream2) ble_iccs_send_to_stream2((uint8_t *)_buffer, strlen(_buffer)+1, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void init_err_stream(void){
  ble_iccs_connect_to_stream2(on_stream2_state_change);

  if(pdPASS != xTaskCreate(stream2_task, "STR1", 128, NULL, 2, &m_stream2_handle)){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  vTaskSuspend(m_stream2_handle);
}

#endif
/*********************/

static void init_task (void *arg){
  UNUSED_PARAMETER(arg);
  power_up_all_systems();

  ic_neuroon_exti_init();
  if(m_welcome != showoff)
    vTaskDelay(pdMS_TO_TICKS(1500));

  if(!ic_button_pressed(IC_BUTTON_PWR_BUTTON_PIN) && m_welcome == welcome){
    power_down_all_systems();
    NRF_POWER->SYSTEMOFF = 1;
  }

  ic_ltc_service_init();

  m_welcome();
  ic_neuroon_exti_init();
  ic_btn_pwr_long_press_handle_init(m_deep_sleep);

  ic_ads_service_init();
  init_acc_afe();
  init_err_stream();
  ic_ble_module_init();
  sd_power_reset_reason_clr(NRF_POWER->RESETREAS);
  ic_service_timestamp_init();
  cmd_module_init();
  on_disconnect();
  vTaskDelete(NULL);
  taskYIELD();
}

void main_on_ble_evt(ble_evt_t * p_ble_evt){
  switch(p_ble_evt->header.evt_id){
    case BLE_GAP_EVT_CONNECTED:
      on_connect();
      break;
    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnect();
      break;
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName){
  NRF_LOG_INFO("Stack overflowed: %s\n\r", (uint32_t)pcTaskName);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    __auto_type err_code = NRF_LOG_INIT(GET_TICK_COUNT);
    APP_ERROR_CHECK(err_code);
    power_down_all_systems();

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);


    if(pdPASS != xTaskCreate(init_task, "INIT", 384, NULL, 4, &m_init_task)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if(pdPASS != xTaskCreate(cleanup_task, "INIT", 192, NULL, 4, &m_cleanup_task)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    vTaskSuspend(m_cleanup_task);


    NRF_LOG_INFO("Reset reason: %d; Ret val: %d\n", NRF_POWER->RESETREAS, sd_power_reset_reason_clr(0xFFFFFFFF));

    m_welcome = NRF_POWER->RESETREAS & (0x01<<16) ? welcome : showoff;

    NRF_LOG_INFO("GPREGRET: %d\n", NRF_POWER->GPREGRET);


    /*SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;*/
    NRF_LOG_INFO("starting scheduler\n");
    vTaskStartScheduler();

    for (;;)
    {
      /*APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);*/
      power_manage();
    }
}

/**
 * @}
 */
