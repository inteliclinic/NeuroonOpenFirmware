/**
 * @file    ic_easy_ltc_driver.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ic_easy_ltc_driver.h"

#include "ic_driver_twi.h"

#define NRF_LOG_MODULE_NAME "LTC"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define LTC_RESOLUTION    20

static bool m_module_initialized = false;
static TaskHandle_t m_ez_ltc_task_handle = NULL;

static void (*m_finished_callback)(void) = NULL;

ALLOCK_SEMAPHORE(ez_ltc_lock);

static struct{
  enum{
    EZ_LTC_GLOWING,
    EZ_LTC_OFF,
    EZ_LTC_ON
  }ltc_state;
  uint8_t current_val;
  bool val_going_up;
}m_ltc_state = {
  .ltc_state = EZ_LTC_OFF,
  .current_val = 63,
  .val_going_up = false
};

TWI_REGISTER(ez_ltc_twi, 0x38);

static void ez_ltc_twi_finished(ic_return_val_e ret_val, void *p_context){
  UNUSED_PARAMETER(ret_val);
  UNUSED_PARAMETER(p_context);
  GIVE_SEMAPHORE(ez_ltc_lock);
}

enum LED_COLOR{
  RED_COLOR,
  GREEN_COLOR,
  BLUE_COLOR
};

#define CHANGE_TO_RED   m_in_buffer[0] = 2
#define CHANGE_TO_GREEN m_in_buffer[0] = 1
#define CHANGE_TO_BLUE  m_in_buffer[0] = 3

static uint8_t m_in_buffer[] = {2, 63};

static void leds_off(void){
  m_in_buffer[1] = 0;

  CHANGE_TO_RED;
  TWI_SEND_DATA(
      ez_ltc_twi,
      m_in_buffer,
      sizeof(m_in_buffer),
      NULL,
      NULL);

  CHANGE_TO_GREEN;
  TWI_SEND_DATA(
      ez_ltc_twi,
      m_in_buffer,
      sizeof(m_in_buffer),
      NULL,
      NULL);
  CHANGE_TO_BLUE;
  TWI_SEND_DATA(
      ez_ltc_twi,
      m_in_buffer,
      sizeof(m_in_buffer),
      NULL,
      NULL);
}

static void send_value_to_LTC(void){
  __auto_type _sem_ret_val = pdTRUE;
  __auto_type _twi_ret_val = IC_SUCCESS;

  TAKE_SEMAPHORE(ez_ltc_lock, LTC_RESOLUTION, _sem_ret_val);

  if(_sem_ret_val == pdTRUE){
    _twi_ret_val =
      TWI_SEND_DATA(
          ez_ltc_twi,
          m_in_buffer,
          sizeof(m_in_buffer),
          ez_ltc_twi_finished,
          NULL);
  }

  if(_twi_ret_val != IC_SUCCESS || _sem_ret_val != pdTRUE){
      TWI_SEND_DATA_FORCED(
        ez_ltc_twi,
        m_in_buffer,
        sizeof(m_in_buffer),
        ez_ltc_twi_finished,
        NULL);
  }
}

void ez_ltc_main_task(void *args){
  UNUSED_PARAMETER(args);
  uint32_t _dummy;
  INIT_SEMAPHORE_BINARY(ez_ltc_lock);
  TAKE_SEMAPHORE(ez_ltc_lock, 0, _dummy);
  UNUSED_PARAMETER(_dummy);

  TWI_SEND_DATA(
      ez_ltc_twi,
      m_in_buffer,
      sizeof(m_in_buffer),
      ez_ltc_twi_finished,
      NULL);

  for(;;){
    __NOP();
    switch(m_ltc_state.ltc_state){
      case EZ_LTC_GLOWING:
        if(m_ltc_state.val_going_up){
          if(m_ltc_state.current_val < 63){
            m_in_buffer[1] = ++m_ltc_state.current_val;
            send_value_to_LTC();
          }
          else{
            m_ltc_state.val_going_up = false;
            send_value_to_LTC();
          }
        }
        else{
          if(m_ltc_state.current_val > 1){
            m_in_buffer[1] = --m_ltc_state.current_val;
            send_value_to_LTC();
          }
          else{
            m_ltc_state.val_going_up = true;
            send_value_to_LTC();
          }
        }
        vTaskDelay(LTC_RESOLUTION);
        continue;
      case EZ_LTC_OFF:
        if (m_ltc_state.current_val != 0){
          m_in_buffer[1] = --m_ltc_state.current_val;
          send_value_to_LTC();
          vTaskDelay(LTC_RESOLUTION);
          continue;
        }
        else {
          break;
        }
      case EZ_LTC_ON:
        if (m_ltc_state.current_val != 63){
          m_in_buffer[1] = ++m_ltc_state.current_val;
          send_value_to_LTC();
          vTaskDelay(LTC_RESOLUTION);
          continue;
        }
        else {
          break;
        }
    }

    if(m_finished_callback != NULL){
      m_finished_callback();
      m_finished_callback = NULL;
    }

    vTaskSuspend(NULL);
  }
}

ic_return_val_e ic_ez_ltc_module_init(void){
  if(m_module_initialized) return IC_SUCCESS;
  TWI_INIT(ez_ltc_twi);
  CHANGE_TO_RED;

  if(m_ez_ltc_task_handle == NULL)
    if(pdPASS != xTaskCreate(ez_ltc_main_task, "EZLTC", 256, NULL, IC_FREERTOS_TASK_PRIORITY_LOW,
          &m_ez_ltc_task_handle)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

  m_module_initialized = true;
  return IC_SUCCESS;
}

void ic_ez_ltc_module_deinit(void){
  leds_off();
  TWI_DEINIT(ez_ltc_twi);

  m_module_initialized = false;

  /*vTaskSuspend(m_ez_ltc_task_handle);*/
  GIVE_SEMAPHORE(ez_ltc_lock);
}

void ic_ez_ltc_glow(){
  m_ltc_state.ltc_state = EZ_LTC_GLOWING;
  RESUME_TASK(m_ez_ltc_task_handle);
}

void ic_ez_ltc_fade(void(*fp)(void)){
  m_finished_callback = fp;
  m_ltc_state.ltc_state = EZ_LTC_OFF;
  RESUME_TASK(m_ez_ltc_task_handle);
}

void ic_ez_ltc_brighten(void(*fp)(void)){
  m_finished_callback = fp;
  m_ltc_state.ltc_state = EZ_LTC_ON;
  RESUME_TASK(m_ez_ltc_task_handle);
}

static void color_change(ic_return_val_e ret_val, void *color){
  switch((enum LED_COLOR)color){
    case RED_COLOR:
      CHANGE_TO_RED;
      break;
    case GREEN_COLOR:
      NRF_LOG_INFO("Changed to green\n");
      CHANGE_TO_GREEN;
      break;
    case BLUE_COLOR:
      NRF_LOG_INFO("Changed to blue\n");
      CHANGE_TO_BLUE;
      break;
  }
  m_ltc_state.current_val = 0;
  ic_ez_ltc_brighten(NULL);
}

static void on_connect(void){
  m_in_buffer[1] = 0;
  uint32_t _ret;
  TAKE_SEMAPHORE(ez_ltc_lock, LTC_RESOLUTION, _ret);
  NRF_LOG_INFO("%s:%d\n",(uint32_t)__func__, _ret);

  __auto_type _ret_val = TWI_SEND_DATA(
      ez_ltc_twi,
      m_in_buffer,
      sizeof(m_in_buffer),
      color_change,
      (void *)BLUE_COLOR);
  NRF_LOG_INFO("%s:%d\n", (uint32_t)__func__, _ret_val);
}

static void on_disconnect(void){
  m_in_buffer[1] = 0;
  uint32_t _ret;
  TAKE_SEMAPHORE(ez_ltc_lock, LTC_RESOLUTION, _ret);
  NRF_LOG_INFO("%s:%d\n",(uint32_t)__func__, _ret);

  __auto_type _ret_val = TWI_SEND_DATA(
      ez_ltc_twi,
      m_in_buffer,
      sizeof(m_in_buffer),
      color_change,
      (void *)RED_COLOR);
  NRF_LOG_INFO("%s:%d\n", (uint32_t)__func__, _ret_val);
}

void ic_ez_ltc_on_ble_evt(ble_evt_t * p_ble_evt){
  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      on_connect();
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnect();
      break;

    case BLE_GATTS_EVT_WRITE:
      /*on_write(p_ble_evt);*/
      break;

    default:
      // No implementation needed.
      break;
  }
}


