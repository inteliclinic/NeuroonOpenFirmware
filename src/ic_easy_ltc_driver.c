/**
 * @file    ic_easy_ltc_driver.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_easy_ltc_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ic_config.h"

#include "ic_driver_twi.h"

#define NRF_LOG_MODULE_NAME "LTC"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static TaskHandle_t m_ez_ltc_task_handle;
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
  .ltc_state = EZ_LTC_GLOWING,
  .current_val = 63,
  .val_going_up = false
};

TWI_REGISTER(ez_ltc_twi, 0x1C);

static void ez_ltc_twi_finished(ic_return_val_e ret_val){
  UNUSED_PARAMETER(ret_val);
  GIVE_SEMAPHORE(ez_ltc_lock);
}

uint8_t m_in_buffer[] = {16, 63};

static void send_value_to_LTC(void){
  TAKE_SEMAPHORE(ez_ltc_lock, portMAX_DELAY);

  __auto_type _ret_val =
    TWI_SEND_DATA(
        ez_ltc_twi,
        m_in_buffer,
        sizeof(m_in_buffer),
        ez_ltc_twi_finished);

  if(_ret_val != IC_SUCCESS){
      TWI_SEND_DATA_FORCED(
        ez_ltc_twi,
        m_in_buffer,
        sizeof(m_in_buffer),
        ez_ltc_twi_finished);
  }

}

void ez_ltc_main_task(void *args){
  UNUSED_PARAMETER(args);
  INIT_SEMAPHORE_BINARY(ez_ltc_lock);
  TAKE_SEMAPHORE(ez_ltc_lock, 0);

  TWI_SEND_DATA(ez_ltc_twi, m_in_buffer, sizeof(m_in_buffer), ez_ltc_twi_finished);

  for(;;){
    __NOP();
    /*NRF_LOG_INFO("{%s}\n\r", (uint32_t)__func__);*/
    switch(m_ltc_state.ltc_state){
      case EZ_LTC_GLOWING:
        if(m_ltc_state.val_going_up){
          if(m_ltc_state.current_val != 63){
            m_in_buffer[1] = ++m_ltc_state.current_val;
            send_value_to_LTC();
          }
          else{
            m_ltc_state.val_going_up = false;
            send_value_to_LTC();
          }
        }
        else{
          if(m_ltc_state.current_val != 0){
            m_in_buffer[1] = --m_ltc_state.current_val;
            send_value_to_LTC();
          }
          else{
            m_ltc_state.val_going_up = true;
            send_value_to_LTC();
          }
        }
        vTaskDelay(5);
        continue;
      case EZ_LTC_OFF:
        if (m_ltc_state.current_val != 0){
          m_in_buffer[1] = --m_ltc_state.current_val;
          send_value_to_LTC();
          vTaskDelay(5);
          continue;
        }
        else {
          break;
        }
      case EZ_LTC_ON:
        if (m_ltc_state.current_val != 63){
          m_in_buffer[1] = ++m_ltc_state.current_val;
          send_value_to_LTC();
          vTaskDelay(5);
          continue;
        }
        else {
          break;
        }
    }
    vTaskSuspend(NULL);
    taskYIELD();
  }
}
void ic_ez_ltc_module_init(void){
  TWI_INIT(ez_ltc_twi);

  if(pdPASS != xTaskCreate(ez_ltc_main_task, "EZLTC", 512, NULL, 3, &m_ez_ltc_task_handle)){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

void ic_ez_ltc_glow(){
  m_ltc_state.ltc_state = EZ_LTC_GLOWING;
  vTaskResume(m_ez_ltc_task_handle);
}

void ic_ez_ltc_fade(){
  m_ltc_state.ltc_state = EZ_LTC_OFF;
  vTaskResume(m_ez_ltc_task_handle);
}

void ic_ez_ltc_brighten(){
  m_ltc_state.ltc_state = EZ_LTC_ON;
  vTaskResume(m_ez_ltc_task_handle);
}
