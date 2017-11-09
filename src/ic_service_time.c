/**
 * @file    ic_service_time.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "ic_service_time.h"

#define NRF_LOG_MODULE_NAME "TIM"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"

#define DATE_01_01_2018 1514764800ul
#define DATE_ZERO       0ul

#define DEFAULT_DATE DATE_ZERO

static volatile ic_unix_timestamp_s m_unix_timestamp = {
  .unix_timestamp = DEFAULT_DATE,
  .sub_timer = 0ul};

struct {
  uint32_t seconds : 22;
  uint32_t fraction: 10;
} m_ralative_timestamp = {
  .seconds  = 0,
  .fraction = 0
};

static TimerHandle_t m_timer_handle;

static inline void increment_timestamp(void){
  m_unix_timestamp.sub_timer = xTaskGetTickCount()&0x3FF;
  m_unix_timestamp.unix_timestamp++;
  m_ralative_timestamp.fraction = xTaskGetTickCount()&0x3FF;
  m_ralative_timestamp.seconds++;
}

static void timer_callback(TimerHandle_t xTimer){
  UNUSED_PARAMETER(xTimer);
  increment_timestamp();
}

void ic_relative_timestamp_set(uint32_t timestamp){

}

void ic_unix_timestamp_set(uint64_t timestamp){
  m_unix_timestamp.unix_timestamp = timestamp;
}

ic_unix_timestamp_s ic_unix_timestamp_get(){
  m_unix_timestamp.sub_timer = xTaskGetTickCount()&0x3FF;
  return m_unix_timestamp;
}

ic_return_val_e ic_service_timestamp_init(){
  __auto_type _tics = pdMS_TO_TICKS(1000);
  m_timer_handle = xTimerCreate("Service Timer", _tics, pdTRUE, (void *)0, timer_callback);

  __auto_type _ret_val = pdFAIL;
  START_TIMER(m_timer_handle, 0, _ret_val);

  if(_ret_val != pdPASS){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  return IC_SUCCESS;
}

