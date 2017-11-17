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

uint32_t m_relative_fraction_mem = 0;
uint32_t m_unix_sub_timer_mem = 0;

static TimerHandle_t m_timer_handle = NULL;
static bool m_module_initialized = true;

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
  m_relative_fraction_mem = xTaskGetTickCount()&0x3FF;
  m_ralative_timestamp.seconds = timestamp;
}

ic_unix_timestamp_s ic_relative_timestamp_get(){
  m_ralative_timestamp.fraction = (xTaskGetTickCount()-m_relative_fraction_mem)&0x3FF;
  return m_unix_timestamp;
}

void ic_unix_timestamp_set(uint64_t timestamp){
  m_unix_sub_timer_mem = xTaskGetTickCount()&0x3FF;
  m_unix_timestamp.unix_timestamp = timestamp;
}

ic_unix_timestamp_s ic_unix_timestamp_get(){
  m_unix_timestamp.sub_timer = (xTaskGetTickCount() - m_unix_sub_timer_mem)&0x3FF;
  return m_unix_timestamp;
}

ic_return_val_e ic_service_timestamp_init(){
  if(m_module_initialized == true) return IC_SUCCESS;

  __auto_type _tics = pdMS_TO_TICKS(1000);

  if(m_timer_handle == NULL)
    m_timer_handle = xTimerCreate("Service Timer", _tics, pdTRUE, (void *)0, timer_callback);

  m_relative_fraction_mem = xTaskGetTickCount()&0x3FF;
  m_unix_sub_timer_mem = xTaskGetTickCount()&0x3FF;
  __auto_type _ret_val = pdFAIL;
  START_TIMER(m_timer_handle, 0, _ret_val);

  if(_ret_val != pdPASS){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  m_module_initialized = true;
  return IC_SUCCESS;
}

ic_return_val_e ic_service_timestamp_deinit(){
  if(m_timer_handle == NULL)
    return IC_NOT_INIALIZED;

  __auto_type _ret_val = pdFAIL;
  STOP_TIMER(m_timer_handle, 0, _ret_val);
  UNUSED_VARIABLE(_ret_val);

  m_module_initialized = false;

  return IC_SUCCESS;
}

ic_return_val_e ic_service_timestamp_destroy(){
  if(m_timer_handle == NULL)
    return IC_NOT_INIALIZED;

  xTimerDelete(m_timer_handle, 0);

  m_timer_handle = NULL;

  return IC_SUCCESS;
}
