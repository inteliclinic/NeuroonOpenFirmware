/**
 * @file    ic_service_time.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_service_time.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


static volatile ic_timestamp_s m_timestamp = {
  .unix_timestamp = 1514764800ul,
  .sub_timer = 0ul};

static TimerHandle_t m_timer_handle;

static inline void increment_unix_timestamp(void){
  m_timestamp.unix_timestamp++;
}

static void timer_callback(TimerHandle_t xTimer){
  increment_unix_timestamp();
  NRF_LOG_INFO("unix ts: %d,%d", m_timestamp.unix_timestamp, m_timestamp.sub_timer);
}

void ic_unix_timestamp_set(uint64_t timestamp){
  m_timestamp.unix_timestamp = timestamp;
}

ic_timestamp_s ic_unix_timestamp_get(){
  m_timestamp.sub_timer = xTaskGetTickCount()&0x3FF;
  return m_timestamp;
}

ic_return_val_e ic_service_timestamp_init(){
  m_timer_handle = xTimerCreate("Service Timer", 1024, pdTrue, (void *)0, timer_callback);

  __auto_type _ret_val = pdFAIL;
  START_TIMER(m_timer_handle, 0, _ret_val);
  return IC_SUCCESS;
}

