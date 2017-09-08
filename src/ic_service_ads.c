/**
 * @file    ic_service_ads.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    September, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_service_ads.h"
#include "ic_driver_ads.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define NRF_LOG_MODULE_NAME "ADS-SERVICE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/*
 *static TaskHandle_t m_ads_service_task_handle;
 */
static TimerHandle_t m_ads_service_timer_handle;

/*ALLOCK_SEMAPHORE(ads_service_lock);*/

/*
 *static void ads_service_main_task(void *args){
 *  for(;;){
 *
 *  }
 *}
 */

void read_callback(int16_t eeg){
  NRF_LOG_INFO("eeg: %d\n", eeg);
}

static void ads_timer_callback(TimerHandle_t xTimer){
  UNUSED_PARAMETER(xTimer);

  bool _force = false;

ENTER_SWITCH: // TODO: Very... VERY... dirty hack.

  switch(ads_get_value(read_callback, _force)){
    case IC_ERROR:
      NRF_LOG_INFO("ads read error!\n");
      break;
    case IC_BUSY:
      NRF_LOG_INFO("ads read busy!\n");
      _force = true;
      goto ENTER_SWITCH;
      break;
    default:
      break;
  }
}

ic_return_val_e ic_ads_service_init(void){

  ic_ads_init();

  /*
   *if(pdPASS != xTaskCreate(
   *      ads_service_main_task,
   *      "ADS_SERVICE",
   *      256,
   *      NULL,
   *      4,
   *      &m_ads_service_task_handle
   *      ) //xTaskCreate
   *  ) //if
   *{
   *  APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
   *}
   */

  m_ads_service_timer_handle = xTimerCreate(
      "ADS_TIMER",
      8,
      pdTRUE,
      NULL,
      ads_timer_callback);

  /*
   *INIT_SEMAPHORE_BINARY(ads_service_lock);
   *TAKE_SEMAPHORE(ads_service_lock, 0);
   */

  xTimerStart(m_ads_service_timer_handle, 3);

  return IC_SUCCESS;
}
