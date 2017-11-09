/**
 * @file    ic_service_ads.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    September, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "app_error.h"

#include "ic_config.h"

#define NRF_LOG_MODULE_NAME "ADS-SERVICE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_ble_service.h"
#include "ic_frame_handle.h"

#include "ic_service_ads.h"
#include "ic_driver_ads.h"

#include "ic_nrf_error.h"

static TimerHandle_t m_ads_service_timer_handle;

static int16_t m_eeg_measurement;
static TaskHandle_t send_data_task_handle;

ALLOCK_SEMAPHORE(m_twi_ready);

void read_callback(int16_t eeg){
  m_eeg_measurement = eeg;
  RESUME_TASK(send_data_task_handle);
}

static void on_stream_state_change(bool active){
  __auto_type _timer_ret_val = pdFAIL;
  if(active)  START_TIMER (m_ads_service_timer_handle, 0, _timer_ret_val);
  else{
    vTaskSuspend(send_data_task_handle);
    STOP_TIMER  (m_ads_service_timer_handle, 0, _timer_ret_val);
  }
  UNUSED_PARAMETER(_timer_ret_val);
}

static void ads_timer_callback(TimerHandle_t xTimer){
  UNUSED_PARAMETER(xTimer);

  static bool _force = false;

  __auto_type _semphr_successfull = pdTRUE;
  TAKE_SEMAPHORE(m_twi_ready, 1, _semphr_successfull);
  if(_semphr_successfull == pdFALSE){
    NRF_LOG_INFO("Could not take TWI transaction semaphore(forced)\n");
  }
  switch(ads_get_value(read_callback, _force)){
    case IC_ERROR:
      NRF_LOG_INFO("ads read error!\n");
      break;
    case IC_BUSY:
      NRF_LOG_INFO("ads read busy!(ADS)\n");
      /*_force = true;*/
      break;
    case IC_SOFTWARE_BUSY:
      NRF_LOG_INFO("ads read busy!(SOFT)\n");
      ads_get_value(read_callback, true);
      _force = true;
      break;
    case IC_DRIVER_BUSY:
      NRF_LOG_INFO("ads read busy!(DRIVER)\n");
      /*_force = true;*/
      break;
    default:
      _force = false;
      break;
  }
}

static void send_data_task(void *arg){
  uint8_t measurement_cnt = 0;
  u_eegDataFrameContainter eeg_packet;
  bool send_via_ble = false;

  uint32_t _nrf_error;

  for(;;){
    if(send_via_ble){
      __auto_type _err = ble_iccs_send_to_stream0(
          eeg_packet.raw_data,
          sizeof(u_eegDataFrameContainter),
          &_nrf_error);

      switch(_err){
        case IC_SUCCESS:
          send_via_ble = false;
          break;
        case IC_BLE_NOT_CONNECTED:
          send_via_ble = false;
          break;
        case IC_BUSY:
          continue; // TODO: Fix it. Can kill CPU.
        default:
          /*NRF_LOG_INFO("err: %s\n", (uint32_t)ic_get_nrferr2str(_nrf_error));*/
          break;
      }
    }
    else{
      if (measurement_cnt == 0)
        eeg_packet.frame.time_stamp = xTaskGetTickCount();

      eeg_packet.frame.eeg_data[measurement_cnt++] = m_eeg_measurement;
      GIVE_SEMAPHORE(m_twi_ready);

      if(measurement_cnt == sizeof(eeg_packet.frame.eeg_data)/sizeof(eeg_packet.frame.eeg_data[0])){
        send_via_ble = true;
        measurement_cnt = 0;
        continue;
      }
    }

    vTaskSuspend(NULL);
    taskYIELD();
  }
}

ic_return_val_e ic_ads_service_init(void){

  ic_ads_init();

  INIT_SEMAPHORE_BINARY(m_twi_ready);
  GIVE_SEMAPHORE(m_twi_ready);

  m_ads_service_timer_handle = xTimerCreate(
      "ADS_TIMER",
      8,
      pdTRUE,
      NULL,
      ads_timer_callback);

  /*xTimerStart(m_ads_service_timer_handle, 0);*/

  if(pdPASS != xTaskCreate(send_data_task, "BLET", 256, NULL, 3, &send_data_task_handle)){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  ble_iccs_connect_to_stream0(on_stream_state_change);

  vTaskSuspend(send_data_task_handle);

  return IC_SUCCESS;
}
