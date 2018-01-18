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

static TimerHandle_t m_ads_service_timer_handle = NULL;
static bool m_module_initialized = false;

static volatile uint8_t m_measurement_cnt = 0;
u_eegDataFrameContainter m_eeg_packet;

static TaskHandle_t send_data_task_handle = NULL;

ALLOCK_SEMAPHORE(m_twi_ready);

static inline bool add_eeg(int16_t eeg){
  if(m_measurement_cnt == 0)
    m_eeg_packet.frame.time_stamp = GET_TICK_COUNT();

  m_eeg_packet.frame.eeg_data[m_measurement_cnt++] = eeg;
  if(m_measurement_cnt ==
      sizeof(m_eeg_packet.frame.eeg_data)/sizeof(m_eeg_packet.frame.eeg_data[0]))
  {
    m_measurement_cnt = 0;
    return true;
  }
  return false;
}

void read_callback(int16_t eeg){
  GIVE_SEMAPHORE(m_twi_ready);
  if(add_eeg(eeg))
    RESUME_TASK(send_data_task_handle);
}

static void on_stream_state_change(bool active){
  __auto_type _timer_ret_val = pdFAIL;
  if(active)  START_TIMER (m_ads_service_timer_handle, 0, _timer_ret_val);
  else{
    vTaskSuspend(send_data_task_handle);
    STOP_TIMER  (m_ads_service_timer_handle, 0, _timer_ret_val);
    m_measurement_cnt = 0;
  }
  UNUSED_PARAMETER(_timer_ret_val);
}

static void ads_timer_callback(TimerHandle_t xTimer){
  UNUSED_PARAMETER(xTimer);

  __auto_type _semphr_successfull = pdTRUE;
  TAKE_SEMAPHORE(m_twi_ready, 0, _semphr_successfull);
  if(_semphr_successfull == pdFALSE){
    NRF_LOG_INFO("Could not take TWI transaction semaphore\n");
  }
  switch(ads_get_value(read_callback, false)){
    case IC_ERROR:
      NRF_LOG_INFO("ads read error!\n");
      break;
    case IC_BUSY:
      NRF_LOG_INFO("ads read busy!(ADS)\n");
      ads_get_value(read_callback, true);
      break;
    case IC_SOFTWARE_BUSY:
      NRF_LOG_INFO("ads read busy!(SOFT)\n");
      ads_get_value(read_callback, true);
      break;
    case IC_DRIVER_BUSY:
      NRF_LOG_INFO("ads read busy!(DRIVER)\n");
      /*_force = true;*/
      break;
    default:
      break;
  }
}

static void send_data_task(void *arg){
  uint32_t _nrf_error;
  for(;;){
    __auto_type _err = ble_iccs_send_to_stream0(
        m_eeg_packet.raw_data,
        sizeof(u_eegDataFrameContainter),
        &_nrf_error);

    __auto_type _timestamp = m_eeg_packet.frame.time_stamp;

    switch(_err){
      case IC_SUCCESS:
        for(int i = 0; i<8; ++i){
          NRF_LOG_RAW_INFO("%d\t\t%d\n", _timestamp, m_eeg_packet.frame.eeg_data[i]);
          _timestamp += 8;
        }
        break;
      case IC_BLE_NOT_CONNECTED:
        break;
      case IC_BUSY:
        NRF_LOG_INFO("Ależ dupa!\n");
        continue; // TODO: Fix it. Can kill CPU.
      default:
        /*NRF_LOG_INFO("err: %s\n", (uint32_t)ic_get_nrferr2str(_nrf_error));*/
        break;
    }
    vTaskSuspend(NULL);
    taskYIELD();
  }
}

ic_return_val_e ic_ads_service_init(void){
  if(m_module_initialized) return IC_SUCCESS;

  while(ic_ads_init() == IC_ERROR);

  if(CHECK_INIT_SEMAPHORE(m_twi_ready))
    INIT_SEMAPHORE_BINARY(m_twi_ready);

  GIVE_SEMAPHORE(m_twi_ready);

  if(m_ads_service_timer_handle == NULL)
    m_ads_service_timer_handle = xTimerCreate(
        "ADS_TIMER",
        IC_ADS_TICK_PERIOD,
        pdTRUE,
        NULL,
        ads_timer_callback);

  if(send_data_task_handle == NULL)
    if(pdPASS != xTaskCreate(send_data_task, "BLET", 256, NULL, 3, &send_data_task_handle)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

  ble_iccs_connect_to_stream0(on_stream_state_change);

  vTaskSuspend(send_data_task_handle);

  m_module_initialized = true;
  return IC_SUCCESS;
}

ic_return_val_e ic_ads_service_deinit(void){
  ic_ads_deinit();

  m_module_initialized = false;

  GIVE_SEMAPHORE(m_twi_ready);

  __auto_type _ret_val = pdTRUE;
  STOP_TIMER(m_ads_service_timer_handle, 0, _ret_val);
  UNUSED_VARIABLE(_ret_val);

  vTaskSuspend(send_data_task_handle);

  return IC_SUCCESS;
}
