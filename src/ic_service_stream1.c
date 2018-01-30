/**
 * @file    ic_service_stream1.c
 * @author  Wojtek Węclewski <w.weclewski@inteliclinic.com>
 * @date    January, 2018
 * @brief   Brief description
 *
 * Description
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "app_error.h"

#define NRF_LOG_MODULE_NAME "STREAM1_SERVICE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_ble_service.h"
#include "ic_frame_handle.h"

#include "ic_service_stream1.h"
#include "ic_driver_acc.h"
#include "ic_driver_afe4400.h"

#include "ic_nrf_error.h"

#define NUM_OF_CONN_DEVS  2

static TimerHandle_t m_service_stream1_timer_handle = NULL;
static TaskHandle_t m_send_data_task_handle = NULL;

static bool m_module_initialized = false;

static uint8_t m_measurement_cnt = 0;
static volatile uint32_t m_stream1_timestamp;
static acc_data_s m_acc_measurement;
static ic_afe_val_s m_afe_measurement;

ALLOCK_SEMAPHORE(m_data_lock);

static void read_acc_callback(acc_data_s acc_measurement);
static void read_afe_callback(ic_afe_val_s afe_measurement);
static void on_stream1_state_change(bool active);

static void stream1_timer_callback(TimerHandle_t xTimer){
  __auto_type _semphr_successfull = pdTRUE;
  TAKE_SEMAPHORE(m_data_lock, 0, _semphr_successfull);
  if(_semphr_successfull == pdFALSE){
    NRF_LOG_ERROR("Stream1 TIMEOUT\n");
  }

  m_stream1_timestamp = GET_TICK_COUNT();

  switch(ic_acc_get_values(read_acc_callback, false)){
    case IC_ERROR:
      NRF_LOG_ERROR("ACC read error!\n");
      break;
    case IC_BUSY:
      NRF_LOG_INFO("ACC read busy!\n");
      ic_acc_get_values(read_acc_callback, true);
      break;
    default:
      break;
  }

  switch(ic_afe_get_values(read_afe_callback, false)){
    case IC_ERROR:
      NRF_LOG_ERROR("AFE read error!\n");
      break;
    case IC_BUSY:
      NRF_LOG_INFO("AFE read busy!\n");
      ic_afe_get_values(read_afe_callback, true);
      break;
    default:
      break;
  }
}

static void send_data_task(void *arg){
  u_otherDataFrameContainer m_stream1_packet;
  for(;;){
    m_stream1_packet.frame.time_stamp = m_stream1_timestamp; //GET_TICK_COUNT();
    GIVE_SEMAPHORE(m_data_lock);

    portENTER_CRITICAL();
    m_stream1_packet.frame.acc[0] = m_acc_measurement.x;
    m_stream1_packet.frame.acc[1] = m_acc_measurement.y;
    m_stream1_packet.frame.acc[2] = m_acc_measurement.z;
    m_stream1_packet.frame.ir_sample = m_afe_measurement.ir_diff;
    m_stream1_packet.frame.red_sample = m_afe_measurement.red_diff;
    portEXIT_CRITICAL();

    /*NRF_LOG_INFO("x: %d\ty: %d\tz: %d\n", m_stream1_packet.frame.acc[0], m_stream1_packet.frame.acc[1], m_stream1_packet.frame.acc[2]);*/

    __auto_type _ret_val = ble_iccs_send_to_stream1(
        m_stream1_packet.raw_data,
        sizeof(u_otherDataFrameContainer),
        NULL);

    switch(_ret_val){
      case IC_SUCCESS:
        break;
      case IC_BLE_NOT_CONNECTED:
        break;
      case IC_BUSY:
        vTaskDelay(1);
        continue; // TODO: Fix it. Can kill CPU.
      default:
        /*NRF_LOG_INFO("err: %s\n", (uint32_t)ic_get_nrferr2str(_nrf_error));*/
        break;
    }
    vTaskSuspend(NULL);
    taskYIELD();
  }
}

ic_return_val_e ic_service_stream1_init(void){
  if(m_module_initialized) return IC_SUCCESS;

  if(ic_acc_init() != IC_SUCCESS){
    NRF_LOG_ERROR("Couldn't initialize acc driver\r\n");
    return IC_ERROR;
  }

  if(ic_afe_init() != IC_SUCCESS){
    NRF_LOG_ERROR("Couldn't initialize afe driver\r\n");
    return IC_ERROR;
  }

  if(CHECK_INIT_SEMAPHORE(m_data_lock))
    INIT_SEMAPHORE_BINARY(m_data_lock);

  GIVE_SEMAPHORE(m_data_lock);

  if(m_service_stream1_timer_handle == NULL)
    m_service_stream1_timer_handle = xTimerCreate(
        "STREAM1_TIMER",
        IC_STREAM1_TICK_PERIOD,
        pdTRUE,
        (void *) 0,
        stream1_timer_callback);

  if(m_send_data_task_handle == NULL)
    if(pdPASS != xTaskCreate(send_data_task, "STREAM1_SENDER", 200, NULL, 3, &m_send_data_task_handle)){
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

  ble_iccs_connect_to_stream1(on_stream1_state_change);

  vTaskSuspend(m_send_data_task_handle);

  m_module_initialized = true;

  return IC_SUCCESS;
}

ic_return_val_e ic_service_stream1_deinit(void){
  if(m_module_initialized == false) return IC_NOT_INIALIZED;
  if (ic_acc_deinit() != IC_SUCCESS)
    return IC_ERROR;

  if (ic_afe_deinit() != IC_SUCCESS)
    return IC_ERROR;

  m_module_initialized = false;

  GIVE_SEMAPHORE(m_data_lock);

  __auto_type _ret_val = pdTRUE;
  STOP_TIMER(m_service_stream1_timer_handle, 0, _ret_val);
  UNUSED_VARIABLE(_ret_val);

  vTaskSuspend(m_send_data_task_handle);

  return IC_SUCCESS;
}

static void on_stream1_state_change(bool active){
  __auto_type _timer_ret_val = pdFAIL;
  NRF_LOG_INFO("{%s}%s\n", (uint32_t)__func__, (uint32_t)(active?"true":"false"));
  if(active){
    GIVE_SEMAPHORE(m_data_lock);
    START_TIMER (m_service_stream1_timer_handle, 0, _timer_ret_val);
  }
  else{
    GIVE_SEMAPHORE(m_data_lock);
    vTaskSuspend(m_send_data_task_handle);
    STOP_TIMER  (m_service_stream1_timer_handle, 0, _timer_ret_val);
    m_measurement_cnt = 0;
  }
  UNUSED_PARAMETER(_timer_ret_val);
}

static void read_acc_callback(acc_data_s acc_measurement){
  /*NRF_LOG_INFO("x: %d\ty: %d\tz: %d\n", acc_measurement.x , acc_measurement.y, acc_measurement.z);*/
  memcpy(&m_acc_measurement, &acc_measurement, sizeof(acc_data_s));
  if(++m_measurement_cnt >= NUM_OF_CONN_DEVS){
    RESUME_TASK(m_send_data_task_handle);
    m_measurement_cnt = 0;
  }
}

static void read_afe_callback(ic_afe_val_s afe_measurement){
  memcpy(&m_afe_measurement, &afe_measurement, sizeof(ic_afe_val_s));
  if(++m_measurement_cnt >= NUM_OF_CONN_DEVS){
    RESUME_TASK(m_send_data_task_handle);
    m_measurement_cnt = 0;
  }
}
