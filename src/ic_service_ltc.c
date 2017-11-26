/**
 * @file    ic_service_ltc.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    November, 2017
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
#include "ic_service_ltc.h"
#include "ic_driver_actuators.h"

#include "ic_command_task.h"
#include "ic_frame_handle.h"
#include "ic_low_level_control.h"

#define NRF_LOG_MODULE_NAME "LTC-SERVICE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define QUANTUM_OF_TIME 16

static TimerHandle_t m_ltc_refresh_timer_handle = NULL;

typedef enum __attribute__((packed)){
  DEV_RIGHT_RED_LED   = 0x01,
  DEV_RIGHT_GREEN_LED = 0x02,
  DEV_RIGHT_BLUE_LED  = 0x04,
  DEV_LEFT_RED_LED    = 0x08,
  DEV_LEFT_GREEN_LED  = 0x10,
  DEV_LEFT_BLUE_LED   = 0x20,
  DEV_VIBRATOR        = 0x40,
  DEV_POWER_LED       = 0x80
}e_deviceType;

static void m_device_parse(u_BLECmdPayload payload){
  if(payload.device_cmd.device&DEV_RIGHT_RED_LED){
    payload.device_cmd.func_type.FUN_TYPE_SQUARE
  }
  if(payload.device_cmd.device&DEV_RIGHT_GREEN_LED){
  }
  if(payload.device_cmd.device&DEV_RIGHT_BLUE_LED){
  }
  if(payload.device_cmd.device&DEV_LEFT_RED_LED){
  }
  if(payload.device_cmd.device&DEV_LEFT_GREEN_LED){
  }
  if(payload.device_cmd.device&DEV_LEFT_BLUE_LED){
  }
  if(payload.device_cmd.device&DEV_VIBRATOR){
  }
  if(payload.device_cmd.device&DEV_POWER_LED){
  }
}

static int m_active_function_counter = 0;

static struct device_state_s{
  uint8_t desired_val;
  void(*associated_callback)(bool);
  bool turned_on;
  e_funcType func;
  uint32_t period;
  uint32_t duration;
  uint32_t cur_period;
  uint32_t cur_duration;
  float alpha;
}m_device_state[] =
{
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF}
};

static ic_return_val_e refresh_device(ic_actuator_e device){
  return ic_actuator_set(
      device,
      m_device_state[device].desired_val,
      m_device_state[device].associated_callback);
}

static void refresh_all_devices(){
  for (int i = 0; i < sizeof(m_device_state)/sizeof(m_device_state[0]); ++i){
    if(m_device_state[i].turned_on){
      __auto_type _ret_val = refresh_device(i);
      if(_ret_val != IC_SUCCESS)
        NRF_LOG_INFO("Could not refresh device %d: {%s}", i, (uint32_t)g_return_val_string[_ret_val]);
    }
  }
}

static void m_ltc_refresh_timer_callback(TimerHandle_t xTimer){
  refresh_all_devices();
}

static uint8_t function_saw(uint8_t step){
  return step>>1;
}

static uint8_t function_triangle(uint8_t step){
  return step<64?step:127-step;
}

uint8_t(*m_function_map[])(uint8_t) = {
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  function_saw,
  function_triangle,
  NULL
};

static uint8_t calculate_val(uint8_t step, e_funcType func){
  m_function_map[func](step);
}

static void refresh_time(struct device_state_s* device){
  device->cur_duration += QUANTUM_OF_TIME;
  device->cur_period += QUANTUM_OF_TIME;
}

static void refresh_all_times(){
  for (int i = 0; i < sizeof(m_device_state)/sizeof(m_device_state[0]); ++i){
    if(m_device_state[i].turned_on){
      refresh_time(m_device_state[i]);
    }
  }
}

static void refresh_function(struct device_state_s* device){
  device->desired_val = calculate_val(device->cur_period*device->alpha, device->func);
}

static void refresh_all_functions(){
  for (int i = 0; i < sizeof(m_device_state)/sizeof(m_device_state[0]); ++i){
    if(m_device_state[i].turned_on){
      refresh_function(m_device_state[i]);
    }
  }
}

static void function_task(void *arg){
  __auto_type last_wake_time = xTaskGetTickCount();

  for(;;){
    if(m_active_function_counter < 1){
      vTaskSuspend(NULL);
      last_wake_time = xTaskGetTickCount();
    }

    refresh_all_times();
    refresh_all_functions();
    refresh_all_devices();

    xTimerReset(m_ltc_refresh_timer_handle, 3);
    vTaskDelayUntil(&last_wake_time, QUANTUM_OF_TIME);

  }
}

ic_return_val_e ic_ltc_service_init(void *arg){
  cmd_task_connect_to_device_cmd(m_device_parse);

  if(m_ltc_refresh_timer_handle == NULL)
    m_ltc_refresh_timer_handle = xTimerCreate(
        "LTC_TIMER",
        pdMS_TO_TICKS(200),
        pdTRUE,
        NULL,
        m_ltc_refresh_timer_callback);
}
