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
/*#include "ic_service_ltc.h"*/
#include "ic_driver_actuators.h"

#include "ic_command_task.h"
#include "ic_frame_handle.h"
#include "ic_low_level_control.h"

#define NRF_LOG_MODULE_NAME "LTC-SERVICE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define QUANTUM_OF_TIME 16

static TimerHandle_t m_ltc_refresh_timer_handle = NULL;
static TaskHandle_t m_ltc_refresh_task_handle = NULL;

static void m_device_parse(u_BLECmdPayload payload){
  if(payload.device_cmd.device&DEV_RIGHT_RED_LED){
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
  ic_actuator_e device;
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
  {.desired_val = 0x00, .device = ACTUATOR_LEFT_GREEN_LED,   .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_LEFT_RED_LED,     .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_LEFT_BLUE_LED,    .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_RIGHT_GREEN_LED,  .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_RIGHT_RED_LED,    .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_RIGHT_BLUE_LED,   .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_VIBRATOR,         .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF},
  {.desired_val = 0x00, .device = ACTUATOR_POWER_LEDS,       .associated_callback = NULL,  .turned_on = false, .func = FUN_TYPE_OFF}
};

static uint8_t function_saw(uint8_t step, void *p_context){
  return step>>1;
}

static uint8_t function_triangle(uint8_t step, void *p_context){
  return step<64?step:127-step;
}

static uint8_t function_off(uint8_t step, void *p_context){
  return 0;
}

static uint8_t function_ramp_down(uint8_t step, void *p_context){
  return ({__auto_type _ret_val =
      ((struct device_state_s *)p_context)->desired_val - 3; _ret_val<0 ? 0 : _ret_val;});
}

uint8_t(*m_function_map[])(uint8_t, void *) = {
  function_off,
  function_ramp_down,
  NULL,
  NULL,
  NULL,
  NULL,
  function_saw,
  function_triangle,
  NULL
};

static uint8_t calculate_val(uint8_t step, struct device_state_s *device){
  NRF_LOG_INFO("step: %d\n", step);
  return m_function_map[device->func] != NULL ?
    m_function_map[device->func](step, device) : ({NRF_LOG_INFO("Function not implemented\n"); 0;});
}

#define REFRESH_ALL(func) do{\
  for (int i = 0; i < sizeof(m_device_state)/sizeof(m_device_state[0]); ++i){\
    if(m_device_state[i].turned_on) refresh_##func(&m_device_state[i]);\
  }\
}while(0)

static void refresh_device(struct device_state_s * device){
  NRF_LOG_INFO("device->desired_val = %d\n", device->desired_val);
  __auto_type _ret_val = ic_actuator_set(
      device->device,
      device->desired_val,
      device->associated_callback);

  if(_ret_val != IC_SUCCESS)
    NRF_LOG_INFO("Could not refresh device %d: {%s}",
        device->device, (uint32_t)g_return_val_string[_ret_val]);
}

static void refresh_time(struct device_state_s * device){
  if((device->cur_period += QUANTUM_OF_TIME)>device->period){
    __auto_type _delta = device->cur_period - device->period;
    device->cur_period = _delta;
  }

  if((device->cur_duration += QUANTUM_OF_TIME)>=device->duration){
    NRF_LOG_INFO("offing, because\n");
    device->func = FUN_TYPE_OFF;
  }
}

static void refresh_function(struct device_state_s *device){
  NRF_LOG_INFO("alpha = %d, cur_period = %d\n",device->alpha, device->cur_period);
  device->desired_val = calculate_val(device->cur_period*device->alpha, device);
}

static void refresh_activation(struct device_state_s *device){
  if((device->desired_val == 0)&&(device->func == FUN_TYPE_OFF)){
    NRF_LOG_INFO("Deactivate\n");
    device->turned_on = false;
    m_active_function_counter--;
  }
}

static void ltc_refresh_timer_callback(TimerHandle_t xTimer){
  NRF_LOG_INFO("{%s}\n", (uint32_t)__func__);
  REFRESH_ALL(device);
}

static void ltc_refresh_task_callback(void *arg){
  __auto_type last_wake_time = xTaskGetTickCount();

  for(;;){
    if(m_active_function_counter < 1){
      vTaskSuspend(NULL);
      last_wake_time = xTaskGetTickCount();
    }

    NRF_LOG_INFO("Task: %s\n", (uint32_t)__func__);

    REFRESH_ALL(time);
    REFRESH_ALL(function);
    REFRESH_ALL(device);
    REFRESH_ALL(activation);

    xTimerReset(m_ltc_refresh_timer_handle, 2);

    vTaskDelayUntil(&last_wake_time, QUANTUM_OF_TIME);
  }
}

ic_return_val_e ic_actuator_lgl_func_triangle(
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  NRF_LOG_INFO("{%s}\n",(uint32_t)__func__);
  m_device_state[ACTUATOR_LEFT_GREEN_LED].alpha = (128.0*intensity)/(63*period);
  m_device_state[ACTUATOR_LEFT_GREEN_LED].cur_period = 0;
  m_device_state[ACTUATOR_LEFT_GREEN_LED].cur_duration = 0;
  m_device_state[ACTUATOR_LEFT_GREEN_LED].func = FUN_TYPE_TRIANGLE;
  m_device_state[ACTUATOR_LEFT_GREEN_LED].turned_on = true;
  m_device_state[ACTUATOR_LEFT_GREEN_LED].period = period-1;
  m_device_state[ACTUATOR_LEFT_GREEN_LED].duration = duration-1;
  if(m_active_function_counter++ == 0){
    RESUME_TASK(m_ltc_refresh_task_handle);
  }
  return IC_SUCCESS;
}

ic_return_val_e ic_ltc_service_init(){

  ic_actuator_init();

  cmd_task_connect_to_device_cmd(m_device_parse);

  if(m_ltc_refresh_timer_handle == NULL)
    m_ltc_refresh_timer_handle = xTimerCreate(
        "LTC_TIMER",
        pdMS_TO_TICKS(200),
        pdTRUE,
        NULL,
        ltc_refresh_timer_callback);

  if(pdPASS != xTaskCreate(
        ltc_refresh_task_callback,
        "LTCS",
        256,
        NULL,
        2,
        &m_ltc_refresh_task_handle))
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

  return IC_SUCCESS;
}
