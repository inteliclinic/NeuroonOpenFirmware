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
#include "nrf_gpio.h"

#define QUANTUM_OF_TIME 16

static volatile uint32_t m_active_function_counter = 0;

const float m_beta_factor = 1.0/IC_LTC_MAX_VAL;
const float m_beta_factor_vib0 = 1.0/IC_LTC_VIB_MAX_VAL;
const float m_beta_factor_vib1 = IC_LTC_VIB_MAX_VAL/63.0;

static TimerHandle_t m_ltc_refresh_timer_handle = NULL;
static TimerHandle_t m_ltc_blink_timer_handle = NULL;
static TaskHandle_t m_ltc_refresh_task_handle = NULL;
static bool m_module_initialized = false;

static struct device_state_s{
  uint8_t desired_val;
  ic_actuator_e device;
  void(*associated_callback)(bool);
  bool turned_on;
  bool refresh_ltc;
  e_funcType func;
  uint32_t period;
  uint32_t duration;
  uint32_t cur_period;
  uint32_t cur_duration;
  uint8_t intensity;
  float alpha;
  float beta;
  float a_ramp_coef;
  uint8_t b_ramp_coef;
  uint8_t ramp_step;
}m_device_state[] =
{
  {.desired_val = 0x00, .device = ACTUATOR_LEFT_GREEN_LED,   .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_LEFT_RED_LED,     .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_LEFT_BLUE_LED,    .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_RIGHT_GREEN_LED,  .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_RIGHT_RED_LED,    .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_RIGHT_BLUE_LED,   .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_VIBRATOR,         .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00},
  {.desired_val = 0x00, .device = ACTUATOR_POWER_LEDS,       .associated_callback = NULL,  .turned_on = false,  .refresh_ltc = false, .func = FUN_TYPE_OFF, .intensity = 0x00}
};

static ic_return_val_e actuator_set_func(
    struct device_state_s *device,
    e_funcType func,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

static void m_device_parse(u_BLECmdPayload payload){

  __auto_type _period = pdMS_TO_TICKS((payload.device_cmd.func_parameter.periodic_func.period*100));
  __auto_type _duration = pdMS_TO_TICKS((payload.device_cmd.func_parameter.periodic_func.duration*100));

  NRF_LOG_INFO("dev: 0x%X\tfunc: %d\tperiod: %d\tduration: %d\n",
      payload.device_cmd.device,
      payload.device_cmd.func_type,
      payload.device_cmd.func_parameter.periodic_func.period,
      payload.device_cmd.func_parameter.periodic_func.duration);

  if(payload.device_cmd.device&DEV_RIGHT_RED_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_RIGHT_RED_LED],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.right_red_led);
  }
  if(payload.device_cmd.device&DEV_RIGHT_GREEN_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_RIGHT_GREEN_LED],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.right_green_led);
  }
  if(payload.device_cmd.device&DEV_RIGHT_BLUE_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_RIGHT_BLUE_LED],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.right_blue_led);
  }
  if(payload.device_cmd.device&DEV_LEFT_RED_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_LEFT_RED_LED],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.left_red_led);
  }
  if(payload.device_cmd.device&DEV_LEFT_GREEN_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_LEFT_GREEN_LED],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.left_green_led);
  }
  if(payload.device_cmd.device&DEV_LEFT_BLUE_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_LEFT_BLUE_LED],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.left_blue_led);
  }
  if(payload.device_cmd.device&DEV_VIBRATOR){
    actuator_set_func(
        &m_device_state[ACTUATOR_VIBRATOR],
        payload.device_cmd.func_type,
        _period,
        _duration,
        payload.device_cmd.intensity.vibrator);
  }
  if(payload.device_cmd.device&DEV_POWER_LED){
    actuator_set_func(
        &m_device_state[ACTUATOR_POWER_LEDS],
        payload.device_cmd.func_type,
        _period,
        _duration,
        0);
  }
}

static uint8_t function_ramp_down(struct device_state_s *device){
  uint8_t _step = QUANTUM_OF_TIME * device->beta;
  if (_step == 0) _step = 1;
  return ({int16_t _ret_val =
      device->desired_val - _step; _ret_val<0 ? 0 : _ret_val;});
}

static uint8_t function_ramp_up(struct device_state_s *device){
  uint8_t _step = QUANTUM_OF_TIME * device->beta;
  if (_step == 0) _step = 1;
  return ({__auto_type _ret_val =
      device->desired_val + _step; _ret_val>device->intensity ? device->intensity : _ret_val;});
}

uint8_t function_ramp(struct device_state_s *device){
  int16_t _step = device->cur_period*device->alpha;
  uint8_t _ret_val;
  NRF_LOG_INFO("_step:%d\tramp_step:%d\n", _step, device->ramp_step);
  if(_step<device->ramp_step || _step==127){
    NRF_LOG_INFO("Turning on\n");
    _ret_val = 127*device->a_ramp_coef + device->b_ramp_coef;
    actuator_set_func(device,FUN_TYPE_ON, 0, 0, _ret_val);
    return device->desired_val;
  }
  _ret_val = _step*device->a_ramp_coef + device->b_ramp_coef;
  device->ramp_step = _step;
  return _ret_val;
}

static uint8_t function_off(struct device_state_s *device){
  return device->device == ACTUATOR_POWER_LEDS ? 0 : function_ramp_down(device);
}

static uint8_t function_on(struct device_state_s *device){
  return device->intensity;
}

static uint8_t function_saw(struct device_state_s *device){
  uint8_t _step = device->cur_period*device->alpha;
  return (_step>>1)*device->beta;
}

static uint8_t function_triangle(struct device_state_s *device){
  uint8_t _step = device->cur_period*device->alpha;
  return (_step<64?_step:127-_step)*device->beta;
}

static uint8_t function_square(struct device_state_s *device){
  uint8_t _step = device->cur_period*device->alpha;
  uint8_t _ret_val;
  if(_step<64){
    if(device->device == ACTUATOR_VIBRATOR)
      _ret_val = device->intensity;
    else
      _ret_val = device->intensity == device->desired_val ?
        device->intensity : function_ramp_up(device);
  } else {
    if(device->device == ACTUATOR_VIBRATOR)
      _ret_val = 0;
    else
      _ret_val = 0 == device->desired_val ? 0 : function_ramp_down(device);
  }
  return _ret_val;
}

static uint8_t function_blink(struct device_state_s *device){
  uint8_t _ret_val;
  if(device->cur_period<QUANTUM_OF_TIME){
    _ret_val = device->intensity;
  } else {
    _ret_val = 0;
  }
  return _ret_val;
}

uint8_t(*m_function_map[])(struct device_state_s *) = {
  function_off,
  function_off,
  function_on,
  function_triangle, // it is dummy func(should be sinus)
  function_blink,
  function_square,
  function_saw,
  function_triangle,
  function_ramp
};

static uint8_t calculate_val(struct device_state_s *device){
  uint8_t _offset = 0;

  __auto_type _ret_val = m_function_map[device->func] != NULL ?
    m_function_map[device->func](device)+_offset : ({/*NRF_LOG_INFO("Function not implemented\n");*/ 0;});

  if(device->device == ACTUATOR_VIBRATOR)
    if(device->func != FUN_TYPE_OFF && _ret_val>0)
      _offset = IC_LTC_VIB_MIN_VAL;

  return _ret_val+_offset;
}

#define REFRESH_ALL(func) do{\
  for (int i = 0; i < sizeof(m_device_state)/sizeof(m_device_state[0]); ++i){\
    if(m_device_state[i].turned_on) refresh_##func(&m_device_state[i]);\
  }\
}while(0)

static void m_power_led_cb(bool b){
  nrf_gpio_pin_set(24);
  __auto_type _dummy = pdFAIL;
  START_TIMER(m_ltc_blink_timer_handle, 0, _dummy);
  UNUSED_PARAMETER(_dummy);
}

static void refresh_device(struct device_state_s * device){
  __auto_type _ret_val = IC_SUCCESS;

  if(device->device == ACTUATOR_POWER_LEDS){
    if(device->cur_period < QUANTUM_OF_TIME){
      _ret_val = ic_actuator_set(
          ACTUATOR_POWER_LEDS,
          63,
          m_power_led_cb);
    }
  }
  else if(device->refresh_ltc){
    _ret_val = ic_actuator_set(
        device->device,
        device->desired_val,
        device->associated_callback);
  }

  if(_ret_val != IC_SUCCESS){
    NRF_LOG_INFO("Could not refresh device %d: {%s}\n",
        device->device, (uint32_t)g_return_val_string[_ret_val]);
  }
}

static void refresh_time(struct device_state_s * device){
  if((device->cur_period += QUANTUM_OF_TIME)>device->period){
    __auto_type _delta = device->cur_period - device->period;
    device->cur_period = _delta-1;
  }

  if(device->duration > 0)
    if((device->cur_duration += QUANTUM_OF_TIME)>=device->duration){
      device->func = FUN_TYPE_OFF;
    }
}

static void refresh_function(struct device_state_s *device){
  __auto_type _tmp_val = calculate_val(device);
  device->refresh_ltc = _tmp_val == device->desired_val ? false : true;
  if(device->refresh_ltc){
    device->desired_val = _tmp_val;
  }
}

static void refresh_activation(struct device_state_s *device){
  if(
      ((device->desired_val == 0)&&(device->func == FUN_TYPE_OFF))
      /*|| ((device->desired_val == device->intensity)&&(device->func == FUN_TYPE_ON))*/
    )
  {
    device->turned_on = false;
    m_active_function_counter--;
  }
}

static void ltc_refresh_timer_callback(TimerHandle_t xTimer){
  REFRESH_ALL(device);
}

static void m_ltc_turned_off(bool b){
  UNUSED_PARAMETER(b);
  nrf_gpio_pin_clear(24);
}

static void ltc_power_led_timer_callback(TimerHandle_t xTimer){
  ic_actuator_set(ACTUATOR_POWER_LEDS, 0, m_ltc_turned_off);
  /*REFRESH_ALL(device);*/
}

static void ltc_refresh_task_callback(void *arg){
  __auto_type _last_wake_time = GET_TICK_COUNT();

  for(;;){
    if(_last_wake_time == GET_TICK_COUNT()){
      _last_wake_time = GET_TICK_COUNT();
    }

    if(m_active_function_counter < 1){
      vTaskSuspend(NULL);
      _last_wake_time = GET_TICK_COUNT();
    }

    REFRESH_ALL(time);
    REFRESH_ALL(function);
    REFRESH_ALL(device);
    REFRESH_ALL(activation);

    xTimerReset(m_ltc_refresh_timer_handle, 2);

    vTaskDelayUntil(&_last_wake_time, QUANTUM_OF_TIME);
  }
}

static ic_return_val_e actuator_set_func(
    struct device_state_s *device,
    e_funcType func,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  device->alpha = (128.0)/(period);

  if((device->device == ACTUATOR_VIBRATOR) && (func != FUN_TYPE_OFF)){
    device->intensity = intensity*m_beta_factor_vib1;
    device->beta = device->intensity*m_beta_factor;
  }
  else if(func==FUN_TYPE_RAMP){
    device->b_ramp_coef = device->desired_val;
    int _tmp = (intensity - device->desired_val);
    device->a_ramp_coef = _tmp/127.0;
    device->ramp_step = 0;
    NRF_LOG_INFO("_tmp: %d, intensity: %d, device->intensity: %d a: " NRF_LOG_FLOAT_MARKER "\n",_tmp, intensity, device->intensity, NRF_LOG_FLOAT(device->a_ramp_coef));
    device->intensity = intensity;
  }
  else{
    device->beta = func==FUN_TYPE_OFF ? device->desired_val*m_beta_factor : intensity*m_beta_factor;
    device->intensity = intensity;
  }

  device->cur_period = 0;
  device->cur_duration = 0;

  if(device->device == ACTUATOR_POWER_LEDS && func != FUN_TYPE_BLINK)
    device->func = FUN_TYPE_OFF;
  else
    device->func = func;

  device->period = period-1;
  device->duration = duration == 0 ? 0 : duration-1;

  if(!device->turned_on){
    device->turned_on = true;
    if(m_active_function_counter++ == 0){
      RESUME_TASK(m_ltc_refresh_task_handle);
    }
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_actuator_set_off_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_OFF,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_on_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_ON,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_sin_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_SIN_WAVE,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_blink_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_BLINK,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_square_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_SQUARE,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_saw_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_SAW,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_triangle_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_TRIANGLE,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_actuator_set_ramp_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity)
{
  return actuator_set_func(
      &m_device_state[device],
      FUN_TYPE_RAMP,
      period,
      duration,
      intensity);
}

ic_return_val_e ic_ltc_service_init(){
  if(m_module_initialized) return IC_SUCCESS;

  nrf_gpio_cfg_output(24);
  ic_actuator_init();

  cmd_task_connect_to_device_cmd(m_device_parse);

  if(m_ltc_refresh_timer_handle == NULL)
    m_ltc_refresh_timer_handle = xTimerCreate(
        "LTC_TIMER",
        pdMS_TO_TICKS(200),
        pdTRUE,
        NULL,
        ltc_refresh_timer_callback);

  if(m_ltc_blink_timer_handle == NULL)
    m_ltc_blink_timer_handle = xTimerCreate(
        "LTC_TIMER",
        2,
        pdFALSE,
        NULL,
        ltc_power_led_timer_callback);

  if(pdPASS != xTaskCreate(
        ltc_refresh_task_callback,
        "LTCS",
        384,
        NULL,
        IC_IRQ_PRIORITY_LOW,
        &m_ltc_refresh_task_handle))
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);

  m_module_initialized = true;

  return IC_SUCCESS;
}

ic_return_val_e ic_ltc_service_deinit(){
  nrf_gpio_cfg_default(24);
  if(m_module_initialized == false) return IC_NOT_INIALIZED;
  ic_actuator_deinit();

  __auto_type _ret_val = pdTRUE;
  UNUSED_VARIABLE(_ret_val);
  STOP_TIMER(m_ltc_refresh_timer_handle, 0, _ret_val);

  vTaskSuspend(m_ltc_refresh_task_handle);

  return IC_SUCCESS;
}
