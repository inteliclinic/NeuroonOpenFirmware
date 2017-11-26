/**
 * @file    ic_driver_actuators.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_actuators.h"
#include "ic_driver_ltc.h"

typedef enum{
  CHNL_LEFT_GREEN_LED = 0x01,
  CHNL_LEFT_RED_LED = 0x02,
  CHNL_LEFT_BLUE_LED = 0x03,
  CHNL_VIBRATOR_CHANNEL1 = 0x08,
  CHNL_VIBRATOR_CHANNEL2 = 0x09,
  CHNL_POWER_LEDS = 0x0C,
  CHNL_RIGHT_GREEN_LED = 0x10,
  CHNL_RIGHT_RED_LED = 0x11,
  CHNL_RIGHT_BLUE_LED = 0x12
}e_device2channel;

static struct device_value_s{
  e_device device;
  e_device2channel channel;
  uint8_t val;
  void(*fp)(bool);
} m_device_value[] =
{
  {.device = LEFT_GREEN_LED,  .channel = CHNL_LEFT_GREEN_LED,   .val = 0, .fp = NULL},
  {.device = LEFT_RED_LED,    .channel = CHNL_LEFT_RED_LED,     .val = 0, .fp = NULL},
  {.device = LEFT_BLUE_LED,   .channel = CHNL_LEFT_BLUE_LED,    .val = 0, .fp = NULL},
  {.device = RIGHT_GREEN_LED, .channel = CHNL_RIGHT_GREEN_LED,  .val = 0, .fp = NULL},
  {.device = RIGHT_RED_LED,   .channel = CHNL_RIGHT_RED_LED,    .val = 0, .fp = NULL},
  {.device = RIGHT_BLUE_LED,  .channel = CHNL_RIGHT_BLUE_LED,  .val = 0, .fp = NULL},
  {.device = VIBRATOR,        .channel = CHNL_VIBRATOR_CHANNEL1,.val = 0, .fp = NULL},
  {.device = POWER_LEDS,      .channel = CHNL_POWER_LEDS,       .val = 0, .fp = NULL}
};

static void m_set_vibrator_callback(ic_return_val_e result, void *context){
  if(context == NULL) return;

  struct device_value_s *_device_value = context;

  if(_device_value->fp != NULL){
    _device_value->fp(result == IC_SUCCESS);
    _device_value->fp = NULL;
  }
}

static void m_set_channel_callback(ic_return_val_e result, void *context){
  if(context == NULL){}

  struct device_value_s *_device_value = context;

  if(_device_value->device == VIBRATOR){
    ic_set_channel(
        CHNL_VIBRATOR_CHANNEL2,
        _device_value->val,
        m_set_vibrator_callback,
        _device_value);
    return;
  }

  if(_device_value->fp != NULL){
    _device_value->fp(result == IC_SUCCESS);
    _device_value->fp = NULL;
  }
}

ic_return_val_e ic_actuator_init(){
  ic_ltc_driver_init();
  /*for(int i=0; i<(sizeof(m_device_value)/sizeof(m_device_value[0])); ++i){*/
    /*ic_enable_channel(m_device_value[i].channel);*/
  /*}*/
  return IC_SUCCESS;
}

ic_return_val_e ic_actuator_set(e_device device, uint8_t val, void(*fp)(bool)){
  __auto_type _val_tmp = m_device_value[device].val;
  m_device_value[device].val = val;
  m_device_value[device].fp = fp;

  __auto_type _ret_val = ic_set_channel(
      m_device_value[device].channel,
      val,
      m_set_channel_callback,
      &m_device_value[device]);

  if(_ret_val != IC_SUCCESS)
    m_device_value[device].val = _val_tmp;

  return _ret_val;
}
