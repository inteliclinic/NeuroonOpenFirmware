/**
 * @file    ic_driver_ltc.c
    nvTaskDelay(3);
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_ltc.h"
#include "ic_driver_twi.h"
#include <string.h>

#define NRF_LOG_MODULE_NAME "LTC"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define ENABLE_CHANNEL(channel) (channel.meta_data.enabled = 1)
#define DISABLE_CHANNEL(channel) (channel.meta_data.enabled = 0)

TWI_REGISTER(ltc_twi, 0x38);

static struct{
  struct{
    char value : 6;
    char reseved : 1;
    char enabled : 1;
  }meta_data;
  uint8_t bufer[2];
}m_ltc_channels[19];

/*
 *static void m_twi_callback(ic_return_val_e ret, void *context){
 *  if(context != NULL){
 *    void(*fp)(s_ltc_result) = context;
 *    s_ltc_result _result = {.channel = 1, .success = ret == IC_SUCCESS};
 *    fp(_result);
 *  }
 *}
 */

ic_return_val_e ic_ltc_driver_init(void){
  TWI_INIT(ltc_twi);
  memset(m_ltc_channels, 0x00, sizeof(m_ltc_channels));
  return IC_SUCCESS;
}

ic_return_val_e ic_ltc_driver_deinit(void){
  TWI_DEINIT(ltc_twi);
  return IC_SUCCESS;
}

ic_return_val_e ic_set_channel(
    uint8_t channel,
    uint8_t val,
    void(*fp)(ic_return_val_e, void *),
    void *context)
{
  /*
   *if(!m_ltc_channels[channel].meta_data.enabled)
   *  return IC_NOT_ENABLED;
   */

  m_ltc_channels[channel].bufer[0] = channel;
  m_ltc_channels[channel].bufer[1] = val&0x3F;

  __auto_type _ret_val = TWI_SEND_DATA(ltc_twi, m_ltc_channels[channel].bufer, 2, fp, context);

  if(_ret_val != IC_SUCCESS){
    ic_twi_refresh_bus();
  }

  return _ret_val;
}

ic_return_val_e ic_enable_channel(uint8_t channel){
  ENABLE_CHANNEL(m_ltc_channels[channel]);

  return IC_SUCCESS;
}

ic_return_val_e ic_disable_channel(uint8_t channel){
  DISABLE_CHANNEL(m_ltc_channels[channel]);

  return IC_SUCCESS;
}
