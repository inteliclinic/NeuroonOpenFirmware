/**
 * @file    ic_driver_twi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */
#include "nrf_drv_twi.h"
#include "app_twi.h"

#include "ic_driver_twi.h"
#include "ic_config.h"

#include "app_error.h"
#include "sdk_errors.h"
#include "nrf_assert.h"

#define NRF_LOG_MODULE_NAME "TWI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static struct{
  app_twi_t nrf_drv_instance;
  uint8_t twi_instance_cnt;
  ic_twi_event_cb callback;
}m_curren_state = {
                    .nrf_drv_instance = APP_TWI_INSTANCE(IC_TWI_INSTANCE),
                    .twi_instance_cnt = 0,
                    .callback          = NULL
                  };

static void twi_event_handler(uint32_t result, void *p_context){
  UNUSED_VARIABLE(p_context);
  UNUSED_PARAMETER(p_context);

  if (result == NRF_SUCCESS) {
    if(m_curren_state.callback != NULL){
      m_curren_state.callback(NULL);
      m_curren_state.callback = NULL;
    }
  }

}

ic_return_val_e ic_twi_init(ic_twi_instance_s * instance){

  ASSERT(instance!=NULL);

  instance->nrf_twi_instance = (void *)&m_curren_state.nrf_drv_instance;

  if(m_curren_state.twi_instance_cnt++ == 0){
    nrf_drv_twi_config_t _twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    _twi_config.sda                 = IC_TWI_SDA_PIN;
    _twi_config.scl                 = IC_TWI_SCL_PIN;
    _twi_config.interrupt_priority  = IC_TWI_IRQ_PRIORITY;
    _twi_config.frequency           = (nrf_twi_frequency_t)IC_TWI_FREQUENCY;
    _twi_config.clear_bus_init      = true;
    _twi_config.hold_bus_uninit     = true;

    uint32_t err_code;

    APP_TWI_INIT(&m_curren_state.nrf_drv_instance, &_twi_config, IC_TWI_PENDIG_TRANSACTIONS, err_code);
    APP_ERROR_CHECK(err_code);
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_twi_send(const ic_twi_instance_s *const instance, uint8_t address,
    uint8_t *in_buffer, size_t len, ic_twi_event_cb callback){

  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(len<=255);

  app_twi_transfer_t _transfers[] = {
    APP_TWI_WRITE(address, in_buffer, len, 0x00)
  };

  app_twi_transaction_t _transaction = {
    .callback     = twi_event_handler,
    .p_transfers  = _transfers,
    .p_user_data  = NULL,
    .number_of_transfers = sizeof(_transfers)/sizeof(_transfers[0])
  };

  switch (app_twi_schedule(&m_curren_state.nrf_drv_instance, &_transaction)){
    case NRF_SUCCESS:
      m_curren_state.callback = callback;
      int _to = 192;
      while(_to--)
        __NOP();
      return IC_SUCCESS;
    case NRF_ERROR_BUSY:
      return IC_BUSY;
    default:
      return IC_ERROR;
  }
}

ic_return_val_e ic_twi_read(const ic_twi_instance_s *const instance, uint8_t address,
    uint8_t reg_addr, uint8_t *in_buffer, size_t len, ic_twi_event_cb callback){

  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(len<=255);

  app_twi_transfer_t _transfers[] = {
    APP_TWI_WRITE(address, &reg_addr, 1, APP_TWI_NO_STOP),
    APP_TWI_READ (address, in_buffer, len, 0x00)
  };

  app_twi_transaction_t _transaction = {
    .callback     = twi_event_handler,
    .p_user_data  = NULL,
    .p_transfers  = _transfers,
    .number_of_transfers = sizeof(_transfers)/sizeof(_transfers[0])
  };

  switch (app_twi_schedule(&m_curren_state.nrf_drv_instance, &_transaction)){
    case NRF_SUCCESS:
      m_curren_state.callback = callback;
      return IC_SUCCESS;
    case NRF_ERROR_BUSY:
      return IC_BUSY;
    default:
      return IC_ERROR;
  }
}
