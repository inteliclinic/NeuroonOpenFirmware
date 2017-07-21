/**
 * @file    ic_driver_twi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */
#include "nrf_drv_twi.h"

#include "ic_driver_twi.h"
#include "ic_config.h"

#include "app_error.h"
#include "sdk_errors.h"
#include "nrf_assert.h"

#define NRF_LOG_MODULE_NAME "TWI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static struct{
  const nrf_drv_twi_t nrf_drv_instance;
  uint8_t twi_instance_cnt;
  ic_twi_event_cb callback;
}m_curren_state = {
                    .nrf_drv_instance = NRF_DRV_TWI_INSTANCE(IC_TWI_INSTANCE),
                    .twi_instance_cnt = 0,
                    .callback          = NULL
                  };

static void twi_event_handler(nrf_drv_twi_evt_t const *p_event, void *p_context){
  /*UNUSED_VARIABLE(p_event);*/
  UNUSED_VARIABLE(p_context);
  switch(p_event->type){
    case NRF_DRV_TWI_EVT_DONE:
      break;
    case NRF_DRV_TWI_EVT_ADDRESS_NACK:
      break;
    case NRF_DRV_TWI_EVT_DATA_NACK:
      break;
  }

  if(m_curren_state.callback != (void *)0xDEADBEEF)
    m_curren_state.callback(NULL);

  m_curren_state.callback = NULL;
  nrf_drv_twi_disable(&m_curren_state.nrf_drv_instance);
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

    APP_ERROR_CHECK(nrf_drv_twi_init(&m_curren_state.nrf_drv_instance, &_twi_config,
          twi_event_handler, NULL));
  }

  /*m_curren_state.twi_instance_cnt++;*/
  return IC_SUCCESS;
}

ic_return_val_e ic_twi_send(const ic_twi_instance_s *const instance, uint8_t address,
    uint8_t *in_buffer, size_t len, ic_twi_event_cb callback){

  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(len<=255);

  if(m_curren_state.callback != NULL && m_curren_state.callback != (void *)0xDEADBEEF)
    return IC_BUSY;

  nrf_drv_twi_enable(&m_curren_state.nrf_drv_instance);
  __auto_type _ret_val = nrf_drv_twi_tx((nrf_drv_twi_t const *)instance->nrf_twi_instance, address,
      in_buffer, (uint8_t)len, false);
  switch(_ret_val){
    case NRF_SUCCESS:
      m_curren_state.callback = callback == NULL ? (void*) 0xDEADBEEF : callback;
      return IC_SUCCESS;
    case NRF_ERROR_BUSY:
      return IC_BUSY;
    default:
      return IC_ERROR;
  }
}

/*void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info){*/
  /*NRF_LOG_ERROR("%s\n\r", (uint32_t)ERR_TO_STR(((error_info_t *)info)->err_code));*/
  /*NRF_LOG_FINAL_FLUSH();*/
/*}*/

ic_return_val_e ic_twi_read(const ic_twi_instance_s *const instance, uint8_t address,
    uint8_t reg_addr, uint8_t *in_buffer, size_t len, ic_twi_event_cb callback){

  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(len<=255);

  if(m_curren_state.callback != NULL && m_curren_state.callback != (void *)0xDEADBEEF)
    return IC_BUSY;

  static uint8_t _reg_addr;
  _reg_addr = reg_addr;

  nrf_drv_twi_enable(&m_curren_state.nrf_drv_instance);
  nrf_drv_twi_xfer_desc_t _xfer =
    NRF_DRV_TWI_XFER_DESC_TXRX(address, &_reg_addr, sizeof(_reg_addr), in_buffer, len);

  __auto_type _ret_val = nrf_drv_twi_xfer((nrf_drv_twi_t const *)instance->nrf_twi_instance, &_xfer, 0);

  switch(_ret_val){
    case NRF_SUCCESS:
      m_curren_state.callback = callback == NULL ? (void*) 0xDEADBEEF : callback;
      return IC_SUCCESS;
    case NRF_ERROR_BUSY:
      return IC_BUSY;
    default:
      return IC_ERROR;
  }
}
