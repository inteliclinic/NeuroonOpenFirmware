/**
 * @file    ic_driver_spi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    czerwiec, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_spi.h"
#include "ic_config.h"

#include "app_error.h"

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

#define NRF_LOG_MODULE_NAME "SPI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

static struct{
  const nrf_drv_spi_t nrf_drv_instance;
  uint8_t spi_instance_cnt;
  ic_spi_event_cb callback;
}m_curren_state = {
                    .nrf_drv_instance = NRF_DRV_SPI_INSTANCE(IC_SPI_INSTANCE),
                    .spi_instance_cnt = 0,
                    .callback         = NULL
                  };

static void spi_event_handler(nrf_drv_spi_evt_t const *p_event){
  UNUSED_VARIABLE(p_event);

  if(m_curren_state.callback != (void *)0xDEADBEEF)
    m_curren_state.callback(NULL);

  m_curren_state.callback = NULL;
}

ic_return_val_e ic_spi_init(ic_spi_instance_s *instance, uint8_t pin){
  ASSERT(instance!=NULL);

  instance->nrf_spi_instance = (void *)&m_curren_state.nrf_drv_instance;

  instance->pin = pin;
  nrf_gpio_cfg_output(instance->pin);

  if(m_curren_state.spi_instance_cnt++ == 0){
    nrf_drv_spi_config_t _spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    _spi_config.ss_pin    = NRF_DRV_SPI_PIN_NOT_USED;
    _spi_config.miso_pin  = IC_SPI_MISO_PIN;
    _spi_config.mosi_pin  = IC_SPI_MOSI_PIN;
    _spi_config.sck_pin   = IC_SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_curren_state.nrf_drv_instance, &_spi_config, spi_event_handler));
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_spi_send(const ic_spi_instance_s *const instance, uint8_t *in_buffer,
    size_t in_len, uint8_t *out_buffer, uint8_t out_len, ic_spi_event_cb callback){
  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(out_buffer!=NULL);

  if(m_curren_state.callback != NULL && m_curren_state.callback != (void *)0xDEADBEEF)
    return IC_BUSY;

  m_curren_state.callback = callback == NULL ? (void *)0xDEADBEEF : callback;

  __auto_type _ret_val = nrf_drv_spi_transfer(instance->nrf_spi_instance, in_buffer, in_len,
      out_buffer, out_len);

  switch(_ret_val){
    case NRF_SUCCESS:
      return IC_SUCCESS;
    case NRF_ERROR_BUSY:
      m_curren_state.callback = (void *)0xDEADBEEF;
      return IC_BUSY;
    default:
      return IC_ERROR;
  }
}

void ic_spi_cs_high(ic_spi_instance_s *instance){
  nrf_gpio_pin_set(instance->pin);
}

void ic_spi_cs_low(ic_spi_instance_s *instance){
  nrf_gpio_pin_clear(instance->pin);
}
