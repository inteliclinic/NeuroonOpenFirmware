/**
 * @file    ic_driver_twi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_twi.h"
#include "ic_config.h"


#include "app_error.h"
#define NRF_LOG_MODULE_NAME "TWI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

void ic_twi_init(const nrf_drv_twi_t *const instance,
    nrf_drv_twi_evt_handler_t event, void *context){

  nrf_drv_twi_config_t _twi_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  _twi_config.sda                 = IC_TWI_SDA_PIN;
  _twi_config.scl                 = IC_TWI_SCL_PIN;
  _twi_config.interrupt_priority  = IC_TWI_IRQ_PRIORITY;
  _twi_config.frequency           = (nrf_twi_frequency_t)IC_TWI_FREQUENCY;
  _twi_config.clear_bus_init      = true;
  _twi_config.hold_bus_uninit     = true;

  APP_ERROR_CHECK(nrf_drv_twi_init(instance, &_twi_config, event, context));
}
