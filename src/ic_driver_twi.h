/**
 * @file    ic_driver_twi.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_TWI_H
#define IC_DRIVER_TWI_H

#include "nrf_drv_twi.h"

#define TWI_REGISTER(name, code)\
  static const nrf_drv_twi_t twi_instance_##name = NRF_DRV_TWI_INSTANCE(IC_TWI_INSTANCE);\
  void twi_IRQ_handle_##name(nrf_drv_twi_evt_t const * p_event, void *p_context){UNUSED_PARAMETER(p_event); code;};\
  ic_twi_init(&twi_instance_##name, twi_IRQ_handle_##name, NULL)

#warn "Fill it!"
#define TWI_SEND_DATA(name, in_buffer, out_buffer, len)\
  nrf_drv_twi_transfer(&spi_instance_##name,\
      (uint8_t *)in_buffer, (uint8_t)len, (uint8_t *)out_buffer, (uint8_t)len);

void ic_twi_init(const nrf_drv_twi_t *const instance,
    nrf_drv_twi_evt_handler_t event, void *context);

#endif /* !IC_DRIVER_TWI_H */
