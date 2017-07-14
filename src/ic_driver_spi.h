/**
 * @file    ic_driver_spi.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_SPI_H
#define IC_DRIVER_SPI_H

#include "nrf_drv_spi.h"

#define SPI_REGISTER(name, ss_pin, code)\
  static const nrf_drv_spi_t name##_spi_instance = NRF_DRV_SPI_INSTANCE(IC_SPI_INSTANCE);\
  static const uint8_t name##_ss_pin = ss_pin;\
  void name##_spi_IRQ_handle(nrf_drv_spi_evt_t const * p_event){UNUSED_PARAMETER(p_event); code;}

#define SPI_INIT(name)\
  ic_spi_init(name##_spi_instance, name##_ss_pin, name##_spi_IRQ_handle)

#define SPI_SEND_DATA(name, in_buffer, out_buffer, len)\
  nrf_drv_spi_transfer(&name##_spi_instance,\
      (uint8_t *)in_buffer, (uint8_t)len, (uint8_t *)out_buffer, (uint8_t)len);

#define SPI_UNINIT(name)\
  nrf_drv_spi_uninit(&name##_spi_instance);

void ic_spi_init(const nrf_drv_spi_t *const instance, uint8_t pin, void(*callback)());

#endif /* !IC_DRIVER_SPI_H */
