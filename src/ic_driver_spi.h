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
#include <stdint.h>
#include <stddef.h>

#include "ic_common_types.h"

typedef void (*ic_spi_event_cb)(void *context);

typedef struct{
  void *nrf_spi_instance;
  ic_spi_event_cb callback;
  uint8_t pin;
}ic_spi_instance_s;

ic_return_val_e ic_spi_init(ic_spi_instance_s *instance, uint8_t pin);

ic_return_val_e ic_spi_send(const ic_spi_instance_s *const instance, uint8_t *in_buffer,
    size_t in_len, uint8_t *out_buffer, uint8_t out_len, ic_spi_event_cb callback);

void ic_spi_cs_high(ic_spi_instance_s *instance);

void ic_spi_cs_low(ic_spi_instance_s *instance);

#define SPI_REGISTER(name)\
  static nrf_drv_spi_t name##_spi_instance = NRF_DRV_SPI_INSTANCE(IC_SPI_INSTANCE)

#define SPI_INIT(name, ss_pin)\
  ic_spi_init(&name##_spi_instance, ss_pin)

#define SPI_SEND_DATA(name, in_buffer, out_buffer, len, callback)\
  ic_spi_send(&name##_spi_instance, (uint8_t *)in_buffer, (uint8_t)len, (uint8_t *)out_buffer,\
      (uint8_t)len, callback)

#define SPI_CS_HIGH(name) ic_spi_cs_high(&name##_spi_instance);

#define SPI_CS_LOW(name) ic_spi_cs_low(&name##_spi_instance);

#define SPI_UNINIT(name)

#endif /* !IC_DRIVER_SPI_H */
