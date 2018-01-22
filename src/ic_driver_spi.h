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
#include <stdbool.h>

#include "ic_common_types.h"

typedef void (*ic_spi_event_cb)(void *context);

typedef struct{
  void *nrf_spi_instance;
  ic_spi_event_cb callback;
  void *context;
  uint8_t pin;
  bool active;                        /** Is line ocupated by device */
  struct{
    size_t in_len;
    uint8_t *in_buffer;
    size_t out_len;
    uint8_t *out_buffer;
    bool open;
  }transaction_desc;
}ic_spi_instance_s;

ic_return_val_e ic_spi_init(ic_spi_instance_s *instance, uint8_t pin);

ic_return_val_e ic_spi_deinit(ic_spi_instance_s *instance);

ic_return_val_e ic_spi_send(
    ic_spi_instance_s *instance,
    uint8_t *in_buffer,
    size_t in_len,
    uint8_t *out_buffer,
    size_t out_len,
    ic_spi_event_cb callback,
    void *context,
    bool open
    );

void ic_spi_cs_high(ic_spi_instance_s *instance);

void ic_spi_cs_low(ic_spi_instance_s *instance);

bool ic_spi_instance_busy(ic_spi_instance_s *instance);

#define SPI_REGISTER(name)\
  static ic_spi_instance_s name##_spi_instance;

#define SPI_INIT(name, ss_pin)\
  ic_spi_init(&name##_spi_instance, ss_pin)

#define SPI_SEND_DATA(name, in_buffer, out_buffer, len, callback, context)\
  ic_spi_send(&name##_spi_instance, (uint8_t *)in_buffer, (uint8_t)len, (uint8_t *)out_buffer,\
      (uint8_t)len, callback, context, false)

#define SPI_SEND_DATA_OPEN(name, in_buffer, out_buffer, len, callback, context)\
  ic_spi_send(&name##_spi_instance, (uint8_t *)in_buffer, (uint8_t)len, (uint8_t *)out_buffer,\
      (uint8_t)len, callback, context, true)

#define SPI_CS_HIGH(name) ic_spi_cs_high(&name##_spi_instance)

#define SPI_CS_LOW(name) ic_spi_cs_low(&name##_spi_instance)

#define SPI_TRANSACTION_ACTIVE(name) ic_spi_instance_busy(&name##_spi_instance)

#define SPI_UNINIT(name) ic_spi_deinit(&name##_spi_instance)

#endif /* !IC_DRIVER_SPI_H */
