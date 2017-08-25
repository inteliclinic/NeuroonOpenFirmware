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
#include <stdint.h>
#include <stddef.h>
#include <stdint.h>

#include "ic_common_types.h"

typedef void (*ic_twi_event_cb)(void *context);

typedef struct{
  void *nrf_twi_instance;
  ic_twi_event_cb callback;
}ic_twi_instance_s;

#define TWI_REGISTER(name)                                                        \
  static ic_twi_instance_s name##_twi_instance;                                   \

#define TWI_INIT(name)                                                            \
  ic_twi_init(&name##_twi_instance);

#define TWI_SEND_DATA(name, address, in_buffer, len, callback)                    \
  ic_twi_send(&name##_twi_instance, address, in_buffer, len, callback)

#define TWI_READ_DATA(name, address, reg_addr, in_buffer, len, callback)          \
  ic_twi_read(&name##_twi_instance, address, reg_addr, in_buffer, len, callback)

ic_return_val_e ic_twi_init(ic_twi_instance_s * instance);

ic_return_val_e ic_twi_send(const ic_twi_instance_s *const instance, uint8_t address,
    uint8_t *in_buffer, size_t len, ic_twi_event_cb callback);

ic_return_val_e ic_twi_read(const ic_twi_instance_s *const instance, uint8_t address,
    uint8_t reg_addr, uint8_t *in_buffer, size_t len, ic_twi_event_cb callback);

#endif /* !IC_DRIVER_TWI_H */
