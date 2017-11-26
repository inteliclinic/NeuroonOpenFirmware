/**
 * @file    ic_driver_actuators.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_ACTUATORS_H
#define IC_DRIVER_ACTUATORS_H

#include "ic_common_types.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum{
  LEFT_GREEN_LED = 0x00,
  LEFT_RED_LED,
  LEFT_BLUE_LED,
  RIGHT_GREEN_LED,
  RIGHT_RED_LED,
  RIGHT_BLUE_LED,
  VIBRATOR,
  POWER_LEDS
}e_device;

ic_return_val_e ic_actuator_init(void);
ic_return_val_e ic_actuator_set(e_device device, uint8_t val, void(*fp)(bool));

#endif /* !IC_DRIVER_ACTUATORS_H */
