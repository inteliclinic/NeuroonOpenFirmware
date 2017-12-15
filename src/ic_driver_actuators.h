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
  ACTUATOR_LEFT_GREEN_LED = 0x00,
  ACTUATOR_LEFT_RED_LED,
  ACTUATOR_LEFT_BLUE_LED,
  ACTUATOR_RIGHT_GREEN_LED,
  ACTUATOR_RIGHT_RED_LED,
  ACTUATOR_RIGHT_BLUE_LED,
  ACTUATOR_VIBRATOR,
  ACTUATOR_POWER_LEDS
}ic_actuator_e;

/**
 * @brief 
 *
 * @return 
 */
ic_return_val_e ic_actuator_init(void);

/**
 * @brief 
 *
 * @param device
 * @param val
 * @param fp
 *
 * @return 
 */
ic_return_val_e ic_actuator_set(ic_actuator_e device, uint8_t val, void(*fp)(bool));

/**
 * @brief 
 *
 * @param device
 *
 * @return 
 */
uint8_t ic_actuator_get_current_val(ic_actuator_e device);

#endif /* !IC_DRIVER_ACTUATORS_H */
