/**
 * @file    ic_service_ltc.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_SERVICE_LTC_H
#define IC_SERVICE_LTC_H

#include <stdint.h>
#include "ic_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 
 */
typedef enum{
  IC_LEFT_GREEN_LED = 0x00,
  IC_LEFT_RED_LED,
  IC_LEFT_BLUE_LED,
  IC_RIGHT_GREEN_LED,
  IC_RIGHT_RED_LED,
  IC_RIGHT_BLUE_LED,
  IC_VIBRATOR,
  IC_POWER_LEDS
}ic_devices_e;

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_off_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_on_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_sin_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_blink_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_square_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_saw_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_triangle_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

/**
 * @brief 
 *
 * @param device
 * @param period
 * @param duration
 * @param intensity
 *
 * @return 
 */
ic_return_val_e ic_actuator_set_ramp_func(
    ic_devices_e device,
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);


/**
 * @brief 
 *
 * @return 
 */
ic_return_val_e ic_ltc_service_init();

/**
 * @brief 
 *
 * @return 
 */
ic_return_val_e ic_ltc_service_deinit();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !IC_SERVICE_LTC_H */
