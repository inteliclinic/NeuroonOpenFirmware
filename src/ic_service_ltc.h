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

#include "ic_config.h"

ic_return_val_e ic_actuator_lgl_func_triangle(
    uint32_t period,
    uint32_t duration,
    uint8_t intensity);

ic_return_val_e ic_ltc_service_init();


#endif /* !IC_SERVICE_LTC_H */
