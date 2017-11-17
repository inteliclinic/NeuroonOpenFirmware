/**
 * @file    ic_easy_ltc_driver.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_EASY_LTC_DRIVER_H
#define IC_EASY_LTC_DRIVER_H

#include "ic_config.h"
#include "ble.h"

ic_return_val_e ic_ez_ltc_module_init(void);
void ic_ez_ltc_module_deinit(void);

void ic_ez_ltc_glow();
void ic_ez_ltc_fade(void(*fp)(void));
void ic_ez_ltc_brighten(void(*fp)(void));

void ic_ez_ltc_on_ble_evt(ble_evt_t * p_ble_evt);

#endif /* !IC_EASY_LTC_DRIVER_H */
