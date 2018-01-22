/**
 * @file    ic_ltc_driver.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_LTC_H
#define IC_DRIVER_LTC_H

#include "ic_config.h"

ic_return_val_e ic_ltc_driver_init(void);
ic_return_val_e ic_ltc_driver_deinit(void);
ic_return_val_e ic_reset_rgb();
ic_return_val_e ic_set_channel(
    uint8_t channel,
    uint8_t val,
    void(*fp)(ic_return_val_e, void *),
    void *context);
ic_return_val_e ic_enable_channel(uint8_t channel);
ic_return_val_e ic_disable_channel(uint8_t channel);

#endif /* !IC_DRIVER_LTC_H */
