/**
 * @file    ic_bluetooth.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    May, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_BLUETOOTH_H
#define IC_BLUETOOTH_H

#include <stdint.h>
#include <stdbool.h>

#include "ic_common_types.h"

bool rc_start_advertising();
ic_return_val_e ic_ble_module_init(void);
ic_return_val_e ic_bluetooth_enable(void);
ic_return_val_e ic_bluetooth_disable(void);

#endif /* !IC_BLUETOOTH_H */
