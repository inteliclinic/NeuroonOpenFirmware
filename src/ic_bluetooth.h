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

void ble_module_init (void);
bool rc_start_advertising();

#endif /* !IC_BLUETOOTH_H */
