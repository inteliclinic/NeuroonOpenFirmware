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

typedef struct{
  uint32_t first_conn_params_update_delay;
  uint32_t next_conn_params_update_delay;
}sRcInitInput;

bool rc_master_init(sRcInitInput args);
bool rc_start_advertising();

#endif /* !IC_BLUETOOTH_H */
