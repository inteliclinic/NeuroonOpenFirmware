/**
 * @file    ic_service_bas.h
 * @author  Wojtek Węclewski <w.weclewski@inteliclinic.com>
 * @date    January, 2018
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_SERVICE_BAS_H
#define IC_SERVICE_BAS_H

#include "ble.h"

void ble_icbas_init(void);
void ble_icbas_on_ble_evt(ble_evt_t *);

#endif /* !IC_SERVICE_BAS_H */
