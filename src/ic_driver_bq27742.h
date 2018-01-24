/**
 * @file    ic_driver_bq27742.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    January, 2018
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_BQ27742_H
#define IC_DRIVER_BQ27742_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ic_driver_bq27742_definitions.h"

void ic_bq_flash_image();
void ic_bq_reset();

uint16_t ic_bq_getChargeLevel(void);
en_chargerState ic_bq_getChargerState(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !IC_DRIVER_BQ27742_H */
