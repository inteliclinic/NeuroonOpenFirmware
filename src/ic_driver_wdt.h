/**
 * @file    ic_driver_wdt.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_WDT_H
#define IC_DRIVER_WDT_H

#include "ic_common_types.h"

ic_return_val_e ic_wdt_init();
void ic_wdt_refresh();

#endif /* !IC_DRIVER_WDT_H */
