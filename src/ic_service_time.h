/**
 * @file    ic_service_time.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_SERVICE_TIME_H
#define IC_SERVICE_TIME_H

#include <stdint.h>
#include "ic_config.h"

typedef struct{
  uint64_t  unix_timestamp;
  uint32_t  sub_timer;
}ic_unix_timestamp_s;

typedef struct{
  uint32_t seconds : 22;
  uint32_t fraction: 10;
} ic_relative_timestamp_s;

void ic_unix_timestamp_set(uint64_t timestamp);
void ic_relative_timestamp_set(uint32_t timestamp);
ic_unix_timestamp_s ic_unix_timestamp_get();
ic_unix_timestamp_s ic_relative_timestamp_get();
ic_return_val_e ic_service_timestamp_init();
ic_return_val_e ic_service_timestamp_deinit();
ic_return_val_e ic_service_timestamp_destroy();

#endif /* !IC_SERVICE_TIME_H */
