/*
 * ic_serial.h
 * Copyright (C) 2019 Paweł Kaźmierzewski <pawel.kazmierzewski@gmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef IC_SERIAL_H
#define IC_SERIAL_H

#define IC_SERIAL_MANUFACTURER  0
#define IC_SERIAL_DAY           1
#define IC_SERIAL_MONTH         2
#define IC_SERIAL_SN            3
#define IC_SERIAL_YEAR          4

#include <stdint.h>
#include <stddef.h>

extern volatile unsigned int __start_serial_number[];
uint8_t crc6_calculate(const uint8_t *data, size_t len);

#endif /* !IC_SERIAL_H */
