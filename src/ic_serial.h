/*
 * ic_serial.h
 * Copyright (C) 2019 Paweł Kaźmierzewski <pawel.kazmierzewski@gmail.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef IC_SERIAL_H
#define IC_SERIAL_H

#define IC_SERIAL_YEAR_POS  0
#define IC_SERIAL_BATCH_POS 1
#define IC_SERIAL_SN_POS    2

extern volatile unsigned int __start_serial_number[];

#endif /* !IC_SERIAL_H */
