/*
 * ic_serial.c
 * Copyright (C) 2019 Paweł Kaźmierzewski <p.kazmierze2@samsung.com>
 *
 * Distributed under terms of the MIT license.
 */

#include "ic_serial.h"

uint8_t tab[] = {9,  21, 59, 5,  38, 57, 58, 14, 17, 16, 43, 45, 4,
                 31, 64, 49, 22, 24, 48, 46, 12, 56, 28, 51, 15, 20,
                 53, 11, 32, 37, 61, 34, 18, 41, 8,  10, 23, 1,  7,
                 2,  50, 35, 60, 27, 63, 62, 55, 47, 6,  42, 26, 19,
                 44, 52, 13, 33, 30, 25, 54, 40, 36, 39, 29, 3};

uint8_t crc6_calculate(const uint8_t *data, size_t len){
  uint8_t crc = 0;
  while(len--){
    crc = tab[(*data++&0x3F)^crc];
  }
  return crc;
}

