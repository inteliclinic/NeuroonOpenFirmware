/**
 * @file    ic_nrf_error.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_NRF_ERROR_H
#define IC_NRF_ERROR_H

#include <stdint.h>

const char * ic_get_nrferr2str(uint32_t err);
const char * ic_get_bleevt2str(uint32_t evt);

#endif /* !IC_NRF_ERROR_H */
