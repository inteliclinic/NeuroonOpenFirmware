/**
 * @file    ic_ble_service.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    September, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_BLE_SERVICE_H
#define IC_BLE_SERVICE_H

#include <stdint.h>
#include <ble.h>

typedef struct{
  uint8_t dummy;
}ble_iccs_init_t;

uint32_t ble_iccs_init(const ble_iccs_init_t *iccs_init);
uint32_t ble_iccs_send_to_stream0(const uint8_t *data, size_t len);
uint32_t ble_iccs_send_to_stream1(const uint8_t *data, size_t len);
uint32_t ble_iccs_send_to_stream2(const uint8_t *data, size_t len);
void ble_iccs_on_ble_evt(ble_evt_t * p_ble_evt);

#endif /* !IC_BLE_SERVICE_H */
