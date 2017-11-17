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
#include <stdlib.h>
#include <ble.h>
#include <stdbool.h>

#include "ic_common_types.h"

#define BLE_UUID_ICCS_SERVICE {0x86, 0x08, 0x1C, 0xB8, 0x1C, 0xA1, 0x0C, 0x84, 0xD3, 0xE2, 0x7F, 0xD9, 0x00, 0x00, 0x9E, 0xD0}

#define BLE_UUID_ICCS_CHARACTERISTIC            0x0100

#define BLE_UUID_ICCS_STREAM0_CHARACTERISTIC    0x0201
#define BLE_UUID_ICCS_STREAM1_CHARACTERISTIC    0x0202

#define BLE_UUID_ICCS_STREAM2_CHARACTERISTIC    0x0301
#define BLE_UUID_ICCS_CMD_CHARACTERISTIC        0x0501

#define UUID_RESPONSE_TX_CHARACTERISTIC       0x0302
#define UUID_TEST_TOOL_TX_CHARACTERISTIC      0x0401
#define UUID_TEST_TOOL_RX_CHARACTERISTIC      0x0402
#define UUID_CMD_RX_CHARACTERISTIC            0x0501

typedef struct{
  uint8_t dummy;
}ble_iccs_init_t;

uint32_t ble_iccs_init(const ble_iccs_init_t *iccs_init);
ic_return_val_e ble_iccs_connect_to_stream0(void (*p_func)(bool));
ic_return_val_e ble_iccs_connect_to_stream1(void (*p_func)(bool));
ic_return_val_e ble_iccs_connect_to_stream2(void (*p_func)(bool));
ic_return_val_e ble_iccs_connect_to_cmd(void (*p_func)(uint8_t *, size_t));
ic_return_val_e ble_iccs_send_to_stream0(const uint8_t *data, size_t len,uint32_t *err);
ic_return_val_e ble_iccs_send_to_stream1(const uint8_t *data, size_t len,uint32_t *err);
ic_return_val_e ble_iccs_send_to_stream2(const uint8_t *data, size_t len,uint32_t *err);
bool ble_iccs_stream0_ready();
bool ble_iccs_stream1_ready();
bool ble_iccs_stream2_ready();
void ble_iccs_on_ble_evt(ble_evt_t * p_ble_evt);

#endif /* !IC_BLE_SERVICE_H */
