/**
 * @file    ic_ble_service.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    September, 2017
 * @brief   Brief description
 *
 * Description
 */

#include <string.h>
#include "ic_ble_service.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "app_error.h"

#ifndef CHAR_MAX_LEN
#define CHAR_MAX_LEN 20
#endif //CHAR_MAX_LEN

#define BLE_UUID_ICCS_SERVICE {0x86, 0x08, 0x1C, 0xB8, 0x1C, 0xA1, 0x0C, 0x84, 0xD3, 0xE2, 0x7F, 0xD9, 0x00, 0x00, 0x9E, 0xD0}

#define BLE_UUID_ICCS_STREAM0_CHARACTERISTIC     0x0201
#define BLE_UUID_ICCS_STREAM1_CHARACTERISTIC     0x0202
#define BLE_UUID_ICCS_STREAM2_CHARACTERISTIC     0x0301

#define UUID_RESPONSE_TX_CHARACTERISTIC       0x0302
#define UUID_TEST_TOOL_TX_CHARACTERISTIC      0x0401
#define UUID_TEST_TOOL_RX_CHARACTERISTIC      0x0402
#define UUID_CMD_RX_CHARACTERISTIC            0x0501

static uint16_t m_service_handle;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

static ble_gatts_char_handles_t m_stream0_char_handle;
static ble_gatts_char_handles_t m_stream1_char_handle;
static ble_gatts_char_handles_t m_stream2_char_handle;


/**@brief Function for adding the Characteristic.
 *
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[in]   dis_attr_md    Security settings of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t char_add(uint16_t                        uuid,
                         uint8_t                       * p_char_value,
                         uint16_t                        char_len,
                         const ble_srv_security_mode_t * iccs_attr_md,
                         ble_gatts_char_handles_t      * p_handles)
{
    ble_uuid_t          _ble_uuid;
    ble_gatts_char_md_t _char_md;
    ble_gatts_attr_t    _attr_char_value;
    ble_gatts_attr_md_t _attr_md;

    /*APP_ERROR_CHECK_BOOL(p_char_value != NULL);*/
    APP_ERROR_CHECK_BOOL(char_len > 0);

    // The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
    memset(&_char_md, 0, sizeof(_char_md));

    _char_md.char_props.read   = 1;
    _char_md.char_props.notify = 1;
    /*
     *char_md.char_props.write  = 1;
     *char_md.char_props.notify = 1;
     */

    _char_md.p_char_user_desc = NULL;
    _char_md.p_char_pf        = NULL;
    _char_md.p_user_desc_md   = NULL;
    _char_md.p_cccd_md        = NULL;
    _char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(_ble_uuid, uuid);

    memset(&_attr_md, 0, sizeof(_attr_md));

    _attr_md.read_perm  = iccs_attr_md->read_perm;
    _attr_md.write_perm = iccs_attr_md->write_perm;
    _attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    _attr_md.rd_auth    = 0;
    _attr_md.wr_auth    = 0;
    _attr_md.vlen       = 0;

    memset(&_attr_char_value, 0, sizeof(_attr_char_value));

    _attr_char_value.p_uuid    = &_ble_uuid;
    _attr_char_value.p_attr_md = &_attr_md;
    _attr_char_value.init_len  = 0;
    _attr_char_value.init_offs = 0;
    _attr_char_value.max_len   = char_len;
    _attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(m_service_handle, &_char_md, &_attr_char_value, p_handles);
}

uint32_t ble_iccs_init(const ble_iccs_init_t *iccs_init){
  ble_uuid128_t _ble_uuid128 = {.uuid128 = BLE_UUID_ICCS_SERVICE};
  ble_uuid_t _ble_uuid;
  uint8_t _uuid_type;

  __auto_type _err_code = sd_ble_uuid_vs_add(&_ble_uuid128, &_uuid_type);
  if(_err_code != NRF_SUCCESS) return _err_code;

  _ble_uuid.type = _uuid_type;
  _ble_uuid.uuid = BLE_UUID_ICCS_STREAM0_CHARACTERISTIC;

  _err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &_ble_uuid, &m_service_handle);
  if(_err_code != NRF_SUCCESS) return _err_code;

  ble_srv_security_mode_t _iccs_attr_md;
  memset(&_iccs_attr_md, 0, sizeof(_iccs_attr_md));

  _iccs_attr_md.read_perm.lv = 1;
  _iccs_attr_md.read_perm.sm = 1;

  _iccs_attr_md.write_perm.lv = 1;
  _iccs_attr_md.write_perm.sm = 1;

  _err_code = char_add(
      BLE_UUID_ICCS_STREAM0_CHARACTERISTIC,
      NULL,
      CHAR_MAX_LEN,
      &_iccs_attr_md,
      &m_stream0_char_handle);

  _err_code = char_add(
      BLE_UUID_ICCS_STREAM1_CHARACTERISTIC,
      NULL,
      CHAR_MAX_LEN,
      &_iccs_attr_md,
      &m_stream1_char_handle);

  _err_code = char_add(
      BLE_UUID_ICCS_STREAM2_CHARACTERISTIC,
      NULL,
      CHAR_MAX_LEN,
      &_iccs_attr_md,
      &m_stream2_char_handle);

  return NRF_SUCCESS;
}

static uint32_t ble_iccs_send_to_char(
    const uint8_t *data,
    size_t len,
    ble_gatts_char_handles_t *characteristic_handle)
{
  uint32_t err_code;

  // Send value if connected and notifying
  if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
    uint16_t hvx_len = len>CHAR_MAX_LEN ? CHAR_MAX_LEN : len;
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = characteristic_handle->value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = (uint8_t *)data;

    err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
    {
      err_code = NRF_ERROR_DATA_SIZE;
    }
  }
  else err_code = NRF_ERROR_INVALID_STATE;

  return err_code;
}

uint32_t ble_iccs_send_to_stream0(const uint8_t *data, size_t len){
  return ble_iccs_send_to_char(data, len, &m_stream0_char_handle);
}

uint32_t ble_iccs_send_to_stream1(const uint8_t *data, size_t len){
  return ble_iccs_send_to_char(data, len, &m_stream1_char_handle);
}

uint32_t ble_iccs_send_to_stream2(const uint8_t *data, size_t len){
  return ble_iccs_send_to_char(data, len, &m_stream2_char_handle);
}

static void on_connect(ble_evt_t *p_ble_evt){
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_evt_t * p_ble_evt){
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_iccs_on_ble_evt(ble_evt_t * p_ble_evt)
{
  UNUSED_PARAMETER(p_ble_evt);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            /*on_write(p_ble_evt);*/
            break;

        default:
            // No implementation needed.
            break;
    }
}
