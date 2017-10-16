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

#define NRF_LOG_MODULE_NAME "ICCS"
#define NRF_LOG_LEVEL 5
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_config.h"

#ifndef CHAR_MAX_LEN
#define CHAR_MAX_LEN 20
#endif //CHAR_MAX_LEN

enum char_dir_e{
  CHAR_READ_ENABLE    = 0x01,
  CHAR_WRITE_ENABLE   = 0x02,
  CHAR_NOTIFY_ENABLE  = 0x04
}char_dir;

#define BLE_UUID_ICCS_SERVICE {0x86, 0x08, 0x1C, 0xB8, 0x1C, 0xA1, 0x0C, 0x84, 0xD3, 0xE2, 0x7F, 0xD9, 0x00, 0x00, 0x9E, 0xD0}

#define BLE_UUID_ICCS_STREAM0_CHARACTERISTIC    0x0201
#define BLE_UUID_ICCS_STREAM1_CHARACTERISTIC    0x0202
#define BLE_UUID_ICCS_STREAM2_CHARACTERISTIC    0x0301

#define BLE_UUID_ICCS_CMD_CHARACTERISTIC        0x0501

#define UUID_RESPONSE_TX_CHARACTERISTIC       0x0302
#define UUID_TEST_TOOL_TX_CHARACTERISTIC      0x0401
#define UUID_TEST_TOOL_RX_CHARACTERISTIC      0x0402
#define UUID_CMD_RX_CHARACTERISTIC            0x0501

#define STREAM0 0
#define STREAM1 1
#define STREAM2 2

typedef struct {
  ble_gatts_char_handles_t char_handle;
  uint16_t uuid;
  uint8_t read_write_notify;
  void (*readiness_notify)(bool);
  bool notification_connected;
}characteritic_desc_t;

static uint16_t m_service_handle;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

/*static characteritic_desc_t m_stream0_char_handle;*/
/*static characteritic_desc_t m_stream1_char_handle;*/
/*static characteritic_desc_t m_stream2_char_handle;*/


static characteritic_desc_t m_char_stream_list[] = {
  {
    .uuid = BLE_UUID_ICCS_STREAM0_CHARACTERISTIC,
    .readiness_notify = NULL,
    .read_write_notify = CHAR_READ_ENABLE|CHAR_NOTIFY_ENABLE
  },
  {
    .uuid = BLE_UUID_ICCS_STREAM1_CHARACTERISTIC,
    .readiness_notify = NULL,
    .read_write_notify = CHAR_READ_ENABLE|CHAR_NOTIFY_ENABLE
  },
  {
    .uuid = BLE_UUID_ICCS_STREAM2_CHARACTERISTIC,
    .readiness_notify = NULL,
    .read_write_notify = CHAR_READ_ENABLE|CHAR_NOTIFY_ENABLE
  },
  {
    .uuid = BLE_UUID_ICCS_STREAM2_CHARACTERISTIC,
    .readiness_notify = NULL,
    .read_write_notify = CHAR_WRITE_ENABLE
  }
};

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
static uint32_t char_add(uint16_t                       uuid,
                         uint8_t                        *p_char_value,
                         uint16_t                       char_len,
                         const ble_srv_security_mode_t  *iccs_attr_md,
                         ble_gatts_char_handles_t       *p_handles,
                         uint8_t                        read_write)
{
    ble_uuid_t          _ble_uuid;
    ble_gatts_char_md_t _char_md;
    ble_gatts_attr_t    _attr_char_value;
    ble_gatts_attr_md_t _attr_md;

    /*APP_ERROR_CHECK_BOOL(p_char_value != NULL);*/
    APP_ERROR_CHECK_BOOL(char_len > 0);

    // The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
    memset(&_char_md, 0, sizeof(_char_md));

    _char_md.char_props.read   = (read_write&CHAR_READ_ENABLE)>0;
    _char_md.char_props.notify = (read_write&CHAR_NOTIFY_ENABLE)>0;
    _char_md.char_props.write  = (read_write&CHAR_WRITE_ENABLE)>0;

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

    __auto_type _ret_val = sd_ble_gatts_characteristic_add(m_service_handle, &_char_md, &_attr_char_value, p_handles);

    return _ret_val;
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

  for(int i=0; i<sizeof(m_char_stream_list)/sizeof(m_char_stream_list[0]); ++i){
    _err_code = char_add(
        m_char_stream_list[i].uuid,
        NULL,
        CHAR_MAX_LEN,
        &_iccs_attr_md,
        &m_char_stream_list[i].char_handle,
        m_char_stream_list[i].read_write_notify);
  }

  return NRF_SUCCESS;
}

static ic_return_val_e ble_iccs_send_to_char(
    const uint8_t *data,
    size_t len,
    characteritic_desc_t *characteristic_handle,
    uint32_t *err)
{
  __auto_type err_code = IC_SUCCESS;

  // Send value if connected and notifying
  if (characteristic_handle->notification_connected){
    uint16_t hvx_len = len>CHAR_MAX_LEN ? CHAR_MAX_LEN : len;
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = characteristic_handle->char_handle.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = (uint8_t *)data;

    __auto_type _sd_err = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);

    if (_sd_err != NRF_SUCCESS)
      err_code = IC_ERROR;
  }
  else err_code = IC_BLE_NOT_CONNECTED;

  return err_code;
}

static ic_return_val_e ble_iccs_connect_to_stream(
    void (*p_func)(bool),
    characteritic_desc_t *char_desc){
  char_desc->readiness_notify = p_func;
  return p_func != NULL ? IC_SUCCESS : IC_ERROR;
}

ic_return_val_e ble_iccs_connect_to_stream0(void (*p_func)(bool)){
  return ble_iccs_connect_to_stream(p_func, &m_char_stream_list[STREAM0]);
}

ic_return_val_e ble_iccs_connect_to_stream1(void (*p_func)(bool)){
  return ble_iccs_connect_to_stream(p_func, &m_char_stream_list[STREAM1]);
}

ic_return_val_e ble_iccs_connect_to_stream2(void (*p_func)(bool)){
  return ble_iccs_connect_to_stream(p_func, &m_char_stream_list[STREAM2]);
}

ic_return_val_e ble_iccs_send_to_stream0(
    const uint8_t *data,
    size_t len,
    uint32_t *err){
  return ble_iccs_send_to_char(data, len, &m_char_stream_list[0], err);
}

ic_return_val_e ble_iccs_send_to_stream1(
    const uint8_t *data,
    size_t len,
    uint32_t *err){
  return ble_iccs_send_to_char(data, len, &m_char_stream_list[1], err);
}

ic_return_val_e ble_iccs_send_to_stream2(
    const uint8_t *data,
    size_t len,
    uint32_t *err){
  return ble_iccs_send_to_char(data, len, &m_char_stream_list[2], err);
}

bool ble_iccs_stream0_ready(){
  return m_char_stream_list[STREAM0].notification_connected;
}

bool ble_iccs_stream1_ready(){
  return m_char_stream_list[STREAM1].notification_connected;
}

bool ble_iccs_stream2_ready(){
  return m_char_stream_list[STREAM2].notification_connected;
}

static void on_connect(ble_evt_t *p_ble_evt){
  m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_evt_t * p_ble_evt){
  m_conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_gatts_evt_write_t * p_write_evt){
for(int i = 0; i<sizeof(m_char_stream_list)/sizeof(m_char_stream_list[0]); ++i){
    if(m_char_stream_list[i].char_handle.cccd_handle == p_write_evt->handle){
      __auto_type _notify_enabled = p_write_evt->data[0] == 0x01;
      m_char_stream_list[i].notification_connected = _notify_enabled;
      if(m_char_stream_list[i].readiness_notify != NULL) m_char_stream_list[i].readiness_notify(_notify_enabled);
      break;
    }
  }
}

void ble_iccs_on_ble_evt(ble_evt_t * p_ble_evt)
{
  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      on_connect(p_ble_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnect(p_ble_evt);
      break;

    case BLE_GAP_EVT_KEY_PRESSED:
      break;
    case BLE_GATTS_EVT_WRITE:
      on_write(&p_ble_evt->evt.gatts_evt.params.write);
      break;


    default:
      // No implementation needed.
      break;
  }
}
