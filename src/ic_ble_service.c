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

#ifndef IC_CHAR_MAX_LEN
#define IC_CHAR_MAX_LEN 20
#endif //CHAR_MAX_LEN

#define LAMBDA(c_) ({ c_ _;}) // TODO: only for quick debug

enum char_dir_e{
  CHAR_READ_ENABLE    = 0x01,
  CHAR_WRITE_ENABLE   = 0x02,
  CHAR_NOTIFY_ENABLE  = 0x04
}char_dir;

#define STREAM0 0
#define STREAM1 1
#define STREAM2 2
#define CMD_CHAR 3

static struct {
  uint8_t uuid_type;
}m_service_desc;

typedef struct {
  ble_gatts_char_handles_t char_handle;
  uint16_t uuid;
  uint8_t read_write_notify;
  union{
    void *p_func;
    void (*readiness_notify_handle)(bool);
    void (*write_handle)(uint8_t *, size_t);
  }char_callback;
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
    .char_callback = {NULL},
    .read_write_notify = CHAR_READ_ENABLE|CHAR_NOTIFY_ENABLE
  },
  {
    .uuid = BLE_UUID_ICCS_STREAM1_CHARACTERISTIC,
    .char_callback = {NULL},
    .read_write_notify = CHAR_READ_ENABLE|CHAR_NOTIFY_ENABLE
  },
  {
    .uuid = BLE_UUID_ICCS_STREAM2_CHARACTERISTIC,
    .char_callback = {NULL},
    .read_write_notify = CHAR_READ_ENABLE|CHAR_NOTIFY_ENABLE
  },
  {
    .uuid = BLE_UUID_ICCS_CMD_CHARACTERISTIC,
    .char_callback = {NULL},
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
static uint32_t char_add(
    uint16_t                  uuid,
    uint8_t                   *p_char_value,
    uint16_t                  char_len,
    ble_gatts_char_handles_t  *p_handles,
    uint8_t                   read_write)
{
  ble_uuid_t          _ble_uuid;
  ble_gatts_char_md_t _char_md;
  ble_gatts_attr_t    _attr_char_value;
  ble_gatts_attr_md_t _attr_md;

  APP_ERROR_CHECK_BOOL(char_len > 0);

  // The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
  memset(&_char_md, 0, sizeof(_char_md));

  _char_md.char_props.read   = (read_write&CHAR_READ_ENABLE)>0;
  _char_md.char_props.notify = (read_write&CHAR_NOTIFY_ENABLE)>0;
  _char_md.char_props.write  = (read_write&CHAR_WRITE_ENABLE)>0;

  _char_md.p_char_user_desc = NULL;
  _char_md.p_char_pf        = NULL;
  _char_md.p_user_desc_md   = NULL;
  _char_md.p_cccd_md        = NULL; //&_cccd_md;
  _char_md.p_sccd_md        = NULL;

  /*BLE_UUID_BLE_ASSIGN(_ble_uuid, uuid);*/
  _ble_uuid.type = m_service_desc.uuid_type;
  _ble_uuid.uuid = uuid;
  memset(&_attr_md, 0, sizeof(_attr_md));

  if(_char_md.char_props.write) BLE_GAP_CONN_SEC_MODE_SET_OPEN(&_attr_md.write_perm);
  if(_char_md.char_props.read)  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&_attr_md.read_perm);

  _attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  _attr_md.rd_auth    = 0;
  _attr_md.wr_auth    = 0;
  _attr_md.vlen       = 1;

  memset(&_attr_char_value, 0, sizeof(_attr_char_value));

  _attr_char_value.p_uuid    = &_ble_uuid;
  _attr_char_value.p_attr_md = &_attr_md;
  _attr_char_value.init_len  = 0;
  _attr_char_value.init_offs = 0;
  _attr_char_value.max_len   = char_len;

  __auto_type _ret_val = sd_ble_gatts_characteristic_add(m_service_handle, &_char_md, &_attr_char_value, p_handles);

  return _ret_val;
}


uint32_t ble_iccs_init(const ble_iccs_init_t *iccs_init){
  ble_uuid128_t _ble_uuid128 = {.uuid128 = BLE_UUID_ICCS_SERVICE};
  ble_uuid_t _ble_uuid;

  __auto_type _err_code = sd_ble_uuid_vs_add(&_ble_uuid128, &m_service_desc.uuid_type);
  if(_err_code != NRF_SUCCESS) return _err_code;

  _ble_uuid.type = m_service_desc.uuid_type;
  _ble_uuid.uuid = BLE_UUID_ICCS_CHARACTERISTIC;

  _err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &_ble_uuid, &m_service_handle);
  if(_err_code != NRF_SUCCESS) return _err_code;

  for(int i=0; i<sizeof(m_char_stream_list)/sizeof(m_char_stream_list[0]); ++i){
    _err_code = char_add(
        m_char_stream_list[i].uuid,
        NULL,
        IC_CHAR_MAX_LEN,
        &m_char_stream_list[i].char_handle,
        m_char_stream_list[i].read_write_notify);
  }

  /*
   *ble_iccs_connect_to_cmd(LAMBDA(void _(uint8_t *b, size_t s){
   *        NRF_LOG_INFO("data:%s\n", (uint32_t)b);
   *        NRF_LOG_INFO("size:%d\n", s);
   *        }));
   */

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
    uint16_t hvx_len = len>IC_CHAR_MAX_LEN ? IC_CHAR_MAX_LEN : len;
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = characteristic_handle->char_handle.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = (uint8_t *)data;

    __auto_type _sd_err = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);

    if (_sd_err != NRF_SUCCESS){
      switch(_sd_err){
        case BLE_ERROR_INVALID_CONN_HANDLE:
        case NRF_ERROR_INVALID_STATE:
        case NRF_ERROR_INVALID_PARAM:
        case BLE_ERROR_INVALID_ATTR_HANDLE:
        case BLE_ERROR_GATTS_INVALID_ATTR_TYPE:
        case NRF_ERROR_NOT_FOUND:
        case NRF_ERROR_DATA_SIZE:
        case BLE_ERROR_GATTS_SYS_ATTR_MISSING:
          err_code = IC_ERROR;
          break;
        case NRF_ERROR_BUSY:
        case BLE_ERROR_NO_TX_PACKETS:
          err_code = IC_BUSY;
          break;
        default:
          err_code = IC_UNKNOWN_ERROR;
          break;
      }
    }
  }
  else err_code = IC_BLE_NOT_CONNECTED;

  return err_code;
}

static ic_return_val_e ble_iccs_connect_to_char(
    void *p_func,
    characteritic_desc_t *char_desc)
{
  char_desc->char_callback.p_func = p_func;
  return p_func != NULL ? IC_SUCCESS : IC_ERROR;
}

ic_return_val_e ble_iccs_connect_to_stream0(void (*p_func)(bool)){
  return ble_iccs_connect_to_char(p_func, &m_char_stream_list[STREAM0]);
}

ic_return_val_e ble_iccs_connect_to_stream1(void (*p_func)(bool)){
  return ble_iccs_connect_to_char(p_func, &m_char_stream_list[STREAM1]);
}

ic_return_val_e ble_iccs_connect_to_stream2(void (*p_func)(bool)){
  return ble_iccs_connect_to_char(p_func, &m_char_stream_list[STREAM2]);
}

ic_return_val_e ble_iccs_connect_to_cmd(void (*p_func)(uint8_t *, size_t)){
  return ble_iccs_connect_to_char(p_func, &m_char_stream_list[CMD_CHAR]);
}

ic_return_val_e ble_iccs_send_to_stream0(
    const uint8_t *data,
    size_t len,
    uint32_t *err){
  return ble_iccs_send_to_char(data, len, &m_char_stream_list[STREAM0], err);
}

ic_return_val_e ble_iccs_send_to_stream1(
    const uint8_t *data,
    size_t len,
    uint32_t *err){
  return ble_iccs_send_to_char(data, len, &m_char_stream_list[STREAM1], err);
}

ic_return_val_e ble_iccs_send_to_stream2(
    const uint8_t *data,
    size_t len,
    uint32_t *err){
  return ble_iccs_send_to_char(data, len, &m_char_stream_list[STREAM2], err);
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
  for(int i = 0; i<sizeof(m_char_stream_list)/sizeof(m_char_stream_list[0]); ++i){
    if(m_char_stream_list[i].char_callback.readiness_notify_handle != NULL)
      m_char_stream_list[i].char_callback.readiness_notify_handle(false);
  }
}

static void on_write(ble_gatts_evt_write_t * p_write_evt){
  for(int i = 0; i<sizeof(m_char_stream_list)/sizeof(m_char_stream_list[0]); ++i){
    if(m_char_stream_list[i].char_handle.cccd_handle == p_write_evt->handle){
      __auto_type _notify_enabled = p_write_evt->data[0] == 0x01;
      m_char_stream_list[i].notification_connected = _notify_enabled;
      if(m_char_stream_list[i].char_callback.readiness_notify_handle != NULL)
        m_char_stream_list[i].char_callback.readiness_notify_handle(_notify_enabled);
      break;
    }
    if(m_char_stream_list[i].char_handle.value_handle == p_write_evt->handle){
      if(m_char_stream_list[i].char_callback.write_handle != NULL){
        m_char_stream_list[i].char_callback.write_handle(p_write_evt->data, p_write_evt->len);
      }
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
