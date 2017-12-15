/**
 * @file    ic_nrf_error.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_nrf_error.h"
#include "nrf_error.h"
#include "ble.h"

const char * ic_get_nrferr2str(uint32_t err){
  switch(err){
    case NRF_SUCCESS:
      return "NRF_SUCCESS";
    case NRF_ERROR_SVC_HANDLER_MISSING:
      return "NRF_ERROR_SVC_HANDLER_MISSING";
    case NRF_ERROR_SOFTDEVICE_NOT_ENABLED:
      return "NRF_ERROR_SOFTDEVICE_NOT_ENABLED";
    case NRF_ERROR_INTERNAL:
      return "NRF_ERROR_INTERNAL";
    case NRF_ERROR_NO_MEM:
      return "NRF_ERROR_NO_MEM";
    case NRF_ERROR_NOT_FOUND:
      return "NRF_ERROR_NOT_FOUND";
    case NRF_ERROR_NOT_SUPPORTED:
      return "NRF_ERROR_NOT_SUPPORTED";
    case NRF_ERROR_INVALID_PARAM:
      return "NRF_ERROR_INVALID_PARAM";
    case NRF_ERROR_INVALID_STATE:
      return "NRF_ERROR_INVALID_STATE";
    case NRF_ERROR_INVALID_LENGTH:
      return "NRF_ERROR_INVALID_LENGTH";
    case NRF_ERROR_INVALID_FLAGS:
      return "NRF_ERROR_INVALID_FLAGS";
    case NRF_ERROR_INVALID_DATA:
      return "NRF_ERROR_INVALID_DATA";
    case NRF_ERROR_DATA_SIZE:
      return "NRF_ERROR_DATA_SIZE";
    case NRF_ERROR_TIMEOUT:
      return "NRF_ERROR_TIMEOUT";
    case NRF_ERROR_NULL:
      return "NRF_ERROR_NULL";
    case NRF_ERROR_FORBIDDEN:
      return "NRF_ERROR_FORBIDDEN";
    case NRF_ERROR_INVALID_ADDR:
      return "NRF_ERROR_INVALID_ADDR";
    case NRF_ERROR_BUSY:
      return "NRF_ERROR_BUSY";
    default:
      return "NRF_ERROR_UNKNOWN";
  }
}

const char * ic_get_bleevt2str(uint32_t evt){
  switch(evt){
    case BLE_EVT_TX_COMPLETE:
      return "BLE_EVT_TX_COMPLETE";
    case BLE_EVT_USER_MEM_REQUEST:
      return "BLE_EVT_USER_MEM_REQUEST";
    case BLE_EVT_USER_MEM_RELEASE:
      return "BLE_EVT_USER_MEM_RELEASE";
    case BLE_GAP_EVT_CONNECTED:
      return "BLE_GAP_EVT_CONNECTED";
    case BLE_GAP_EVT_DISCONNECTED:
      return "BLE_GAP_EVT_DISCONNECTED";
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
      return "BLE_GAP_EVT_CONN_PARAM_UPDATE";
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      return "BLE_GAP_EVT_SEC_PARAMS_REQUEST";
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
      return "BLE_GAP_EVT_SEC_INFO_REQUEST";
    case BLE_GAP_EVT_PASSKEY_DISPLAY:
      return "BLE_GAP_EVT_PASSKEY_DISPLAY";
    case BLE_GAP_EVT_KEY_PRESSED:
      return "BLE_GAP_EVT_KEY_PRESSED";
    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
      return "BLE_GAP_EVT_AUTH_KEY_REQUEST";
    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
      return "BLE_GAP_EVT_LESC_DHKEY_REQUEST";
    case BLE_GAP_EVT_AUTH_STATUS:
      return "BLE_GAP_EVT_AUTH_STATUS";
    case BLE_GAP_EVT_CONN_SEC_UPDATE:
      return "BLE_GAP_EVT_CONN_SEC_UPDATE";
    case BLE_GAP_EVT_TIMEOUT:
      return "BLE_GAP_EVT_TIMEOUT";
    case BLE_GAP_EVT_RSSI_CHANGED:
      return "BLE_GAP_EVT_RSSI_CHANGED";
    case BLE_GAP_EVT_ADV_REPORT:
      return "BLE_GAP_EVT_ADV_REPORT";
    case BLE_GAP_EVT_SEC_REQUEST:
      return "BLE_GAP_EVT_SEC_REQUEST";
    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
      return "BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST";
    case BLE_GAP_EVT_SCAN_REQ_REPORT:
      return "BLE_GAP_EVT_SCAN_REQ_REPORT";
    case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
      return "BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP";
    case BLE_GATTC_EVT_REL_DISC_RSP:
      return "BLE_GATTC_EVT_REL_DISC_RSP";
    case BLE_GATTC_EVT_CHAR_DISC_RSP:
      return "BLE_GATTC_EVT_CHAR_DISC_RSP";
    case BLE_GATTC_EVT_DESC_DISC_RSP:
      return "BLE_GATTC_EVT_DESC_DISC_RSP";
    case BLE_GATTC_EVT_ATTR_INFO_DISC_RSP:
      return "BLE_GATTC_EVT_ATTR_INFO_DISC_RSP";
    case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
      return "BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP";
    case BLE_GATTC_EVT_READ_RSP:
      return "BLE_GATTC_EVT_READ_RSP";
    case BLE_GATTC_EVT_CHAR_VALS_READ_RSP:
      return "BLE_GATTC_EVT_CHAR_VALS_READ_RSP";
    case BLE_GATTC_EVT_WRITE_RSP:
      return "BLE_GATTC_EVT_WRITE_RSP";
    case BLE_GATTC_EVT_HVX:
      return "BLE_GATTC_EVT_HVX";
    case BLE_GATTC_EVT_TIMEOUT:
      return "BLE_GATTC_EVT_TIMEOUT";
    case BLE_GATTS_EVT_WRITE:
      return "BLE_GATTS_EVT_WRITE";
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
      return "BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST";
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      return "BLE_GATTS_EVT_SYS_ATTR_MISSING";
    case BLE_GATTS_EVT_HVC:
      return "BLE_GATTS_EVT_HVC";
    case BLE_GATTS_EVT_SC_CONFIRM:
      return "BLE_GATTS_EVT_SC_CONFIRM";
    case BLE_GATTS_EVT_TIMEOUT:
      return "BLE_GATTS_EVT_TIMEOUT";
    case BLE_L2CAP_EVT_RX:
      return "BLE_L2CAP_EVT_RX";
    default:
      return "BLE_UNKNOWN";
  }
}
