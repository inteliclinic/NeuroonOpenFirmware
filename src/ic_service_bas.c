/**
 * @file    ic_service_bas.c
 * @author  Wojtek Węclewski <w.weclewski@inteliclinic.com>
 * @date    January, 2018
 * @brief   Brief description
 *
 * Description
 */

#include "ic_service_bas.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"
#include "ble_bas.h"

#include "ic_driver_bq27742.h"

#define NRF_LOG_MODULE_NAME "ICBAS"
#define NRF_LOG_LEVEL 5
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"

#include "ic_config.h"

ble_bas_t m_ble_bas;
static TimerHandle_t m_service_bas_timer_handle = NULL;

void ble_icbas_on_ble_evt(ble_evt_t *p_ble_evt){
  ble_bas_on_ble_evt(&m_ble_bas, p_ble_evt);
}

void icbas_evt_handler(ble_bas_t *p_bas, ble_bas_evt_t *p_evt){
  UNUSED_PARAMETER(p_bas);
  __auto_type _timer_ret_val = pdFAIL;
  switch (p_evt->evt_type){
    case BLE_BAS_EVT_NOTIFICATION_DISABLED:
      STOP_TIMER  (m_service_bas_timer_handle, 0, _timer_ret_val);
      NRF_LOG_INFO("BAS notification disconnected.\r\n");
      break; // BLE_BAS_EVT_NOTIFICATION_DISABLED

    case BLE_BAS_EVT_NOTIFICATION_ENABLED:
      ble_bas_battery_level_update(&m_ble_bas, 0);  //TODO: brak funkcji BQ
      START_TIMER (m_service_bas_timer_handle, 0, _timer_ret_val);
      NRF_LOG_INFO("BAS notification connected.\r\n");
      break; // BLE_BAS_EVT_NOTIFICATION_ENABLED
  }
  UNUSED_VARIABLE(_timer_ret_val);
}

void ble_icbas_init(void){
  ble_bas_init_t bas_init;
  memset(&bas_init, 0, sizeof(ble_bas_init_t));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

  bas_init.initial_batt_level = 100;
  bas_init.support_notification = true;
  bas_init.evt_handler = icbas_evt_handler;

  __auto_type err_code = ble_bas_init(&m_ble_bas, &bas_init);
  APP_ERROR_CHECK(err_code);

}

static void bas_timer_callback(TimerHandle_t xTimer){
  UNUSED_PARAMETER(xTimer);
  __auto_type _bat = ic_bq_getChargeLevel();
  NRF_LOG_INFO("bat: %d\n", _bat);
  if(NRF_SUCCESS != ble_bas_battery_level_update(&m_ble_bas, _bat)) //TODO: brak funkcji BQ
    NRF_LOG_ERROR("Battery level not updated\n");
}

ic_return_val_e ic_init_battery_update(void){
  if(m_service_bas_timer_handle == NULL)
    m_service_bas_timer_handle = xTimerCreate(
        "BAS",
        IC_BAS_TICK_PERIOD,
        pdTRUE,
        (void *) 0,
        bas_timer_callback);
  return IC_SUCCESS;
}

