/**
 * @file    ic_driver_wdt.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_config.h"
#include "ic_driver_wdt.h"
#include "nrf_drv_wdt.h"

static nrf_drv_wdt_channel_id m_channel_id;

static bool m_wdt_inited = false;

void wdt_event_handler(void)
{
}

ic_return_val_e ic_wdt_init(){
  nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
  __auto_type err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);
  nrf_drv_wdt_enable();

  m_wdt_inited = true;

  return IC_SUCCESS;
}

void ic_wdt_refresh(){
  if(m_wdt_inited)
    nrf_drv_wdt_channel_feed(m_channel_id);
}
