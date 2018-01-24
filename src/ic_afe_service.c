/**
 * @file    ic_afe_service.c
 * @author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_clock.h"
#define NRF_LOG_MODULE_NAME "AFE_SERVICE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ic_config.h"

#include "ic_driver_afe4400.h"
#include "ic_afe_service.h"

#define TIMER_READ_INTERVAL     64


static TaskHandle_t m_init_thread;
#ifndef _USE_AFE_INT
  TimerHandle_t m_read_afeTimer;
#endif

static void (*m_user_cb)(s_led_val);

static s_led_val m_led_val =
{
  .ir_val  = 0,
  .red_val  = 0,
  .air_val = 0,
  .ared_val = 0,
  .ir_diff = 0,
  .red_diff = 0
};

/*********************************************************************************************************************/
void afe_led_callback(s_led_val led_val)
{
    /*  copy led values to the global variable  */
  memcpy(&m_led_val, &led_val, sizeof(m_led_val));
}
/*********************************************************************************************************************/
#ifndef _USE_AFE_INT
void afe_read_callback()
{
    /*NRF_LOG_INFO("{ %s: %p }\r\n", (uint32_t)__func__, (uint32_t)m_user_cb);*/
  if (m_user_cb != NULL)
    if (afe_read_led_reg(m_user_cb) != IC_SUCCESS)
      NRF_LOG_ERROR("AFE read led register error\r\n");
}
#endif
/*********************************************************************************************************************/
/*********************************************************************************************************************/
                          /***********************	GPIO CONFIGURATION  **************************************/
void afe_gpio_configuration(void) // TODO: what for?
{
  nrf_gpio_cfg_output(AFE4400_CS_PIN);
  nrf_gpio_cfg_output(AFE4400_PDN_PIN);
  nrf_gpio_cfg_output(AFE4400_RST_PIN);
  nrf_gpio_cfg_input(AFE4400_RDY_PIN, GPIO_PIN_CNF_PULL_Disabled);

  nrf_gpio_pin_set(AFE4400_PDN_PIN);
  nrf_gpio_pin_set(AFE4400_RST_PIN);
}
/*********************************************************************************************************************/
static void afe_gpio_deconfiguration(void)  // TODO: same
{
  nrf_gpio_cfg_default(AFE4400_CS_PIN);
  nrf_gpio_cfg_default(AFE4400_PDN_PIN);
  nrf_gpio_cfg_default(AFE4400_RST_PIN);
  nrf_gpio_cfg_default(AFE4400_RDY_PIN);
}

/*********************************************************************************************************************/
ic_return_val_e ic_afe_self_test()
{
#ifndef _USE_AFE_INT
    /*  if timer is created, stop it for self-test function  */
  if (m_read_afeTimer != NULL)
    xTimerStop(m_read_afeTimer, 0);
#endif
  /**
   * Check led current register
   */
    /*  set led current temp values  */
  uint8_t _test_led1 = 64, _test_led2 = 64;
  afe_set_led_current(_test_led1, _test_led2);
    /*  turn on led current source  */
  afe_write_bit_reg(AFE4400_LEDCNTRL, LEDCURR_OFF_BIT, LED_CURRENT_ON);
  uint32_t _check_val = 0;
    /*  read led register  */
  afe_read_reg(AFE4400_LEDCNTRL, &_check_val);
  if (!(((_check_val & 0xFF) == _test_led1) && (((_check_val >> 8) & 0xFF) == _test_led2)))
    NRF_LOG_ERROR("Error in checking LEDCNTRL register val\r\n");
    /*
     *  set different values in timing register
     *  you have to remember, that indexing in register starts from 0 !
     *  (it is necessary to subtract address value )
     */
  uint16_t _temp_timing_data[29] = {0};
  memcpy(_temp_timing_data, m_timing_data, 29);
  _temp_timing_data[AFE4400_LED2STC - 1] = 6000;
  _temp_timing_data[AFE4400_LED2ENDC - 1] = 8000;
  afe_set_timing_fast(_temp_timing_data, sizeof(m_timing_data) / sizeof(uint16_t));
  afe_read_reg(AFE4400_LED2STC, &_check_val);
  if (_check_val != 6000)
    NRF_LOG_ERROR("Error in AFE4400_LED2STC register value\r\n");
  afe_read_reg(AFE4400_LED2ENDC, &_check_val);
  if (_check_val != 8000)
    NRF_LOG_ERROR("Error in AFE4400_LED2ENDC register value\r\n");
  /**
   * Check setting gain in afe module
   */
  s_tia_amb_gain _tia_amb_value =
    {
      .amb_dac  = AMB_DAC_6uA,
      .stg2gain = STG2GAIN_6dB,
      .cfLED    = CF_LED_25plus5pF,
      .rfLED    = RF_LED_100k
    };
  afe_set_gain(&_tia_amb_value);
  afe_read_reg(AFE4400_TIA_AMB_GAIN, &_check_val);
  if(!(
      (( _check_val        & 0x07)  == RF_LED_100k)      &&
      (( _check_val >> 3   & 0x1F)  == CF_LED_25plus5pF) &&
      (((_check_val >> 8)  & 0x07)  == STG2GAIN_6dB)     &&
      (((_check_val >> 16) & 0x0F)  == AMB_DAC_6uA)
    ))
    NRF_LOG_ERROR("Error in AFE4400_TIA_AMB_GAIN\r\n");

  afe_conf();
#ifndef _USE_AFE_INT
  xTimerStart(m_read_afeTimer, 0);
#endif
  return IC_SUCCESS;
}
/*********************************************************************************************************************/
		/***********************	INIT FUNCTION  **************************************/
/**
 * @brief AFE4400 module initialization
 *
 * @return
 */
ic_return_val_e ic_afe_init(void(*cb)(s_led_val))
{
  afe_init();
  afe_conf(); // TODO: Move to init

  if (cb != NULL)
    m_user_cb = cb;

  return IC_SUCCESS;
}
/****************************************************************************************************/
/**
 * @brief Afe4400 deinitialization function
 *
 * Delete all of the created tasks, timers, etc.
 */
ic_return_val_e ic_afe_deinit(void)
{
    /*  deconfigurate afe4400 module  */
  afe_deinit();
    /* deconfigurate gpio used for afe4400  */
    /*	delete afe_wait task	*/
  if (m_init_thread != NULL)
    vTaskDelete(m_init_thread);

  NRF_LOG_INFO("AFE deinitialized\r\n");

  return IC_SUCCESS;
}
/****************************************************************************************************/
/****************************************************************************************************/
