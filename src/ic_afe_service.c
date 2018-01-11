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
  .led1_val  = 0,
  .led2_val  = 0,
  .aled1_val = 0,
  .aled2_val = 0,
  .diff_led1 = 0,
  .diff_led2 = 0
};

/*********************************************************************************************************************/
void afe_led_callback(s_led_val led_val)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/
    /*  copy led values to the global variable  */
  memcpy(&m_led_val, &led_val, sizeof(m_led_val));
    /*  you can print values here  */
/*
  NRF_LOG_INFO("1: %lu\r\n", m_led_val.led1_val);
  NRF_LOG_INFO("2: %lu\r\n", m_led_val.led2_val);
  NRF_LOG_INFO("3: %lu\r\n", m_led_val.aled1_val);
  NRF_LOG_INFO("4: %lu\r\n", m_led_val.aled2_val);
  NRF_LOG_INFO("5: %lu\r\n", m_led_val.diff_led1);
  NRF_LOG_INFO("6: %lu\r\n", m_led_val.diff_led2);
*/
}
/*********************************************************************************************************************/
#ifndef _USE_AFE_INT
void afe_readCallback()
{
    /*NRF_LOG_INFO("{ %s: %p }\r\n", (uint32_t)__func__, (uint32_t)m_user_cb);*/
  if (m_user_cb != NULL)
    afe_read_led_reg(m_user_cb);
}
#endif
/*********************************************************************************************************************/
#ifdef _USE_AFE_INT
/**
 * @brief Interrupt handler
 *
 * @param pin 		- number of pin on which interrupt occured
 * @param action 	- channel polarity
 *
 * Interrupt handler for AFE4400_RDY pin.
 * When ADC converting operation is done, RDY pin in AFE4400 goes high.
 *
 * You can also check which pin generates calling irq_function_handler
 * and what polarity action was set
 *
 */
void irq_afe_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//  NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);
  if (m_user_cb != NULL)
    afe_read_led_reg(m_user_cb);
}
/*********************************************************************************************************************/
                          /***********************	GPIO INTERRUPT INIT  **************************************/
void gpio_interrupt_init()
{
  NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);
  nrf_drv_gpiote_in_uninit(AFE4400_RDY_PIN);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  __auto_type err_code = nrf_drv_gpiote_in_init(AFE4400_RDY_PIN, &in_config, irq_afe_handler);
  NRF_LOG_INFO("CHECK ERR: %d\r\n", err_code);
  APP_ERROR_CHECK(err_code);
}
#endif
/*********************************************************************************************************************/
                          /***********************	GPIO CONFIGURATION  **************************************/
void afe_gpio_configuration(void)
{
  nrf_gpio_cfg_output(AFE4400_CS_PIN);
  nrf_gpio_cfg_output(AFE4400_PDN_PIN);
  nrf_gpio_cfg_output(AFE4400_RST_PIN);
  nrf_gpio_cfg_input(AFE4400_RDY_PIN, GPIO_PIN_CNF_PULL_Disabled);

  nrf_gpio_pin_set(AFE4400_PDN_PIN);
  nrf_gpio_pin_set(AFE4400_RST_PIN);
}
/*********************************************************************************************************************/
static void afe_gpio_deconfiguration(void)
{
  nrf_gpio_cfg_default(AFE4400_CS_PIN);
  nrf_gpio_cfg_default(AFE4400_PDN_PIN);
  nrf_gpio_cfg_default(AFE4400_RST_PIN);
  nrf_gpio_cfg_default(AFE4400_RDY_PIN);
}

  /* Array with timing values you want to write to specific timing registers
   * It is needed to write timing values in correct sequence (given in datasheet (page 31 table 2))
   */
uint16_t m_timing_data[29] =
{
  6050,	7998, 6000, 7999, 50, 1998, 2050, 3998,	2000, 3999, 4050, 5998,
  4, 1999, 2004, 3999, 4004, 5999, 6004, 7999, 0, 3, 2000, 2003, 4000,
  4003,	6000, 6003, 7999
};
/*********************************************************************************************************************/
                          /***********************	AFE TASK  **************************************/
void afe_conf(void)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/

  /*  check diagnostic register to be sure, that everything is okay  */
  if (afe_check_diagnostic() != 0)
    NRF_LOG_ERROR("Error has occurred, please check it\r\n");
    /*  set default timing values (from afe4400 datasheet)	*/
  /*afe_set_default_timing();*/
  /***
   * afe_set_timing_fast
   *
   * Using this function, you need to pass pointer to data (or just an array)
   * with specific timing values you want to write in timing registers.
   * !!!
   * 		 The most important thing is to write timing values in correct sequence (given in datasheet).
   * 		 If you give timing values in correct sequence, you do not need to worry about register addresses
   * !!!
   */
  afe_set_timing_fast(m_timing_data, sizeof(m_timing_data) / sizeof(uint16_t));
    /*	set led current on led1 and led2 (0 - 255)	*/
  afe_set_led_current(LED_CURRENT_MAX / 2, LED_CURRENT_MAX / 2);
  /*
  uint32_t _temp = 0;
  afe_read_reg(AFE4400_LEDCNTRL, &_temp);
  NRF_LOG_INFO("LED: %lu\r\n", _temp);
 */
  /***	set gain
   *
   *	amb_dac - value of cancellation current (0 - 10uA)
   * 	stg2gain - stage 2 gain (0dB - 12dB)
   *	cfLED - program capacity for LEDs (5pF - 150pF)
   *	rfLED - program resistance for LEDs (10kOhm - 1MOhm)
   *
   ***/
  s_tia_amb_gain _tia_amb_value =
  {
    .amb_dac  = AMB_DAC_1uA,
    .stg2gain = STG2GAIN_3dB,
    .cfLED    = CF_LED_15plus5pF,
    .rfLED    = RF_LED_50k
  };
  afe_set_gain(&_tia_amb_value);
  /*
  afe_read_reg(AFE4400_TIA_AMB_GAIN, &_temp);
  NRF_LOG_INFO("GAIN: %lu\r\n", _temp);
  */
    /*	call begin measure to turn leds and timers on  */
  afe_begin_measure();
#ifdef _USE_AFE_INT
    /*	enable interrupt on AFE4400_RDY_PIN	(configured earlier in gpio_interrupt_init() function)	*/
  nrf_drv_gpiote_in_event_enable(AFE4400_RDY_PIN, true);
    /*  enable interrupt vector  */
  NVIC_EnableIRQ(GPIOTE_IRQn);
#endif
}
/*********************************************************************************************************************/
ic_return_val_e ic_afe_set_gain(s_tia_amb_gain *tia_amb_value)
{
  afe_set_gain(tia_amb_value);

  return IC_SUCCESS;
}
/*********************************************************************************************************************/
ic_return_val_e ic_afe_set_led_current(uint8_t led1, uint8_t led2)
{
  afe_set_led_current(led1, led2);

  return IC_SUCCESS;
}
/*********************************************************************************************************************/
ic_return_val_e ic_afe_set_timing(uint16_t *tim_array, size_t len)
{
  afe_set_timing_fast(tim_array, len);

  return IC_SUCCESS;
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
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/
    /*	configurate gpio for afe4400  */
  /*afe_gpio_configuration();*/
#ifdef _USE_AFE_INT
    /*  initialize interrupt for adc RDY pin  */
  gpio_interrupt_init(); // TODO: delete!
#endif
    /*  init afe4400  */
  afe_init();
    /*  configurate afe4400 to start measure  */
  afe_conf(); // TODO: Move to init

  if (cb != NULL)
    m_user_cb = cb;

#ifndef _USE_AFE_INT
  if (m_read_afeTimer == NULL)
    m_read_afeTimer = xTimerCreate("AFE_READ", TIMER_READ_INTERVAL, pdTRUE, NULL, afe_readCallback);
    /*	if you want to check data in timer, just uncomment below  */
  if (xTimerStart(m_read_afeTimer, 0) != pdTRUE)
    NRF_LOG_ERROR("AFE_READ timer start error\r\n");
#endif

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
  afe_gpio_deconfiguration(); // TODO: move it to deinit
    /*  disable NVIC  */
  /*NVIC_DisableIRQ(GPIOTE_IRQn);*/ // TODO: delete it
    /*	delete afe_wait task	*/
  if (m_init_thread != NULL)
    vTaskDelete(m_init_thread);
#ifndef _USE_AFE_INT
  if (m_read_afeTimer != NULL)
    xTimerDelete(m_read_afeTimer, 0);
#endif

  NRF_LOG_INFO("AFE deinitialized\r\n");

  return IC_SUCCESS;
}
/****************************************************************************************************/
/****************************************************************************************************/
