/**
 * @file    ic_driver_button.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_button.h"
#include "app_button.h"
#include "nordic_common.h"
#include "nrf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <string.h>

#include "app_error.h"

#include "ic_config.h"

#define NRF_LOG_MODULE_NAME "BTN"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_drv_gpiote.h"

#define EXECUTE_HANDLER(code) do{\
  if(code!=NULL)code();else NRF_LOG_INFO("No handler!");\
}while(0)

static void exti_btn_callback(uint8_t pin, uint8_t button_action);
static void exti_callback(nrf_drv_gpiote_pin_t, nrf_gpiote_polarity_t);
static void btn_long_press(TimerHandle_t xTimer);

static TimerHandle_t m_long_btn_press_timer;

static void on_pwr_press(){
  NRF_LOG_INFO("{%s}\n\r", (uint32_t)__func__);
}

static void on_pwr_release(){
  NRF_LOG_INFO("{%s}\n\r", (uint32_t)__func__);
}

static void on_pwr_long_press(){
  NRF_LOG_INFO("{%s}\n\r", (uint32_t)__func__);
}

static p_btn_code m_pwr_press_handle = on_pwr_press;
static p_btn_code m_pwr_release_handle = on_pwr_release;
static p_btn_code m_pwr_long_press_handle = on_pwr_long_press;
static p_btn_code m_usb_connect_handle = NULL;
static p_btn_code m_usb_disconnect_handle = NULL;
static p_exti_code m_acc_handle = NULL;
static p_exti_code m_afe_handle = NULL;

static struct {
    uint8_t              pin_no;
    nrf_gpio_pin_pull_t  pull_cfg;
    nrf_drv_gpiote_evt_handler_t exti_callback_code;
}m_exti[] = {
  {
    .pin_no = IC_ACC_EXTI_PIN,
    .pull_cfg = GPIO_PIN_CNF_PULL_Pulldown,
    .exti_callback_code = exti_callback
  },
  {
    .pin_no = IC_AFE_EXTI_PIN,
    .pull_cfg = GPIO_PIN_CNF_PULL_Pulldown,
    .exti_callback_code = exti_callback
  }
};

static app_button_cfg_t m_buttons[] = {
  {
    .pin_no = IC_BUTTON_PWR_BUTTON_PIN,
    .active_state = APP_BUTTON_ACTIVE_HIGH,
    .pull_cfg = GPIO_PIN_CNF_PULL_Disabled,
    .button_handler = exti_btn_callback
  },
  {
    .pin_no = IC_BUTTON_USB_CONNECT_PIN,
    .active_state = APP_BUTTON_ACTIVE_HIGH,
    .pull_cfg = GPIO_PIN_CNF_PULL_Disabled,
    .button_handler = exti_btn_callback
  }
};

void ic_btn_pwr_press_handle_init(p_btn_code code){
  m_pwr_press_handle = code;
}

void ic_btn_pwr_release_handle_init(p_btn_code code){
  m_pwr_release_handle = code;
}

void  ic_btn_pwr_long_press_handle_init(p_btn_code code){
  m_pwr_long_press_handle = code;
}

void ic_btn_usb_connect_handle_init(p_btn_code code){
  m_usb_connect_handle = code;
}

void ic_btn_usb_disconnect_handle_init(p_btn_code code){
  m_usb_disconnect_handle = code;
}

void ic_acc_exti_handle_init(p_exti_code code){
  m_acc_handle = code;
}

void ic_afe_exti_handle_init(p_exti_code code){
  m_afe_handle = code;
}

void neuroon_exti_init(void){
  __auto_type err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  for (int i=0; i<sizeof(m_exti)/sizeof(m_exti[0]); ++i){
    nrf_drv_gpiote_in_config_t _pin_config =
    {
      .is_watcher = false,
      .hi_accuracy = true,
      .pull = NRF_GPIO_PIN_PULLDOWN,
      .sense = NRF_GPIOTE_POLARITY_TOGGLE,
    };
    err_code = nrf_drv_gpiote_in_init(m_exti[i].pin_no, &_pin_config, m_exti[i].exti_callback_code);
    nrf_drv_gpiote_in_event_enable(m_exti[i].pin_no, true);
    APP_ERROR_CHECK(err_code);
  }
  /*nrf_gpio_cfg_output(18);*/
  /*nrf_gpio_pin_clear(18);*/
  /*nrf_gpio_cfg_output(m_exti[0].pin_no);*/
  /*nrf_gpio_pin_clear(m_exti[0].pin_no);*/
  err_code = app_button_init(m_buttons,sizeof(m_buttons)/sizeof(m_buttons[0]) , 6);
  APP_ERROR_CHECK(err_code);
  err_code = app_button_enable();
  APP_ERROR_CHECK(err_code);
  m_long_btn_press_timer = xTimerCreate("BTNT", IC_BUTTON_LONG_PRESS_OFFSET, pdFALSE, NULL,
      btn_long_press);
}

static void btn_long_press(TimerHandle_t xTimer){
  UNUSED_VARIABLE(xTimer);
  EXECUTE_HANDLER(m_pwr_long_press_handle);
}

static void exti_callback(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t button_action){
  switch(pin&0xFF){
    case IC_ACC_EXTI_PIN:
      if(m_acc_handle!=NULL){
        m_acc_handle(nrf_gpio_pin_read(pin)?EXTI_EDGE_UP:EXTI_EDGE_DOWN);
      }
      else
        NRF_LOG_INFO("No handler!(LIS3DH)");
      break;
    case IC_AFE_EXTI_PIN:
      if(m_afe_handle!=NULL)
        m_afe_handle(button_action==NRF_GPIOTE_POLARITY_LOTOHI?EXTI_EDGE_UP:EXTI_EDGE_DOWN);
      else
        /*NRF_LOG_INFO("No handler!(AFE)");*/
      break;
    default:
      NRF_LOG_ERROR("Unsupported pin %d!\n", pin);
      break;
  }
}

static void exti_btn_callback(uint8_t pin, uint8_t button_action){
  switch(pin&0xFF){
    case IC_BUTTON_PWR_BUTTON_PIN:
      if (button_action == APP_BUTTON_PUSH){
        EXECUTE_HANDLER(m_pwr_press_handle);
        xTimerStart(m_long_btn_press_timer, 0);
      }
      else{
        EXECUTE_HANDLER(m_pwr_release_handle);
        if(xTimerIsTimerActive(m_long_btn_press_timer)!=pdFALSE){
          xTimerStop(m_long_btn_press_timer, 0);
        }
        else{
        }
      }
      /*if(button_action == APP_BUTTON_PUSH){*/
        /*if(m_pwr_press_handle != 0) ((p_btnCode)m_pwr_press_handle)();*/
      /*}*/
      /*else{*/
        /*if(m_pwr_release_handle != 0) ((p_btnCode)m_pwr_release_handle)();*/
      /*}*/
      break;
    case IC_BUTTON_USB_CONNECT_PIN:
      break;
    default:
      NRF_LOG_ERROR("Unsupported pin %d!\n", pin);
      break;
  }
#if 0
  NRF_LOG_INFO("Pressed pin: %d\n", 0xFF&pin);
  printf("Pressed pin: %d\n\r", 0xFF&pin);

  if(button_action == APP_BUTTON_PUSH){
    NRF_LOG_INFO("{%s}Button pressed!\n", (uint32_t)__func__);
    printf("{%s}Button pressed!\n\r", __func__);
    xTimerStart(m_long_btn_press_timer, OSTIMER_WAIT_FOR_QUEUE);
  }
  else{
    NRF_LOG_INFO("{%s}Button released!\n", (uint32_t)__func__);
    printf("{%s}Button released!\n\r", __func__);
  }
#endif
}

