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

#define EXECUTE_HANDLER(code) do{\
  if(code!=NULL)code();else NRF_LOG_WARNING("No handler!");\
}while(0)

static void exti_btn(uint8_t pin, uint8_t button_action);
static void btn_long_press(TimerHandle_t xTimer);

static TimerHandle_t m_long_btn_press_timer;

static p_btnCode m_pwr_press_handle = 0;
static p_btnCode m_pwr_release_handle = 0;
static p_btnCode m_pwr_long_press_handle = 0;
static p_btnCode m_usb_connect_handle = 0;
static p_btnCode m_usb_disconnect_handle = 0;
static p_btnCode m_acc_rised_handle = 0;
static p_btnCode m_acc_fell_handle = 0;

static app_button_cfg_t m_buttons[] = {
  {
    .pin_no = IC_BUTTON_PWR_BUTTON_PIN,
    .active_state = APP_BUTTON_ACTIVE_HIGH,
    .pull_cfg = GPIO_PIN_CNF_PULL_Disabled,
    .button_handler = exti_btn
  },
  {
    .pin_no = IC_BUTTON_USB_CONNECT_PIN,
    .active_state = APP_BUTTON_ACTIVE_HIGH,
    .pull_cfg = GPIO_PIN_CNF_PULL_Disabled,
    .button_handler = exti_btn
  },
  {
    .pin_no = IC_BUTTON_ACC_EXTI_PIN,
    .active_state = APP_BUTTON_ACTIVE_HIGH,
    .pull_cfg = GPIO_PIN_CNF_PULL_Disabled,
    .button_handler = exti_btn
  }
};

void ic_btn_pwr_press_handle_init(p_btnCode code){
  m_pwr_press_handle = code;
}

void ic_btn_pwr_release_handle_init(p_btnCode code){
  m_pwr_release_handle = code;
}

void  ic_btn_pwr_long_press_handle_init(p_btnCode code){
  m_pwr_long_press_handle = code;
}

void ic_btn_usb_connect_handle_init(p_btnCode code){
  m_usb_connect_handle = code;
}

void ic_btn_usb_disconnect_handle_init(p_btnCode code){
  m_usb_disconnect_handle = code;
}

void ic_btn_acc_rise_handle_init(p_btnCode code){
  m_acc_rised_handle = code;
}

void ic_btn_acc_fall_handle_init(p_btnCode code){
  m_acc_fell_handle = code;
}

void neuroon_exti_init(void){
  __auto_type err_code = app_button_init(m_buttons,sizeof(m_buttons)/sizeof(m_buttons[0]) , 20);
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

void exti_btn(uint8_t pin, uint8_t button_action){
  __auto_type _HigherPriorityTaskWoken = pdFALSE;
  switch(pin&0xFF){
    case IC_BUTTON_PWR_BUTTON_PIN:
      if (button_action == APP_BUTTON_PUSH){
        EXECUTE_HANDLER(m_pwr_press_handle);
        if(pdFALSE == xTimerStartFromISR(m_long_btn_press_timer, &_HigherPriorityTaskWoken)){
          NRF_LOG_ERROR("xTimerStart Error!\n");
          NRF_LOG_FINAL_FLUSH();
        }
      }
      else{
        EXECUTE_HANDLER(m_pwr_release_handle);
        if(xTimerIsTimerActive(m_long_btn_press_timer)!=pdFALSE){
          if(pdFALSE == xTimerStopFromISR(m_long_btn_press_timer, &_HigherPriorityTaskWoken)){
            NRF_LOG_ERROR("xTimerStop Error!\n");
            NRF_LOG_FINAL_FLUSH();
          }
        }
        else{
          /*
           *printf("Well... Timer is dead\n\r");
           */
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
    case IC_BUTTON_ACC_EXTI_PIN:
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

