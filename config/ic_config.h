/**
 * @file    ic_config.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_CONFIG_H
#define IC_CONFIG_H

#include "nrf.h"
#include "app_util_platform.h"
#include "ic_common_types.h"

#define OSTIMER_WAIT_FOR_QUEUE  2 /**< Number of ticks to wait for the timer queue to be ready */

#define IC_IRQ_PRIORITY_HIGHEST APP_IRQ_PRIORITY_HIGHEST
#define IC_IRQ_PRIORITY_HIGH    APP_IRQ_PRIORITY_HIGH
#define IC_IRQ_PRIORITY_MID     APP_IRQ_PRIORITY_MID
#define IC_IRQ_PRIORITY_LOW     APP_IRQ_PRIORITY_LOW
#define IC_IRQ_PRIORITY_LOWEST  APP_IRQ_PRIORITY_LOWEST

/*
 *
 * UART
 *
 */
/** @defgroup IC_UART
 *  @{
 */

#define IC_UART_BUFFER        10
#define IC_UART_RX_BUF_SIZE   256
#define IC_UART_TX_BUF_SIZE   256

#define IC_UART_IRQ_PRIORITY  IC_IRQ_PRIORITY_LOWEST

#define IC_UART_RX_PIN        19
#define IC_UART_TX_PIN        20
#define IC_UART_RTS_PIN       0
#define IC_UART_CTS_PIN       0
#define IC_UART_FLOW_CONTROL  APP_UART_FLOW_CONTROL_DISABLED
#define IC_UART_BAUD_RATE     UART_BAUDRATE_BAUDRATE_Baud921600

/** @} */

/*
 *
 * BUTTON
 *
 */
/** @defgroup IC_BUTTON
 *  @{
 */

#define IC_BUTTON_LONG_PRESS_OFFSET 1500

#define IC_BUTTON_PWR_BUTTON_PIN    12
#define IC_BUTTON_USB_CONNECT_PIN   11
#define IC_BUTTON_ACC_EXTI_PIN      17

/** @} */

/*
 *
 * SPI
 *
 */
/** @defgroup IC_SPI
 *  @{
 */

#define IC_SPI_INSTANCE     1

#define IC_SPI_BUFFER       10
#define IC_SPI_RX_BUF_SIZE  256
#define IC_SPI_TX_BUF_SIZE  16

#define IC_SPI_IRQ_PRIORITY  IC_IRQ_PRIORITY_LOW

#define IC_SPI_FLASH_SS_PIN 13
#define IC_SPI_MISO_PIN     22
#define IC_SPI_MOSI_PIN     23
#define IC_SPI_SCK_PIN      6

/** @} */

/*
 *
 * TWI
 *
 */
/** @defgroup IC_TWI
 *  @{
 */
#define IC_TWI_100KHZ_FREQUENCY 0x01980000ul
#define IC_TWI_200KHZ_FREQUENCY 0x04000000ul
#define IC_TWI_400KHZ_FREQUENCY 0x06680000ul

#define IC_TWI_INSTANCE         0

#define IC_TWI_SDA_PIN          28
#define IC_TWI_SCL_PIN          29
#define IC_TWI_IRQ_PRIORITY     IC_IRQ_PRIORITY_HIGH
#define IC_TWI_FREQUENCY        IC_TWI_400KHZ_FREQUENCY

#define IC_TWI_PENDIG_TRANSACTIONS  5

/** @} */

/*
 *
 * LTC
 *
 */


/*
 *
 * SHORTCUTS
 *
 */
/** @defgroup IC_LTC
 *  @{
 */

#define IC_LTC_POWER_PIN    25

/** @} */

#include "core_cm0.h"

static inline int isr_context(){
  return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

/** @defgroup FreeRTOS
 *  @{
 */

#define IC_FREERTOS_TASK_PRIORITY_HIGHEST 4
#define IC_FREERTOS_TASK_PRIORITY_HIGH    3
#define IC_FREERTOS_TASK_PRIORITY_MEDIUM  2
#define IC_FREERTOS_TASK_PRIORITY_LOW     1
#define IC_FREERTOS_TASK_PRIORITY_LOWEST  0

#ifdef SEMAPHORE_H

#define ALLOCK_SEMAPHORE(name)                                              \
  static SemaphoreHandle_t name##_semaphore = NULL

#define INIT_SEMAPHORE_BINARY(name)                                         \
  name##_semaphore = xSemaphoreCreateBinary()

#define TAKE_SEMAPHORE(name, time_ms, ret_val)                              \
  do{                                                                       \
    if(isr_context()){                                                      \
      __auto_type higher_priority_task_woken = pdFALSE;                     \
      ret_val = xSemaphoreTakeFromISR(name##_semaphore,                     \
          &higher_priority_task_woken);                                     \
      if(ret_val == pdTRUE)                                                 \
        portYIELD_FROM_ISR(higher_priority_task_woken);                     \
    }                                                                       \
    else{                                                                   \
      ret_val = xSemaphoreTake(name##_semaphore, time_ms);                  \
    }                                                                       \
  }while(0)

#define GIVE_SEMAPHORE(name)                                                \
  do{                                                                       \
    if(isr_context()){                                                      \
      __auto_type higher_priority_task_woken = pdFALSE;                     \
      xSemaphoreGiveFromISR(name##_semaphore, &higher_priority_task_woken); \
      portYIELD_FROM_ISR(higher_priority_task_woken);                       \
    }                                                                       \
    else{                                                                   \
      xSemaphoreGive(name##_semaphore);                                     \
    }                                                                       \
  }while(0)

#endif /* SEMAPHORE_H */

#ifdef INC_TASK_H
#define RESUME_TASK(task)                                                   \
  do{                                                                       \
    if(isr_context()){                                                      \
      __auto_type task##_yield_required = xTaskResumeFromISR(task);         \
      portYIELD_FROM_ISR(task##_yield_required);                            \
    }                                                                       \
    else{                                                                   \
      vTaskResume(task);                                                    \
    }                                                                       \
  }while(0)

#endif /* INC_TASK_H */

#ifdef TIMERS_H

#define START_TIMER(timer, block_time, ret_val)                             \
  do{                                                                       \
    if(isr_context()){                                                      \
      __auto_type timer##_yield_required = pdFALSE;                         \
      ret_val = xTimerStartFromISR(timer, &timer##_yield_required);         \
      portYIELD_FROM_ISR(timer##_yield_required);                           \
    }                                                                       \
    else{                                                                   \
      ret_val = xTimerStart(timer, block_time);                             \
    }                                                                       \
  }while(0)

#define STOP_TIMER(timer, block_time, ret_val)                              \
  do{                                                                       \
    if(isr_context()){                                                      \
      __auto_type timer##_yield_required = pdFALSE;                         \
      ret_val = xTimerStopFromISR(timer, &timer##_yield_required);          \
      portYIELD_FROM_ISR(timer##_yield_required);                           \
    }                                                                       \
    else{                                                                   \
      ret_val = xTimerStop(timer, block_time);                              \
    }                                                                       \
  }while(0)

#endif /* TIMERS_H */
/** @} */

#endif /* !IC_CONFIG_H */
