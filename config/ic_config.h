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

#define IC_UART_IRQ_PRIORITY  APP_IRQ_PRIORITY_LOWEST

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

#define IC_UART_IRQ_PRIORITY  APP_IRQ_PRIORITY_LOWEST

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


/** @} */
#endif /* !IC_CONFIG_H */
