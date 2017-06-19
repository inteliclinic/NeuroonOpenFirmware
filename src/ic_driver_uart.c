/**
 * @file    ic_driver_uart.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */

/*#include "ic_driver_uart.h"*/
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_uart.h"
#include "ic_config.h"
//#include "ble_nus.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define NRF_LOG_MODULE_NAME "UART"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

void uart_receive_handler(uint8_t character);

//static ble_nus_t m_nus; /**< Structure to identify the Nordic UART Service. */
//
void __attribute__((weak)) uart_receive_handler(uint8_t character){
  app_uart_put(character);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
  uint8_t _character;

  switch (p_event->evt_type)
  {
    case APP_UART_DATA_READY:
/*
 *      UNUSED_VARIABLE(app_uart_get(&data_array[index]));
 *      index++;
 *
 *      if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
 *      {
 *        err_code = ble_nus_string_send(&m_nus, data_array, index);
 *        if (err_code != NRF_ERROR_INVALID_STATE)
 *        {
 *          APP_ERROR_CHECK(err_code);
 *        }
 *
 *        index = 0;
 *      }
 */
      while(app_uart_get(&_character) == NRF_SUCCESS){
        uart_receive_handler(_character);
      }

      break;

    case APP_UART_COMMUNICATION_ERROR:
      NRF_LOG_INFO("APP_UART_COMMUNICATION_ERROR: %d\n", p_event->data.error_communication);
      /*APP_ERROR_HANDLER(p_event->data.error_communication);*/
      break;

    case APP_UART_FIFO_ERROR:
      APP_ERROR_HANDLER(p_event->data.error_code);
      break;

    default:
      break;
  }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
*/

/**@snippet [UART Initialization] */
void ic_uart_init(void)
{
  uint32_t                     err_code;
  const app_uart_comm_params_t comm_params =
  {
    IC_UART_RX_PIN,
    IC_UART_TX_PIN,
    IC_UART_RTS_PIN,
    IC_UART_CTS_PIN,
    IC_UART_FLOW_CONTROL,
    false,
    IC_UART_BAUD_RATE
  };

  APP_UART_FIFO_INIT( &comm_params,
      IC_UART_RX_BUF_SIZE,
      IC_UART_TX_BUF_SIZE,
      uart_event_handle,
      IC_UART_IRQ_PRIORITY,
      err_code);
  APP_ERROR_CHECK(err_code);

}
/**@snippet [UART Initialization] */

int _write(int fd, char *str, int len){
  for(int i = 0; i<len; i++){
    app_uart_put(*str++);
  }
  return len;
}
