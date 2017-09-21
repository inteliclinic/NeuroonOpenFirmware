/**
 * @file    ic_afe_service.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_AFE_SERVICE_H
#define _IC_AFE_SERVICE_H

#include <stdint.h>
#include <stddef.h>

#include "ic_driver_afe4400.h"
#include "nrf_drv_gpiote.h"

/**
 * if you want to use AFE4400 with RDY interrupt
 * uncomment below
 */
//#define _USE_AFE_INT

/**
 * @brief Initialize AFE4400 module
 *
 * By calling this function, you initialize all modules used in ic_afe_service
 *
 * afe_gpio_configuration() - configuration used ports/pins for afe4400
 * gpio_interrupt_init()    - interrupt configuration for handling RDY pin in afe4400
 * afe_task() 		    - afe4400 testing function which calls all the necessary functions for start using afe module
 *
 *   -> afe_init() 								- init SPI connection and set afe for reading mode
 *   -> afe_check_diagnostic()							- check diagnostic register to see whether everything is okay with your afe4400
 *   -> afe_set_default_timing()/afe_set_default_timing_fast(data, data_size) 	- set timing values in timing registers
 *   -> afe_set_led_current(led1_value, led2_value) 				- set led current values
 *   -> afe_set_gain(tia_amb_gain_values) 					- set correct values in TIA_AMB_GAIN register
 *   -> afe_begin_measure() 							- set the last needed values for starting measurements
 *
 * It is necessary to call nrf_drv_gpiote_in_event_enable() and NVIC_EnableIRQ() functions AFTER setting afe_begin_measure()
 * to be sure, that interrupt will not start until setting correct values in specific registers
 *
 *   -> nrf_drv_gpiote_in_event_enable(AFE4400_RDY_PIN, true);
 *   -> NVIC_EnableIRQ(GPIOTE_IRQn);
 *
 * @return
 */
ic_return_val_e ic_afe_init(void(*cb)(led_val_s));

/**
 * @brief Deinitialize AFE4400 module
 *
 * Delete all created freeRTOS tasks and timers
 *
 * @return
 */
ic_return_val_e ic_afe_deinit(void);

ic_return_val_e ic_afe_set_gain(s_tia_amb_gain *tia_amb_value);
ic_return_val_e ic_afe_set_led_current(uint8_t led1, uint8_t led2);
ic_return_val_e ic_afe_set_timing(uint32_t *tim_array, size_t len);

#endif /* !_IC_AFE_SERVICE_H */
