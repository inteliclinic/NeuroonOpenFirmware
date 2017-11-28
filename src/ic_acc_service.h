/**
 * @file    ic_acc_service.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */
#include "ic_acc_driver.h"

/**
 * Define whether you want to use accelerometer in EXTINT mode
 */
//#define _ACC_EXTI_MODE

/**
 * @brief Reset accelerometer watchdog timer
 *
 * @param data - structure containing x,y,z data
 *
 * Function for reseting timer - just for dealing with TWI problems
 *
 */
void ic_reset_acc_wdt(acc_data_s data);

/**
 *
 * @param data
 */
void ic_wdt_get_data(acc_data_s data);

/**
 * @brief Initialize accelerometer module
 */
ic_return_val_e ic_acc_module_init(void);

/**
 * @brief Deinitialization accelerometer module
 *
 * @return
 */
ic_return_val_e ic_acc_module_deinit(void);

/**
 * @brief Get x,y,x data from accelerometer
 *
 * @return acc_data_s structure with x,y,z accelerometer data
 */
acc_data_s ic_acc_get_data(void);


/**
 * @brief Set data rate for accelerometer module
 *
 * @param data_rate - value of g range
 *
 *	 LIS3DH_RATE_1Hz
 *	 LIS3DH_RATE_10Hz
 *	 LIS3DH_RATE_25Hz
 *	 LIS3DH_RATE_50Hz
 *	 LIS3DH_RATE_100Hz
 *	 LIS3DH_RATE_200Hz
 *	 LIS3DH_RATE_400Hz
 *	 LIS3DH_RATE_LP
 *
 * @return IC_SUCCESS if everything goes okay
 */
ic_return_val_e ic_acc_set_rate(acc_power_mode_e data_rate);

