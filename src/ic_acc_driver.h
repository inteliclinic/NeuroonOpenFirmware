/**
 * @file    ic_acc_driver.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_ACC_DRIVER_H
#define IC_ACC_DRIVER_H

#include "ic_driver_lis3dh.h"
#include "ic_config.h"

/**
 * @brief Initialize accelerometer module
 *
 * @param fp - function pointer
 *
 * @return IC_SUCCESS when everything goes okay
 */
ic_return_val_e ic_acc_init(void(*fp)(acc_data_s));

/**
 * @brief Deinitialize accelerometer module
 *
 * @return
 */
ic_return_val_e ic_acc_deinit(void);

/**
 * @brief Read accelerometer data
 *
 * @param fp - function pointer
 *
 * @return IC_SUCCESS when everything goes okay
 */
ic_return_val_e ic_acc_read(void(*fp)(acc_data_s));

/**
 * @brief Set data rate on accelerometer
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
ic_return_val_e ic_acc_set_data_rate(acc_power_mode_e data_rate);

#endif	// !IC_ACC_DRIVER_H //
