/**
 * @file    ic_acc_driver.c
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_lis3dh.h"
#include "ic_acc_driver.h"
#include "ic_driver_twi.h"

#define NRF_LOG_MODULE_NAME "LIS3DH"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ic_driver_button.h"

ic_return_val_e ic_acc_init(void(*fp)(acc_data_s))
{
  uint8_t _val = 0;

  ic_lis3dh_init(fp);
  ic_lis3dh_set_g_range(LIS3DH_RATE_G_RANGE_4g);
  ic_lis3dh_get_g_range(&_val);

  return IC_SUCCESS;
}

ic_return_val_e ic_acc_deinit()
{
  ic_lis3dh_uninit();

  return IC_SUCCESS;
}

ic_return_val_e ic_acc_read(void(*fp)(acc_data_s))
{
  ic_lis3dh_read_data(fp);

  return IC_SUCCESS;
}

ic_return_val_e ic_acc_set_data_rate(acc_power_mode_e data_rate)
{
  ic_lis3dh_set_power_mode(data_rate);

  return IC_SUCCESS;
}

ic_return_val_e ic_acc_do_self_test()
{
  ic_lis3dh_self_test();

  return IC_SUCCESS;
}

