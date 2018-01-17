/**
 * @file    ic_driver_acc.c
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_lis3dh.h"
#include "ic_driver_acc.h"
#include "ic_driver_twi.h"

#define NRF_LOG_MODULE_NAME "LIS3DH"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ic_driver_button.h"

/*************************************************************/
ic_return_val_e ic_acc_init(void(*fp)(acc_data_s))
{
  if (ic_lis3dh_init(fp) == IC_SUCCESS)
  {
    ic_lis3dh_set_g_range(LIS3DH_RATE_G_RANGE_4g);
    ic_lis3dh_set_power_mode(LIS3DH_RATE_25Hz);
    return IC_SUCCESS;
  }
  return IC_ERROR;
}
/*************************************************************/
ic_return_val_e ic_acc_deinit()
{
  if (ic_lis3dh_uninit() != IC_SUCCESS)
    return IC_ERROR;

  return IC_SUCCESS;
}
/*************************************************************/
ic_return_val_e ic_acc_get_values(void(*fp)(acc_data_s), bool force)
{
  return ic_lis3dh_read_data(fp, force) != IC_SUCCESS;
}
/*************************************************************/
ic_return_val_e ic_acc_set_data_rate(acc_power_mode_e data_rate)
{
  if (ic_lis3dh_set_power_mode(data_rate) != IC_SUCCESS)
    return IC_ERROR;

  return IC_SUCCESS;
}
/*************************************************************/
ic_return_val_e ic_acc_do_self_test1()
{
  if (ic_lis3dh_self_test1() != IC_SUCCESS)
    return IC_ERROR;

  return IC_SUCCESS;
}
/*************************************************************/
ic_return_val_e ic_acc_do_self_test2()
{
  if (ic_lis3dh_self_test2() != IC_SUCCESS)
    return IC_ERROR;

  return IC_SUCCESS;
}
/*************************************************************/
ic_return_val_e ic_acc_do_self_test3()
{
  if (ic_lis3dh_self_test3() != IC_SUCCESS)
    return IC_ERROR;

  return IC_SUCCESS;
}
/*************************************************************/
