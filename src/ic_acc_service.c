/**
 * @file    ic_acc_service.c
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "app_timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ic_acc_driver.h"
#include "ic_acc_service.h"

#define WATCHDOG_TIMER_PERIOD   60
#define ACC_TIMER_DATA		50

TimerHandle_t acc_wdt_timer;
TimerHandle_t acc_data_timer;

static volatile acc_data_s buffer;

#ifdef _ACC_EXTI_MODE
/**************************************************************************************************************************/
void ic_wdt_get_data(acc_data_s data)
{
  NRF_LOG_INFO("ACC WDT VAL:\tx: %d, y: %d, z: %d\n",
    data.x,
    data.y,
    data.z);
}
/**************************************************************************************************************************/
void wdt_timer()
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);

  ic_acc_read(ic_wdt_get_data);
}
/**************************************************************************************************************************/
void data_callback()
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t) __func__);

//	acc_data_s data = {0};
//	buffer = ic_acc_get_data();
  NRF_LOG_INFO("ACC VAL:\tx: %d, y: %d, z: %d\n",
    buffer.x,
    buffer.y,
    buffer.z);
}
/**************************************************************************************************************************/
void ic_reset_acc_wdt(acc_data_s data)
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);
    /*  pass lis3dh x y z data struct  */
  buffer = data;
    /*  reset the watchdog timer  */
  if (acc_wdt_timer != NULL)
    if (xTimerResetFromISR(acc_wdt_timer, 0) != pdPASS)
      NRF_LOG_ERROR("Couldn't reset timer\r\n");
}
/**************************************************************************************************************************/
acc_data_s ic_acc_get_data()
{
  return buffer;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_set_rate(acc_power_mode_e data_rate)
{
  ic_acc_set_data_rate(data_rate);

  return IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_selftest(void)
{
  if (ic_acc_do_self_test() != IC_SUCCESS)
    return IC_ERROR;

  return IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_init(void)
{
    /**
     *  init accelerometer module and pass your callback function
     *  in which you reset watchdog timer (twi multiple data handling problem)
     *  and get data structure with accelerometer data
     **/
  if (ic_acc_init(ic_reset_acc_wdt) != IC_SUCCESS)
    return IC_ERROR;

  acc_wdt_timer  = xTimerCreate("acc_wdt_timer", WATCHDOG_TIMER_PERIOD, pdTRUE, (void *) 0, wdt_timer);
  acc_data_timer = xTimerCreate("acc_data_tim", ACC_TIMER_DATA, pdTRUE, (void *) 0, data_callback);

  if (xTimerStart(acc_wdt_timer, 0) != pdPASS)
    NRF_LOG_ERROR("Couldn't start acc_timer\r\n");
  if (xTimerStart(acc_data_timer, 0) != pdPASS)
    NRF_LOG_ERROR("Couldn't start acc_timer\r\n");

  return IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_deinit()
{
  ic_acc_deinit();

  if (acc_wdt_timer != NULL || (xTimerDelete(acc_wdt_timer, 0) != pdPASS))
  {
  	NRF_LOG_ERROR("acc_wdt_timer delete error\r\n")
  	return IC_ERROR;
  }
  if (acc_data_timer != NULL || xTimerDelete(acc_data_timer, 0) != pdPASS)
  {
  	NRF_LOG_ERROR("acc_wdt_timer delete error\r\n")
  	return IC_ERROR;
  }
  return IC_SUCCESS;
}
/*********************************************************************************************************************************/
#else

void ic_wdt_get_data(acc_data_s data)
{
  NRF_LOG_INFO("ACC WDT VAL:\tx: %d, y: %d, z: %d\n",
    data.x,
    data.y,
    data.z);
}
/**************************************************************************************************************************/
void wdt_timer()
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);

  ic_acc_read(ic_wdt_get_data);
}
/**************************************************************************************************************************/
void ic_reset_acc_wdt(acc_data_s data)
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);
		/*	pass lis3dh x y z data struct  */
  buffer = data;
		/*  reset the watchdog timer  */
  if (acc_wdt_timer != NULL)
  {
    if (xTimerResetFromISR(acc_wdt_timer, 0) != pdPASS)
    {
      NRF_LOG_ERROR("Couldn't reset timer\r\n");
    }
  }
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_init(void)
{
  /***
   *  init accelerometer module and pass your callback function
   *  in which you reset watchdog timer (twi multiple data handling problem)
   *  and get data structure with accelerometer data
   */
  if (ic_acc_init(NULL) != IC_SUCCESS)
    return IC_ERROR;

  acc_wdt_timer  = xTimerCreate("acc_wdt_timer", ACC_TIMER_DATA, pdTRUE, (void *) 0, wdt_timer);

  if (xTimerStart(acc_wdt_timer, 0) != pdPASS)
  {
    NRF_LOG_ERROR("Couldn't start acc_timer\r\n");
  }

  return IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_deinit()
{
  ic_acc_deinit();

  if (acc_wdt_timer != NULL || (xTimerDelete(acc_wdt_timer, 0) != pdPASS))
  {
    NRF_LOG_ERROR("acc_wdt_timer delete error\r\n")
    return IC_ERROR;
  }

  return IC_SUCCESS;
}
#endif  // !_ACC_EXTI_MODE
/**************************************************************************************************************************/
