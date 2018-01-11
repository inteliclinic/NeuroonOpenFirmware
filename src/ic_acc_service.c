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

#define ACC_TIMER_DATA_PERIOD		64

TimerHandle_t m_acc_data_timer;

static volatile acc_data_s m_acc_buffer;

static void(*m_user_cb)(acc_data_s data);

/**************************************************************************************************************************/
acc_data_s ic_acc_get_data()
{
  return m_acc_buffer;
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
  ic_acc_do_self_test1();
  vTaskDelay(90);
  ic_acc_do_self_test2();
  vTaskDelay(90);
  ic_acc_do_self_test3();
  vTaskDelay(90);

  return IC_SUCCESS;
}
/**************************************************************************************************************************/

#ifdef _ACC_EXTI_MODE
/**************************************************************************************************************************/
#ifdef _CHECK_DATA_TIMER

void data_print_timer_callback()
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t) __func__);

  NRF_LOG_INFO("ACC VAL:\tx: %d, y: %d, z: %d\n",
    m_acc_buffer.x,
    m_acc_buffer.y,
    m_acc_buffer.z);
}
#endif
/**************************************************************************************************************************/
void ic_acc_irq_get_data(acc_data_s data)
{
//	NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);
    /*  pass lis3dh x y z data struct to static buffer */
  m_acc_buffer = data;
  if (m_user_cb != NULL)
    m_user_cb(m_acc_buffer);
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_init(void(*cb)(acc_data_s))
{
  m_user_cb = cb;
    /**
     *  init accelerometer module and pass your callback function
     *  in which you get data structure with accelerometer data
     **/
  if (ic_acc_init(ic_acc_irq_get_data) != IC_SUCCESS)
    return IC_ERROR;

#ifdef _CHECK_DATA_TIMER
  m_acc_data_timer = xTimerCreate("acc_data_tim", ACC_TIMER_DATA_PERIOD, pdTRUE, (void *) 0, data_print_timer_callback);

  if (xTimerStart(m_acc_data_timer, 0) != pdPASS)
    NRF_LOG_ERROR("Couldn't start acc_timer\r\n");
#endif
  return IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_deinit()
{
  if (ic_acc_deinit() != IC_SUCCESS)
    return IC_ERROR;

#ifdef _CHECK_DATA_TIMER
  if (m_acc_data_timer != NULL || xTimerDelete(m_acc_data_timer, 0) != pdPASS)
  {
  	NRF_LOG_ERROR("acc_wdt_timer delete error\r\n")
  	return IC_ERROR;
  }
#endif
  return IC_SUCCESS;
}
/*********************************************************************************************************************************/
#else

/**************************************************************************************************************************/
void acc_read_data_timer()
{
  static bool _driver_crushed = false;
  if(m_user_cb != NULL)
    _driver_crushed = ic_acc_read(m_user_cb, _driver_crushed) == IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_init(void(*cb)(acc_data_s))
{
  /***
   *  init accelerometer module and pass your callback function
   *  in which you reset watchdog timer (twi multiple data handling problem)
   *  and get data structure with accelerometer data
   */
  if (ic_acc_init(NULL) != IC_SUCCESS)
    return IC_ERROR;

  m_user_cb = cb;
    /*  create timer in which you will be periodically read data  */
  if (m_acc_data_timer == NULL)
    m_acc_data_timer  = xTimerCreate("acc_wdt_timer", ACC_TIMER_DATA_PERIOD, pdTRUE, (void *) 0, acc_read_data_timer);

  if (xTimerStart(m_acc_data_timer, 0) != pdPASS)
    NRF_LOG_ERROR("Couldn't start acc_timer\r\n");

  return IC_SUCCESS;
}
/**************************************************************************************************************************/
ic_return_val_e ic_acc_module_deinit()
{
  ic_acc_deinit();

  if (m_acc_data_timer != NULL || (xTimerDelete(m_acc_data_timer, 0) != pdPASS))
  {
    NRF_LOG_ERROR("acc_wdt_timer delete error\r\n")
    return IC_ERROR;
  }

  return IC_SUCCESS;
}
#endif  // !_ACC_EXTI_MODE
/**************************************************************************************************************************/
