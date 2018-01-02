/**
 * @file    ic_driver_afe4400.c
 * @author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#include <stdio.h>
#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ic_config.h"

#define NRF_LOG_MODULE_NAME "AFE4400"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_driver_spi.h"
#include "ic_driver_afe4400.h"

  /*  semaphore for blocking and releasing spi transfer  */
static volatile bool m_semaphore = true;

/**********************************************************************************************************/
	/*	configure SPI REGISTER to handle the SPI interrupt	*/
SPI_REGISTER(afe_spi_write);

//static uint8_t m_input_buffer[sizeof(afe_send_s)];
static uint8_t m_output_buffer[64];
static uint8_t m_input_buffer[128] = {0};
//static afe_send_s *m_data_to_send = (afe_send_s*)m_input_buffer;
/**********************************************************************************************************/
/**
 * @brief SPI transaction callback
 *
 * @param context
 *
 * You have to declare spi transaction callback.
 * The callback is returned after spi transaction is finished.
 * In callback function is highly recommended to handle some semaphore
 * which will be taken or given while triggering callback function.
 *
 * In this example, semaphore (bool variable) is set to true which means that spi transaction is completed.
 * Before using SPI_SEND_DATA in later code, semaphore is checked and properly handled.
 */
void callback_spi(void *context)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/
  m_semaphore = true;  // transaction ended, set semaphore value to true

  if(context != NULL)
  {
    *(uint32_t*)context  = m_output_buffer[7];
    *(uint32_t*)context |= m_output_buffer[6]  << 8;
    *(uint32_t*)context |= m_output_buffer[5]  << 16;
  }
  else
    UNUSED_VARIABLE(context);
}
/**********************************************************************************************************/
static uint32_t _led_val[AFE4400_LED_LEN] = {0};

void spi_led_callback(void *p_context)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/
  m_semaphore = true;  // transaction ended, set semaphore value to true

  if (p_context != NULL)
  {
    for (int i = 0; i < AFE4400_LED_LEN; i++)
    {
      _led_val[i]  = m_output_buffer[i * 4 + 3];
      _led_val[i] |= m_output_buffer[i * 4 + 2] << 8;
      _led_val[i] |= m_output_buffer[i * 4 + 1] << 16;
    }
    ((event_cb_done)p_context)(*(s_led_val*)_led_val);
  }
  else
    UNUSED_VARIABLE(p_context);
}
/**********************************************************************************************************/
ic_afe_ret_val afe_enable_read(void)
{
  if (m_semaphore)
  {
    m_semaphore = false;
    afe_send_pack_s *_data_to_send = (afe_send_pack_s *)&m_input_buffer[0];

    _data_to_send->reg = AFE4400_CONTROL0;
    convert(1, _data_to_send->data);  // convert data for sending via spi
    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, _data_to_send, m_output_buffer, sizeof(afe_send_pack_s), callback_spi, NULL);
    if (_ret_val != IC_SUCCESS)
      NRF_LOG_ERROR("ERROR SPI\r\n");
    while(m_semaphore == false);

    return AFE_SUCCESSS;
  }
  else
    return AFE_OPERATION_ON_GOING;
}
/**********************************************************************************************************/
ic_afe_ret_val afe_enable_write(void)
{
  if (m_semaphore)
  {
    m_semaphore = false;
    afe_send_pack_s *_data_to_send = (afe_send_pack_s *)&m_input_buffer[0];

    _data_to_send->reg = AFE4400_CONTROL0;
    memset(_data_to_send->data, 0, 3);	// convert data for sending via spi
    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, m_input_buffer, m_output_buffer, sizeof(afe_send_pack_s), callback_spi, NULL);
    if (_ret_val != IC_SUCCESS)
      NRF_LOG_ERROR("ERROR SPI\r\n");
    while(m_semaphore == false);

    return AFE_SUCCESSS;
  }
  else
    return AFE_OPERATION_ON_GOING;
}
/**********************************************************************************************************/
bool afe_set_timing_data(uint8_t regAddr, uint16_t tim_val)
{
    /* check if correct timing register is given  */
  if (regAddr < AFE4400_LED2STC || regAddr > AFE4400_PRPCOUNT)
  {
    NRF_LOG_INFO("Invalid register address\r\n");

    return false;
  }
  else
    afe_write_reg(regAddr, tim_val);

  return true;
}
/**********************************************************************************************************/
ic_afe_ret_val afe_write_reg(uint8_t regAddr, uint32_t regVal)
{
  if (m_semaphore)
  {
    afe_enable_write();
      /*  enable write flag  */
    m_semaphore = false;
      /*  put correct register address and register value to send  */
    afe_send_pack_s *_data_to_send = (afe_send_pack_s*)m_input_buffer;
    _data_to_send->reg = regAddr;
    convert(regVal, _data_to_send->data);	// convert data for sending via spi
    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, m_input_buffer, m_output_buffer, sizeof(afe_send_pack_s), callback_spi, NULL);
    if (_ret_val != IC_SUCCESS)
      NRF_LOG_ERROR("ERROR SPI\r\n");
    while(m_semaphore == false);

    afe_enable_read();	// enable reading (disable writing)

    return AFE_SUCCESSS;
  }
  else
  {
    NRF_LOG_INFO("Can't send via SPI\r\n");
    return AFE_OPERATION_ON_GOING;
  }
}
/**********************************************************************************************************/
void afe_write_bit_reg(uint8_t regAddr, uint8_t bit, bool bit_high)
{
  uint32_t _temp_data = 0;
  afe_read_reg(regAddr, &_temp_data);

  if (bit_high & !(_temp_data & 1 << bit))
    _temp_data |= (bit_high << bit);
  else if ((!bit_high) & (_temp_data & 1 << bit))
    _temp_data &= ~(bit_high << bit);

  afe_write_reg(regAddr, _temp_data);
}
/**********************************************************************************************************/
ic_afe_ret_val afe_read_reg(uint8_t regAddr, uint32_t *reg_value)
{
  if (m_semaphore)
  {
    m_semaphore = false;

    afe_send_s *_data_to_send = (afe_send_s*)m_input_buffer;
    _data_to_send->operation = READ_OPERATION;  // store register address
    _data_to_send->data.reg = regAddr;
    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, m_input_buffer, m_output_buffer, sizeof(afe_send_s), callback_spi, NULL);
    if (_ret_val != IC_SUCCESS)
      NRF_LOG_ERROR("SPI ERROR\r\n");
      /*  make this function blocking  */
    while(m_semaphore == false){}

    *reg_value  = m_output_buffer[7];
    *reg_value |= m_output_buffer[6] << 8;
    *reg_value |= m_output_buffer[5] << 16;

    return AFE_SUCCESSS;
  }
  else
  {
    NRF_LOG_INFO("Can't send via SPI\r\n");

    return AFE_OPERATION_ON_GOING;
  }
}
/**********************************************************************************************************/
ic_return_val_e afe_read_led_reg(event_cb_done cb)
{
  if (m_semaphore)
  {
    m_semaphore = false;
    afe_send_pack_s *_data_to_send = (afe_send_pack_s *)&m_input_buffer[0];

    for (int i = 0; i < AFE4400_LED_LEN; i++)
    {
      _data_to_send->reg = AFE4400_LED2VAL + i;
      memset(_data_to_send->data, 0, sizeof(_data_to_send->data));
      ++(_data_to_send);
    }
    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, m_input_buffer, m_output_buffer, (SPI_SEND_4BYTES * AFE4400_LED_LEN) + (AFE4400_LED_LEN * 3), spi_led_callback, cb);
    if (_ret_val != IC_SUCCESS)
      return IC_ERROR;
  }

  return IC_SUCCESS;
}
/**********************************************************************************************************/
//void afe_read_led_reg_instant(size_t len)
//{
//  for (int i = 0;i < len;i++)
//  {
//    data_to_send->reg = AFE4400_LED2VAL + i;
//    convert(0, data_to_send->data);
//    (++data_to_send);
//  }
//    /*  set pointer back to starting index in input_buffer  */
//  data_to_send = (s_spiSend*)&input_buffer[0];
//
//  if (semaphore)
//  {
//    semaphore = false;
//    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, input_buffer, output_buffer, SPI_SEND_4BYTES * len, led_read_callback);
//    if (_ret_val != IC_SUCCESS)
//      NRF_LOG_ERROR("SPI ERROR\r\n");
//  }
//}
/**********************************************************************************************************/
void afe_begin_measure(void)
{
#ifdef AFE_NRF_DEBUG
  uint32_t temp_data = 0;
#endif
  afe_write_bit_reg(AFE4400_CONTROL2, TXBRG_MODE_BIT, HBRIDGE_MODE);	/*	configure as an H-bridge	*/
  afe_write_bit_reg(AFE4400_CONTROL2, 17, 1);				/*	must be 1	 */
  afe_write_bit_reg(AFE4400_CONTROL2, 8, 1);				/*	must be 1	 */
#ifdef AFE_NRF_DEBUG
  afe_read_reg(AFE4400_CONTROL2, &temp_data);
  NRF_LOG_INFO("CONTROL2: %lu\r\n", temp_data);
#endif
  afe_write_bit_reg(AFE4400_LEDCNTRL, 16, 1);				/*	must be 1  */
  afe_write_bit_reg(AFE4400_LEDCNTRL, LEDCURR_OFF_BIT, LED_CURRENT_ON);	/*	turn on led current source	*/
#ifdef AFE_NRF_DEBUG
  afe_read_reg(AFE4400_LEDCNTRL, &temp_data);
  NRF_LOG_INFO("LEDCNTRL: %lu\r\n", temp_data);
#endif
  afe_write_bit_reg(AFE4400_CONTROL1, 1, 1);				/*	must be 1	 */
  afe_write_bit_reg(AFE4400_CONTROL1, TIMER_ENABLE, 1);			/*	enable timer module  */
#ifdef AFE_NRF_DEBUG
  afe_read_reg(AFE4400_CONTROL1, &temp_data);
  NRF_LOG_INFO("CONTROL1: %lu\r\n", temp_data);
#endif
}
/**********************************************************************************************************/
void afe_end_measure(void)
{
#ifdef AFE_NRF_DEBUG
  uint32_t temp_data = 0;
#endif
  afe_write_bit_reg(AFE4400_LEDCNTRL, LEDCURR_OFF_BIT, LED_CURRENT_OFF);	/*	turn off led current source	*/
#ifdef AFE_NRF_DEBUG
  afe_read_reg(AFE4400_LEDCNTRL, &temp_data);
  NRF_LOG_INFO("LEDCNTRL: %lu\r\n", temp_data);
#endif
  afe_write_bit_reg(AFE4400_CONTROL1, TIMER_ENABLE, 0);			/*	disable timer module  */
#ifdef AFE_NRF_DEBUG
  afe_read_reg(AFE4400_CONTROL1, &temp_data);
  NRF_LOG_INFO("CONTROL1: %lu\r\n", temp_data);
#endif
}
/**********************************************************************************************************/
void afe_set_default_timing(void)
{
    /*	set pulse repetition frequency to 500Hz, duty cycle to 25%  */
  afe_set_timing_data(AFE4400_LED2STC,      6050);
  afe_set_timing_data(AFE4400_LED2ENDC,     7998);
  afe_set_timing_data(AFE4400_LED2LEDSTC,   6000);
  afe_set_timing_data(AFE4400_LED2LEDENDC,  7999);
  afe_set_timing_data(AFE4400_ALED2STC,     50);
  afe_set_timing_data(AFE4400_ALED2ENDC,    1998);
  afe_set_timing_data(AFE4400_LED1STC,      2050);
  afe_set_timing_data(AFE4400_LED1ENDC,     3998);
  afe_set_timing_data(AFE4400_LED1LEDSTC,   2000);
  afe_set_timing_data(AFE4400_LED1LEDENDC,  3999);
  afe_set_timing_data(AFE4400_ALED1STC,     4050);
  afe_set_timing_data(AFE4400_ALED1ENDC,    5998);
  afe_set_timing_data(AFE4400_LED2CONVST,   4);
  afe_set_timing_data(AFE4400_LED2CONVEND,  1999);
  afe_set_timing_data(AFE4400_ALED2CONVST,  2004);
  afe_set_timing_data(AFE4400_ALED2CONVEND, 3999);
  afe_set_timing_data(AFE4400_LED1CONVST,   4004);
  afe_set_timing_data(AFE4400_LED1CONVEND,  5999);
  afe_set_timing_data(AFE4400_ALED1CONVST,  6004);
  afe_set_timing_data(AFE4400_ALED1CONVEND, 7999);
  afe_set_timing_data(AFE4400_ADCRSTSTCT0,  0);
  afe_set_timing_data(AFE4400_ADCRSTENDCT0, 3);
  afe_set_timing_data(AFE4400_ADCRSTSTCT1,  2000);
  afe_set_timing_data(AFE4400_ADCRSTENDCT1, 2003);
  afe_set_timing_data(AFE4400_ADCRSTSTCT2,  4000);
  afe_set_timing_data(AFE4400_ADCRSTENDCT2, 4003);
  afe_set_timing_data(AFE4400_ADCRSTSTCT3,  6000);
  afe_set_timing_data(AFE4400_ADCRSTENDCT3, 6003);
  afe_set_timing_data(AFE4400_PRPCOUNT,     7999);
}
/**********************************************************************************************************/
void afe_set_timing_fast(uint32_t *timing_data, size_t data_len)
{
  if (m_semaphore)
  {
      /*  set write flag  */
    afe_enable_write();
    m_semaphore = false;
    afe_send_pack_s *_data_to_send = (afe_send_pack_s*)&m_input_buffer[0];

    for (int i = 0; i < data_len; i++)
    {
      _data_to_send->reg = AFE4400_LED2STC + i;
      convert(timing_data[i], _data_to_send->data);
      ++(_data_to_send);
    }
    __auto_type _ret_val = SPI_SEND_DATA(afe_spi_write, m_input_buffer, m_output_buffer, data_len * sizeof(afe_send_pack_s), callback_spi, NULL);
    if (_ret_val != IC_SUCCESS)
      NRF_LOG_ERROR("SPI ERROR\r\n");
  }
    /*  set read flag  */
  afe_enable_read();
}
/**********************************************************************************************************/
void afe_set_led_current(uint8_t led1, uint8_t led2)
{
  uint32_t _temp_data = 0;

  _temp_data |= (1     << 16);		/*	must be 1	*/
  _temp_data |= (led1  << 8);
  _temp_data |= (led2);

  afe_write_reg(AFE4400_LEDCNTRL, _temp_data);
}
/**********************************************************************************************************/
void afe_reset(void)
{
  afe_write_bit_reg(AFE4400_CONTROL0, 3, 1);
}
/**********************************************************************************************************/
void afe_init(void)
{
    /*	Initialize SPI by giving name of the handler and CS pin  */
  SPI_INIT(afe_spi_write, AFE4400_CS_PIN);
//  afe_disable_read();
  afe_reset();
    /*	enable reading from afe4400  */
//  afe_enable_read();
}
/**********************************************************************************************************/
void afe_deinit(void)
{
    /*  for sure, you can set led current on leds to 0  */
  afe_set_led_current(0, 0);
  afe_end_measure();
    /*  uninit spi interface  */
  SPI_UNINIT(afe_spi_write);
}
/**********************************************************************************************************/
uint16_t afe_check_diagnostic(void)
{
  uint32_t _temp_data = 0;

  _temp_data = 0;
    /*  enable diagnostic flag  */
  afe_write_bit_reg(AFE4400_CONTROL0, DIAG_EN, ENABLE_DIAGNOSTIC);
  afe_read_reg(AFE4400_DIAG, &_temp_data);

  return _temp_data & 0x1FFF;
}
/**********************************************************************************************************/
void afe_set_gain(s_tia_amb_gain *tia_amb_value)
{
  uint32_t _temp_data = 0;

  _temp_data = 0;
  _temp_data |= tia_amb_value->amb_dac   << 16;
  _temp_data |= STAGE2EN;
  _temp_data |= tia_amb_value->stg2gain  << 8;
  _temp_data |= tia_amb_value->cfLED     << 3;
  _temp_data |= tia_amb_value->rfLED;

  afe_write_reg(AFE4400_TIA_AMB_GAIN, _temp_data);
}
/**********************************************************************************************************/
void convert(uint32_t regVal, uint8_t *data)
{
  data[0] = regVal >> 16;
  data[1] = regVal >> 8;
  data[2] = regVal;
}
/**********************************************************************************************************/
