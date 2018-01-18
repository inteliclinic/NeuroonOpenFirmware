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

	/* define AFE4400 pins  */
/** @brief AFE4400 pins connected to NRF51822
 *
 * Definitions should go to ic_config file
 *
 */
#define AFE4400_RDY_PIN	0
#define AFE4400_CS_PIN	2
#define AFE4400_PDN_PIN 10
#define AFE4400_RST_PIN 30

#define AFE4400_LED_LEN 6
/**
 * @brief Structure for sending data packages
 *
 * @param reg 	  - register value you want to read/write
 * @param data[3] - 3 bytes of data (read/write)
 *
 * Single sending frame contains 1 byte of register address
 * and 3 bytes of sending/receiving data
 *
 * Even if multiple instructions sending, the frame is still the same
 */
typedef struct __attribute__((packed))
{
  uint8_t reg;	  // 1 byte reg addr
  uint8_t data[3];  // 3 bytes of data
}afe_send_pack_s;

typedef struct __attribute__((packed))
{
  uint32_t operation;    // read or write operation
  afe_send_pack_s data;
}afe_send_s;

#define WRITE_OPERATION 0
#define READ_OPERATION (uint32_t)1 << 24

/**
 * @brief Register addresses
 *
 * Enumeration contains addresses of AFE4400 registers
 */
enum
{
    /*	Defines for Registers	*/
  AFE4400_CONTROL0        = 0x00,  //!< AFE4400_CONTROL0
    /*	timing registers	*/
  AFE4400_LED2STC         = 0x01,  //!< AFE4400_LED2STC
  AFE4400_LED2ENDC        = 0x02,  //!< AFE4400_LED2ENDC
  AFE4400_LED2LEDSTC      = 0x03,  //!< AFE4400_LED2LEDSTC
  AFE4400_LED2LEDENDC     = 0x04,  //!< AFE4400_LED2LEDENDC
  AFE4400_ALED2STC        = 0x05,  //!< AFE4400_ALED2STC
  AFE4400_ALED2ENDC       = 0x06,  //!< AFE4400_ALED2ENDC
  AFE4400_LED1STC         = 0x07,  //!< AFE4400_LED1STC
  AFE4400_LED1ENDC        = 0x08,  //!< AFE4400_LED1ENDC
  AFE4400_LED1LEDSTC     	= 0x09,  //!< AFE4400_LED1LEDSTC
  AFE4400_LED1LEDENDC    	= 0x0A,  //!< AFE4400_LED1LEDENDC
  AFE4400_ALED1STC       	= 0x0B,  //!< AFE4400_ALED1STC
  AFE4400_ALED1ENDC      	= 0x0C,  //!< AFE4400_ALED1ENDC
  AFE4400_LED2CONVST     	= 0x0D,  //!< AFE4400_LED2CONVST
  AFE4400_LED2CONVEND    	= 0x0E,  //!< AFE4400_LED2CONVEND
  AFE4400_ALED2CONVST    	= 0x0F,  //!< AFE4400_ALED2CONVST
  AFE4400_ALED2CONVEND   	= 0x10,  //!< AFE4400_ALED2CONVEND
  AFE4400_LED1CONVST     	= 0x11,  //!< AFE4400_LED1CONVST
  AFE4400_LED1CONVEND    	= 0x12,  //!< AFE4400_LED1CONVEND
  AFE4400_ALED1CONVST    	= 0x13,  //!< AFE4400_ALED1CONVST
  AFE4400_ALED1CONVEND   	= 0x14,  //!< AFE4400_ALED1CONVEND
  AFE4400_ADCRSTSTCT0    	= 0x15,  //!< AFE4400_ADCRSTSTCT0
  AFE4400_ADCRSTENDCT0   	= 0x16,  //!< AFE4400_ADCRSTENDCT0
  AFE4400_ADCRSTSTCT1    	= 0x17,  //!< AFE4400_ADCRSTSTCT1
  AFE4400_ADCRSTENDCT1   	= 0x18,  //!< AFE4400_ADCRSTENDCT1
  AFE4400_ADCRSTSTCT2    	= 0x19,  //!< AFE4400_ADCRSTSTCT2
  AFE4400_ADCRSTENDCT2   	= 0x1A,  //!< AFE4400_ADCRSTENDCT2
  AFE4400_ADCRSTSTCT3    	= 0x1B,  //!< AFE4400_ADCRSTSTCT3
  AFE4400_ADCRSTENDCT3  	= 0x1C,  //!< AFE4400_ADCRSTENDCT3
  AFE4400_PRPCOUNT       	= 0x1D,  //!< AFE4400_PRPCOUNT
    /*	rest of the registers  */
  AFE4400_CONTROL1       	= 0x1E,  //!< AFE4400_CONTROL1
  AFE4400_SPARE1         	= 0x1F,  //!< AFE4400_SPARE1
  AFE4400_TIAGAIN        	= 0x20,  //!< AFE4400_TIAGAIN
  AFE4400_TIA_AMB_GAIN    = 0x21,  //!< AFE4400_TIA_AMB_GAIN
  AFE4400_LEDCNTRL       	= 0x22,  //!< AFE4400_LEDCNTRL
  AFE4400_CONTROL2       	= 0x23,  //!< AFE4400_CONTROL2
  AFE4400_SPARE2         	= 0x24,  //!< AFE4400_SPARE2
  AFE4400_SPARE3          = 0x25,  //!< AFE4400_SPARE3
  AFE4400_SPARE4         	= 0x26,  //!< AFE4400_SPARE4
  AFE4400_RESERVED1      	= 0x27,  //!< AFE4400_RESERVED1
  AFE4400_RESERVED2      	= 0x28,  //!< AFE4400_RESERVED2
  AFE4400_ALARM          	= 0x29,  //!< AFE4400_ALARM
  AFE4400_LED2VAL        	= 0x2A,  //!< AFE4400_LED2VAL
  AFE4400_ALED2VAL       	= 0x2B,  //!< AFE4400_ALED2VAL
  AFE4400_LED1VAL        	= 0x2C,  //!< AFE4400_LED1VAL
  AFE4400_ALED1VAL       	= 0x2D,  //!< AFE4400_ALED1VAL
  AFE4400_LED2_ALED2VAL  	= 0x2E,  //!< AFE4400_LED2_ALED2VAL
  AFE4400_LED1_ALED1VAL  	= 0x2F,  //!< AFE4400_LED1_ALED1VAL
  AFE4400_DIAG           	= 0x30,  //!< AFE4400_DIAG
};

/**
 * @brief Spi data length
 *
 * Enumeration contains the length of spi transaction,
 * just for making the code looks better
 */
enum
{
  SPI_SEND_1BYTE  = 0x01,  //!< SPI_SEND_1BYTE
  SPI_SEND_4BYTES = 0x04,  //!< SPI_SEND_4BYTES
};

#define	LED_CURRENT_MAX   255

/** @defgroup LOW_LEVEL_AFE4400_BITS definition of bitmap configuration in specific registers
 *  @ingroup LOW_LEVEL_DRIVER_AFE4400
 *  @{
 */
	/*	CONTROL0 register bit map	 */
#define SPI_READ_BIT 	    0
#define TIM_COUNT_RST	    1
#define DIAG_EN	          2
#define SW_RST		        3

#define ENABLE_DIAGNOSTIC 1

  /*  CONTROL1 register bitmap  */
#define TIMER_ENABLE      8

  /*  CONTROL2 register bits	*/
#define TXBRG_MODE_BIT    11
#define HBRIDGE_MODE      0
#define PUSHPULL_MODE     1

  /*	LEDCNTRL register bits  */
#define LEDCURR_OFF_BIT   17
#define LED_CURRENT_ON    0
#define LED_CURRENT_OFF   1

/**
 * @}
 */

/** @defgroup TIA_AMB_GAIN enumeration for setting gain values
 *  @ingroup LOW_LEVEL_DRIVER_AFE4400
 *  @{
 */
  /*	TIA_AMB_GAIN register	*/
/**
 * @brief Ambient dac values
 */
typedef enum __attribute__((packed))
{
  AMB_DAC_0uA = 0x00,  //!< AMB_DAC_0uA
  AMB_DAC_1uA,         //!< AMB_DAC_1uA
  AMB_DAC_2uA,         //!< AMB_DAC_2uA
  AMB_DAC_3uA,         //!< AMB_DAC_3uA
  AMB_DAC_4uA,         //!< AMB_DAC_4uA
  AMB_DAC_5uA,         //!< AMB_DAC_5uA
  AMB_DAC_6uA,         //!< AMB_DAC_6uA
  AMB_DAC_7uA,         //!< AMB_DAC_7uA
  AMB_DAC_8uA,         //!< AMB_DAC_8uA
  AMB_DAC_9uA,         //!< AMB_DAC_9uA
  AMB_DAC_10uA         //!< AMB_DAC_10uA
}e_amb_dac;

#define STAGE2EN	(1<<14)

/**
 * @brief Stage2gain values
 */
typedef enum __attribute__((packed))
{
  STG2GAIN_0dB = 0x00,  //!< STG2GAIN_0dB
  STG2GAIN_3dB,         //!< STG2GAIN_3dB
  STG2GAIN_6dB,         //!< STG2GAIN_6dB
  STG2GAIN_9dB,         //!< STG2GAIN_9dB
  STG2GAIN_12dB         //!< STG2GAIN_12dB
}e_stg2gain;

/**
 * @brief Cf LED values
 */
typedef enum __attribute__((packed))
{
  CF_LED_5pF        = 0x00,  //!< CF_LED_5pF
  CF_LED_5plus5pF 	= 0x01,  //!< CF_LED_5plus5pF
  CF_LED_15plus5pF 	= 0x02,  //!< CF_LED_15plus5pF
  CF_LED_25plus5pF 	= 0x04,  //!< CF_LED_25plus5pF
  CF_LED_50plus5pF 	= 0x08,  //!< CF_LED_50plus5pF
  CF_LED_150plus5pF = 0xF1   //!< CF_LED_150plus5pF
}e_cf_LED;

/**
 * @brief Rf LED values
 */
typedef enum __attribute__((packed))
{
  RF_LED_500k = 0x00,  //!< RF_LED_500k
  RF_LED_250k,         //!< RF_LED_250k
  RF_LED_100k,         //!< RF_LED_100k
  RF_LED_50k,          //!< RF_LED_50k
  RF_LED_25k,          //!< RF_LED_25k
  RF_LED_10k,          //!< RF_LED_10k
  RF_LED_1M            //!< RF_LED_1M
}e_rf_LED;

/**
 * @brief Structure for TIA_AMB_GAIN register
 *
 * Collect data to set specific gain values in TIA_AMB_GAIN register
 */
typedef struct __attribute__((packed))
{
  e_amb_dac 	amb_dac;
  e_stg2gain 	stg2gain;
  e_cf_LED 	  cfLED;
  e_rf_LED 	  rfLED;
}s_tia_amb_gain;

/**
 * @}
 */

  /*  semaphore for blocking and releasing spi transfer  */
static volatile bool m_semaphore = true;

/**********************************************************************************************************/
	/*	configure SPI REGISTER to handle the SPI interrupt	*/
SPI_REGISTER(afe_spi_write);

//static uint8_t m_input_buffer[sizeof(afe_send_s)];
static uint8_t m_output_buffer[128];
static uint8_t m_input_buffer[128] = {0};

  /* Array with timing values you want to write to specific timing registers
   * It is needed to write timing values in correct sequence (given in datasheet (page 31 table 2))
   */
static uint16_t m_timing_data[29] =
{
  6050,	7998, 6000, 7999, 50, 1998, 2050, 3998,	2000, 3999, 4050, 5998,
  4, 1999, 2004, 3999, 4004, 5999, 6004, 7999, 0, 3, 2000, 2003, 4000,
  4003,	6000, 6003, 7999
};
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
static void callback_spi(void *context)
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

static void spi_led_callback(void *p_context)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/
  m_semaphore = true;  // transaction ended, set semaphore value to true
  static uint32_t _led_val[AFE4400_LED_LEN] = {0};

  if (p_context != NULL)
  {
    for (int i = 0; i < AFE4400_LED_LEN; i++)
    {
      _led_val[i]  = m_output_buffer[i * 4 + 3];
      _led_val[i] |= m_output_buffer[i * 4 + 2] << 8;
      _led_val[i] |= m_output_buffer[i * 4 + 1] << 16;
        /*  two MSB can be ignored, but we're using 24-bit word format  */
      /*_led_val[i] &= ~(0xC00000);*/
    }
    ((ic_afe_event_cb_done)p_context)(*(ic_afe_val_s*)_led_val);
  }
  else
    UNUSED_VARIABLE(p_context);
}

/**
 * @brief Convert uint32_t register value to the uint8_t array for pushing data via SPI in correct order
 *
 * @param regVal 	- register value
 * @param data 		- pointer to uint8_t array
 */
static void convert(uint32_t regVal, uint8_t *data)
{
  data[0] = regVal >> 16;
  data[1] = regVal >> 8;
  data[2] = regVal;
}

/**
 * @brief Enable read option in AFE4400
 */
static ic_return_val_e afe_enable_read(void)
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

    return IC_SUCCESS;
  }
  else
    return IC_BUSY;
}

/**
 * @brief Disable read option (enable write option) in AFE4400
 */
static ic_return_val_e afe_enable_write(void)
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

    return IC_SUCCESS;
  }
  else
    return IC_BUSY;
}

/**
 * @brief Write data to afe4400's specific register
 *
 * @param regAddr - register address
 * @param regVal  - register value
 *
 * Example:
 * @code
 *
 * uint8_t regAddr = AFE4400_LED2STC;
 * uint32_t regVal = 0x1F;
 *
 *  // be sure that afe4400 is in writing mode
 *  // if not sure, firstly call afe_disable_read() function
 *
 * afe_write_reg(regAddr, regVal);
 *
 * @endcode
 */
static ic_return_val_e afe_write_reg(uint8_t regAddr, uint32_t regVal)
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

    return IC_SUCCESS;
  }
  else
  {
    NRF_LOG_INFO("Can't send via SPI\r\n");
    return IC_BUSY;
  }
}

/**
 * @brief Read AFE4400's specific register
 *
 * @param regAddr   - register address
 * @param reg_value - pointer to collected data
 *
 * Example:
 * @code
 *
 * uint8_t regAddr = AFE4400_LED2STC;
 * uint32_t reg_value = 0;
 *
 * // be sure that afe4400 is in reading mode
 * // if not sure, firstly call afe_read_enable() function
 *
 * afe_read_reg(regAddr, &regVal);
 * printf("Register value: %lu", regVal);
 *
 * @endcode
 */
static ic_return_val_e afe_read_reg(uint8_t regAddr, uint32_t *reg_value)
{
  if (m_semaphore)
  {
    m_semaphore = false;

    afe_send_s *_data_to_send = (afe_send_s*)m_input_buffer;
    _data_to_send->operation = READ_OPERATION;  // store register address
    _data_to_send->data.reg = regAddr;

    __auto_type _ret_val =
      SPI_SEND_DATA(
          afe_spi_write,
          m_input_buffer,
          m_output_buffer,
          sizeof(afe_send_s),
          callback_spi,
          NULL);

    if (_ret_val != IC_SUCCESS)
      NRF_LOG_ERROR("SPI ERROR\r\n");

      /*  make this function blocking  */
    while(m_semaphore == false){}

    *reg_value  = m_output_buffer[7];
    *reg_value |= m_output_buffer[6] << 8;
    *reg_value |= m_output_buffer[5] << 16;

    return IC_SUCCESS;
  }
  else
  {
    NRF_LOG_INFO("Can't send via SPI\r\n");
    return IC_BUSY;
  }
}

/**
 * @brief Write single bit to the afe4400's register
 *
 * @param regAddr 	- register address you want to write
 * @param bit		- number of bit in register you want to set/reset
 * @param bit_high 	- set (1) or reset (0) specific bit
 *
 * Example:
 * @code
 *
 * afe_write_bit_reg(AFE4400_CONTROL2, TXBRG_MODE_BIT, HBRIDGE_MODE);
 *
 * @endcode
 */
static void afe_write_bit_reg(uint8_t regAddr, uint8_t bit, bool bit_high){
  uint32_t _temp_data = 0;
  afe_read_reg(regAddr, &_temp_data);

  if (bit_high & !(_temp_data & 1 << bit))
    _temp_data |= (bit_high << bit);
  else if ((!bit_high) & (_temp_data & 1 << bit))
    _temp_data &= ~(bit_high << bit);

  afe_write_reg(regAddr, _temp_data);
}

/**
 * @brief Set gain in TIA_AMB_GAIN
 *
 * Specific ambient light cancellation amplifier gain, cancellation current and filter corner frequency in AFE4400
 *
 * @param tia_amb_value @ref s_tia_amb_gain:
 *  .amb_dac using 	@ref e_amb_dac	- ambient DAC value (0 - 10uA)
 *  .stg2gain	using	@ref e_stg2gain	- stage 2 gain setting (0dB; 3.5dB; 6dB; 9.5dB; 12dB)
 *  .cfLED using  	@ref e_cfLED	  - LEDs Cf value
 *  .rfLED using 	  @ref e_rfLED	  - LEDs Rf value
 *
 * Example:
 * @code
 *
 * 	s_tia_amb_gain tia_amb_value = {
 *		.amb_dac 	= AMB_DAC_2uA,
 *		.stg2gain = STG2GAIN_6dB,
 *		.cfLED 	 	= CF_LED_15plus5pF,
 *		.rfLED 	 	= RF_LED_100k
 *	};
 * afe_set_gain(&tia_amb_value);
 *
 * @endcode
 */
static void afe_set_gain(s_tia_amb_gain *tia_amb_value)
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

/**
 * @brief Set timing values in specific registers
 *
 * @param regAddr - register address
 * @param tim_val - timer value
 * @return true if everything is okay
 */
static ic_return_val_e afe_set_timing_data(uint8_t regAddr, uint16_t tim_val)
{
    /* check if correct timing register is given  */
  if (regAddr < AFE4400_LED2STC || regAddr > AFE4400_PRPCOUNT)
  {
    NRF_LOG_INFO("Invalid register address\r\n");

    return IC_ERROR;
  }
  else
    afe_write_reg(regAddr, tim_val);

  return IC_SUCCESS;
}

/**
 * @brief Check diagnostic register values
 *
 * @return value which can be checked for diagnostic
 */
static uint16_t afe_check_diagnostic(void)
{
  uint32_t _temp_data = 0;
    /*  enable diagnostic flag  */
  afe_write_bit_reg(AFE4400_CONTROL0, DIAG_EN, ENABLE_DIAGNOSTIC);
  afe_read_reg(AFE4400_DIAG, &_temp_data);

  return _temp_data & 0x1FFF;
}

ic_return_val_e ic_afe_get_values(ic_afe_event_cb_done cb, bool force){
  UNUSED_PARAMETER(force);
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

/**
 * @brief Begin measure
 *
 * Function which sets all of the necessary bits in specific registers,
 * so you can start measuring pulse and oxidation data
 */
static void afe_begin_measure(void)
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

/**
 * @brief End measures
 *
 * Function for deinitialization of afe4400 module
 * dealing with turning off timers and led current
 *
 */
static void afe_end_measure(void)
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

/**
 * @brief Set deafult timing values in specific registers
 *
 * Function sets specific timing values given by the device producer in the AFE4400 datasheet
 */
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

/**
 * @brief Set timing values in specific registers faster than afe_set_timing_data function
 *
 * @param timing_data - pointer to timing values you want to set in afe4400
 * @param data_len 		- length of the data you want to write
 *
 * Example:
 * @code
 *
 * uint32_t timing_data[29] =
 * {
 *   6050,	7998, 6000,	7999,	50, 1998,	2050,	3998,	2000,	3999,	4050,	5998,
 *   4, 1999, 2004, 3999, 4004, 5999, 6004, 7999, 0, 3, 2000, 2003, 4000,
 *   4003,	6000, 6003, 7999
 * };
 *
 * afe_set_timing_fast(timing_data, sizeof(timing_data) / sizeof(uint32_t));
 *
 * @endcode
 */
void afe_set_timing_fast(uint16_t *timing_data, size_t data_len)
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

/**
 * @brief Set current value on LED1 and LED2
 *
 * @param led1 - current value on led1 (0-255)
 * @param led2 - current value on led2 (0-255)
 *
 * The nominal value of LED current is given by the equation (page 65 in AFE4400 datasheet):
 * ( LED[7:0] / 256 ) * Full-Scale Current
 *
 * Example:
 * @code
 *
 * uint8_t led1_val = LED_CURRENT_MAX / 4;
 * uint8_t led2_val = LED_CURRENT_MAX / 2;
 *
 * afe_set_led_current(led1_val, led2_val);
 *
 * @endcode
 */
static void afe_set_led_current(uint8_t led1, uint8_t led2){
  union{
    uint32_t val;
    uint8_t array [4];
  }_temp_data;

  _temp_data.val = 0x00;

  _temp_data.array[0] = led2;
  _temp_data.array[1] = led1;
  _temp_data.array[2] = 0x01;
  _temp_data.array[3] = 0x00;
  NRF_LOG_INFO("0x%08X\n", _temp_data.val);

  afe_write_reg(AFE4400_LEDCNTRL, _temp_data.val);
}

/**
 * @brief Software device reset
 */
static void afe_reset(void)
{
  afe_write_bit_reg(AFE4400_CONTROL0, 3, 1);
}

static void afe_conf(void)
{
  /*NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);*/

  /*  check diagnostic register to be sure, that everything is okay  */
  if (afe_check_diagnostic() != 0)
    NRF_LOG_ERROR("Error has occurred, please check it\r\n");
  /***
   * afe_set_timing_fast
   *
   * Using this function, you need to pass pointer to data (or just an array)
   * with specific timing values you want to write in timing registers.
   * !!!
   * 		 The most important thing is to write timing values in correct sequence (given in datasheet).
   * 		 If you give timing values in correct sequence, you do not need to worry about register addresses
   * !!!
   */
  afe_set_timing_fast(m_timing_data, sizeof(m_timing_data) / sizeof(uint16_t));
    /*	set led current on led1 and led2 (0 - 255)	*/
  afe_set_led_current(0x05, 0x3F);
  /***	set gain
   *
   *	amb_dac - value of cancellation current (0 - 10uA)
   * 	stg2gain - stage 2 gain (0dB - 12dB)
   *	cfLED - program capacity for LEDs (5pF - 150pF)
   *	rfLED - program resistance for LEDs (10kOhm - 1MOhm)
   *
   ***/
  s_tia_amb_gain _tia_amb_value =
  {
    .amb_dac  = AMB_DAC_1uA,
    .stg2gain = STG2GAIN_3dB,
    .cfLED    = CF_LED_15plus5pF,
    .rfLED    = RF_LED_50k
  };
  afe_set_gain(&_tia_amb_value);
    /*	call begin measure to turn leds and timers on  */
  afe_begin_measure();
}

ic_return_val_e ic_afe_init(void)
{
    /*	Initialize SPI by giving name of the handler and CS pin  */
  SPI_INIT(afe_spi_write, AFE4400_CS_PIN);
//  afe_disable_read();
  afe_reset();
  afe_conf();
    /*	enable reading from afe4400  */
//  afe_enable_read();
  return IC_SUCCESS;
}

ic_return_val_e ic_afe_deinit(void)
{
    /*  for sure, you can set led current on leds to 0  */
  afe_set_led_current(0, 0);
  afe_end_measure();
    /*  uninit spi interface  */
  SPI_UNINIT(afe_spi_write);

  return IC_SUCCESS;
}

//void afe_self_test()
//{
//
//}

/**********************************************************************************************************/
