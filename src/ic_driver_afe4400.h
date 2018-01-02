/**
 * @file    ic_driver_afe4400.h
 * @author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_DRIVER_AFE4400_H
#define _IC_DRIVER_AFE4400_H

#include <stdint.h>
#include <stddef.h>

#define AFE_NRF_DEBUG
#undef AFE_NRF_DEBUG


/** @defgroup LOW_LEVEL_DRIVER_AFE4400 low level driver for afe4400 module
 *  @{
 */

void callback_spi(void *context);

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
typedef struct __packed
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
#define READ_OPERATION (uint32_t)1<<24

/**
 * @brief Structure for holding led values
 *
 */
typedef struct __attribute__((packed))
{
  uint32_t led2_val;
  uint32_t aled2_val;
  uint32_t led1_val;
  uint32_t aled1_val;
  uint32_t diff_led2;
  uint32_t diff_led1;
}s_led_val;

typedef enum
{
  AFE_SUCCESSS,
  AFE_ERROR,
  AFE_OPERATION_ON_GOING
}ic_afe_ret_val;

typedef void (*event_cb_done)(s_led_val);

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
CF_LED_150plus5pF   = 0xF1   //!< CF_LED_150plus5pF
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
  e_cf_LED 	cfLED;
  e_rf_LED 	rfLED;
}s_tia_amb_gain;

/**
 * @}
 */

/** @defgroup AFE4400_FUNCTIONS_DECLARATION
 *  @ingroup LOW_LEVEL_DRIVER_AFE4400
 *  @{
 */
																					/*********	Functions declaration	*********/

/**
 * @brief AFE4400 initialization function
 *
 * Initialize SPI interface for AFE4400 and set module in reading mode (read from registers)
 */
void afe_init(void);

/**
 * @brief Deinitialization of AFE4400
 *
 */
void afe_deinit(void);

/**
 * @brief Enable read option in AFE4400
 */
ic_afe_ret_val afe_enable_read(void);

/**
 * @brief Disable read option (enable write option) in AFE4400
 */
ic_afe_ret_val afe_enable_write(void);

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
ic_afe_ret_val afe_write_reg(uint8_t regAddr, uint32_t regVal);

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
void afe_write_bit_reg(uint8_t regAddr, uint8_t bit, bool bit_high);

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
ic_afe_ret_val afe_read_reg(uint8_t regAddr, uint32_t *regVal);

/**
 * @brief Read all of the leds registers
 *
 * @param led_val - pointer to led values structure
 * @param cb      - callback for reading data when available
 *
 * @return IC_SUCCESS if everything goes okay
 *
 * Example:
 * @code
 *
 * led_val_s led_value;
 *
 * afe_read_led_reg(&led_value, my_callback_fun);
 *
 * @endcode
 */
ic_return_val_e afe_read_led_reg(event_cb_done cb);

/**
 * @brief Begin measure
 *
 * Function which sets all of the necessary bits in specific registers,
 * so you can start measuring pulse and oxidation data
 */
void afe_begin_measure(void);

/**
 * @brief End measures
 *
 * Function for deinitialization of afe4400 module
 * dealing with turning off timers and led current
 *
 */
void afe_end_measure(void);

/**
 * @brief Set deafult timing values in specific registers
 *
 * Function sets specific timing values given by the device producer in the AFE4400 datasheet
 */
void afe_set_default_timing(void);

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
void afe_set_led_current(uint8_t led1, uint8_t led2);

/**
 * @brief Software device reset
 */
void afe_reset(void);

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
void afe_set_gain(s_tia_amb_gain *tia_amb_value);

/**
 * @brief Check diagnostic register values
 *
 * @return value which can be checked for diagnostic
 */
uint16_t afe_check_diagnostic(void);

/**
 * @brief Set timing values in specific registers
 *
 * @param regAddr - register address
 * @param tim_val - timer value
 * @return true if everything is okay
 */
bool afe_set_timing_data(uint8_t regAddr, uint16_t tim_val);

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
void afe_set_timing_fast(uint32_t *timing_data, size_t data_len);

/**
 * @brief Convert uint32_t register value to the uint8_t array for pushing data via SPI in correct order
 *
 * @param regVal 	- register value
 * @param data 		- pointer to uint8_t array
 */
void convert(uint32_t regVal, uint8_t *data);

/**
 *  @}
 */
#endif /* !_IC_DRIVER_AFE4400_H */
