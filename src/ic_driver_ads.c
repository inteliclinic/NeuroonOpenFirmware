/**
 * @file    ic_driver_ads.c
 * @Author  Izabela Poirier
 * @date    August, 2017
 * @brief   ADS1115 driver methods
 *
 * Functions provided with this file blablabla
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ic_driver_twi.h"
#include "ic_driver_ads.h"

#define NRF_LOG_MODULE_NAME "ADS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

struct __attribute__((packed)){
  const uint8_t conf_reg;
  union{
    struct{
      uint16_t mode       : 1;
      uint16_t pga        : 3;
      uint16_t mux        : 3;
      uint16_t os         : 1;
      uint16_t comp_que   : 2;
      uint16_t comp_lat   : 1;
      uint16_t comp_pol   : 1;
      uint16_t comp_mode  : 1;
      uint16_t dr         : 3;
    }bit_map;
    uint16_t data;
  }payload;
}m_config_frame = {.conf_reg = 0x01};

#define SWAP_2_BYTES(x) (x>>8) | (x<<8)
static bool m_ads_initialized = false;

static volatile uint16_t m_conversion_read_frame = 100;

typedef void (*twi_cb)(void);

TWI_REGISTER(ADS, ADS_TWI_ADDRESS);

/**
 * @fn ads_init ()
 * @brief Examine communication I2C, initialization device
 *
 * @param[out]	error_write	fail to read/write.
 *
 * @return  if returned @ref error_write true it means that initialization succeded. In other cases
 * initialization attempt failed.
 */
ic_return_val_e ic_ads_init (void){
  if(m_ads_initialized)
    return IC_SUCCESS;

  static uint16_t check_value = 0;

  TWI_INIT(ADS);

  /**Check communication I2C*/
  TWI_READ_DATA(ADS, ADS_ADDR_LO_REG, (uint8_t *)&check_value, ADS_REG_SIZE, NULL, NULL);

  check_value = SWAP_2_BYTES(check_value);

  NRF_LOG_INFO("check_value:0x%X, ADS_LO_THRESH:0x%X\n", check_value, ADS_LO_THRESH);

  if  (check_value != ADS_LO_THRESH) {
    m_ads_initialized = false;
    return IC_ERROR;
  }

  ic_ads_power_up();

  m_ads_initialized = true;

  return IC_SUCCESS;
}

void ic_ads_deinit(void){
  ic_ads_power_down();
  TWI_DEINIT(ADS);
  m_ads_initialized = false;
  return;
}

/**
 * @fn ic_ads_power_down ()
 * @brief ADS will be turned OFF
 */
void ic_ads_power_down(void){
  memset(&m_config_frame.payload.data, 0x00, sizeof(m_config_frame));
  m_config_frame.payload.bit_map.mode = ADS_MODE_POW_D;
  TWI_SEND_DATA(ADS, (uint8_t *)&m_config_frame, sizeof(m_config_frame), NULL, NULL);
}

/**
 * @fn ic_ads_power_up ()
 * @brief ADS will be turned ON
 */
void ic_ads_power_up(void){
  memset(&m_config_frame.payload.data, 0x00, sizeof(m_config_frame));

  m_config_frame.payload.bit_map.os       = ADS_SINGLE_SHOT_CONV;
  m_config_frame.payload.bit_map.pga      = ADS_PGA_1;
  m_config_frame.payload.bit_map.dr       = ADS_DR_860;
  m_config_frame.payload.bit_map.comp_que = ADS_COMP_QUE_DIS;
  m_config_frame.payload.bit_map.mode     = ADS_MODE_CONT;

  TWI_SEND_DATA(ADS, (uint8_t *)&m_config_frame, sizeof(m_config_frame), NULL, NULL);
}

static volatile void (*m_user_read_callback)(int16_t);

static void m_read_value_cb(ic_return_val_e ret_val, void *p_context){
  if(m_user_read_callback != NULL){
    m_user_read_callback(SWAP_2_BYTES(m_conversion_read_frame));
    m_user_read_callback = NULL;
  }
}

/**
 *@fn ads_get_value ()
 *@brief Get conversion value
 *@return conversion value
 */
ic_return_val_e ads_get_value(void (*p_read_callback)(int16_t), bool force){

  if (m_ads_initialized == false){
    return IC_ERROR;
  }

  if (m_user_read_callback == NULL || force)
    m_user_read_callback = p_read_callback;
  else
    return IC_BUSY;

  __auto_type _ret_val = IC_ERROR;

  if(force){
    _ret_val = TWI_READ_DATA(
        ADS,
        ADS_ADDR_CONV_REG,
        (uint8_t *)&m_conversion_read_frame,
        ADS_REG_SIZE,
        NULL,
        NULL
        );

    if(m_user_read_callback != NULL){
      m_user_read_callback(SWAP_2_BYTES(m_conversion_read_frame));
      m_user_read_callback = NULL;
    }
  }else{
    _ret_val = TWI_READ_DATA(
        ADS,
        ADS_ADDR_CONV_REG,
        (uint8_t *)&m_conversion_read_frame,
        ADS_REG_SIZE,
        m_read_value_cb,
        NULL
        );
  }

  if(_ret_val != IC_SUCCESS){
    ic_twi_refresh_bus();
  }

  return _ret_val;
}

/**@@fn ads_change_gain ()
 * @brief 			Amplifier will be set with provided gain
 *
 * @param[in] 	new_gain	0,1,2,4,8,16 gain value. Different values will return false.
 * @return  		if returned @ref error_write true it means that gain change succeded. Else, if returned false
 * 							it means that gain value is not valid.
 */
/*bool ads_change_gain(uint16_t new_gain){*/
	/*uint16_t new_gain_value = 0;*/
	/*bool error_write = false;*/
	/*bool check_write_config = false;*/

	/*if 		(new_gain == 0){*/
		/*new_gain_value = ADS_PGA_6;*/
	/*}*/
	/*else if (new_gain == 1){*/
		/*new_gain_value = ADS_PGA_4;*/
	/*}*/
	/*else if (new_gain == 2){*/
		/*new_gain_value = ADS_PGA_2;*/
	/*}*/
	/*else if (new_gain == 4){*/
		/*new_gain_value = ADS_PGA_1;*/
	/*}*/
	/*else if (new_gain == 8){*/
		/*new_gain_value = ADS_PGA_00_512;*/
	/*}*/
	/*else if (new_gain == 16){*/
		/*new_gain_value = ADS_PGA_01_256;*/
	/*}*/

	/*else {	return false;*/
	/*}*/
	/*config_frame = ((ADS_SINGLE_SHOT_CONV << ADS_OS_POS) |*/
					  /*(new_gain_value << ADS_PGA_POS));*/
	/*check_write_config = TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, ADS_REG_SIZE, callback_twi);*/

	/*if (check_write_config == 1)*/
		/*error_write = false;*/
	/*else {*/
		/*error_write = true;*/
	/*}*/
	/*return error_write;*/
/*}*/

/**@@fn ads_change_data_rate ()
 * @brief Function change data rate value
 *
 * @param[in] rate_code 	1-6 value of data rate. Different values will return false.
 *
 * @return  	if returned @ref error_write true it means that data rate succeded. Else, if returned false
 * 						it means that data rate value is not valid.
 */
/*
 *bool ads_change_data_rate(uint16_t rate_code)
 *{
 *        uint8_t new_rate_value = 0;
 *        bool error_write = false;
 *        bool check_write_config = false;
 *
 *        if (rate_code == 0) {
 *                new_rate_value = ADS_DR_8;
 *        }
 *        else if (rate_code == 1){
 *                new_rate_value = ADS_DR_16;
 *        }
 *        else if (rate_code == 2) {
 *                new_rate_value = ADS_DR_32;
 *        }
 *        else if (rate_code == 3) {
 *                new_rate_value = ADS_DR_64;
 *        }
 *        else if (rate_code == 4) {
 *                new_rate_value = ADS_DR_128;
 *        }
 *        else if (rate_code == 5) {
 *                new_rate_value = ADS_DR_250;
 *        }
 *        else if (rate_code == 6) {
 *                new_rate_value = ADS_DR_475;
 *        }
 *        else if (rate_code == 6) {
 *                new_rate_value = ADS_DR_860;
 *        }
 *        else {
 *                return false;
 *        }
 *        config_frame = ((ADS_COMP_QUE_DIS << ADS_COMP_QUE_POS) |
 *                                           (new_rate_value << ADS_DR_POS));
 *        check_write_config = TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, ADS_REG_SIZE, callback_twi);
 *
 *        if (!callback_twi()) {
 *                        printf("TWI is busy");
 *                        error_write = true;
 *        }
 *        else {
 *                        error_write = false;
 *        }
 *        return error_write;
 *}
 */

