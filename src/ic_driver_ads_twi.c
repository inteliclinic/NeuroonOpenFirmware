/**
 * @file    ic_driver_flash.c
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
#include "ic_driver_ads_twi.h"

//
//struct {
//	uint16_t COMP_QUE	: 2;
//	uint16_t COMP_LAT : 1;
//	uint16_t COMP_POL : 1;
//	uint16_t COMP_MDE : 1;
//	uint16_t DR				: 3;
//	uint16_t MODE			: 1;
//	uint16_t PGA			: 3;
//	uint16_t MUX			: 3;
//	uint16_t OS				: 1;
//}conf_reg;
//static ic_return_val_e ret_val; //przyrownywac do twi_send data

uint16_t config_frame;
uint16_t conversion_read_frame;

typedef void (*twi_cb)(void);

void callback_twi(void *context){
	printf("TWI callback\r\n");

}
/**uint8_t conv_16_to_8(uint16_t *data, size_t len ){
 static uint8_t frame[2] = {0};
	frame[0] = ((uint8_t)(config_frame >> 8)) & 0b11111111;
	frame[1] = ((uint8_t)config_frame) & 0b11111111;
return ;
}
*/

TWI_REGISTER(ADS);

/**
 * @fn ads_init ()
 * @brief Examine communication I2C, initialization device
 *
 * @param[out]	error_write	fail to read/write.
 *
 * @return  if returned @ref error_write true it means that initialization succeded. In other cases
 * initialization attempt failed.
 */
bool ads_init (void){
	bool error_write = false;
	uint16_t check_value = 0;
	uint16_t check_frame = 0;
	TWI_INIT(ADS);

	/**Check communication I2C*/
	TWI_READ_DATA(ADS, ADS_TWI_ADDRESS, ADS_ADDR_LO_REG, (uint8_t *)&check_frame, ADS_REG_SIZE, callback_twi);
	check_value |= check_frame;
	if  (check_value == ADS_LO_THRESH) {
		error_write = true; //ADS_LO_THRESH value = 8000h
	}
	else {
		error_write = false;
	}
	config_frame = (((ADS_SINGLE_SHOT_CONV << ADS_OS_POS)		|
									(ADS_PGA_1 << ADS_PGA_POS) 							|
									(ADS_DR_860 << ADS_DR_POS) 							|
									(ADS_COMP_QUE_DIS << ADS_COMP_QUE_POS))	&
									(1111111011111111<<ADS_MODE_POS)); 			// ADS_MODE_CONT);
	TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, sizeof(config_frame), callback_twi);
	return error_write;
}

void ads_deinit(void){
  return;
}
/**
 * @fn ads_power_down ()
 * @brief ADS will be turned OFF
 */
void ads_power_down(void){
	config_frame = (ADS_MODE_POW_D << ADS_MODE_POS) | (0b00000000 << 0);
	TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, ADS_REG_SIZE, callback_twi);
}


/**
 * @fn ads_power_up ()
 * @brief ADS will be turned ON
 */
void ads_power_up(void){
  config_frame = (((ADS_SINGLE_SHOT_CONV << ADS_OS_POS)	|
						(ADS_PGA_4 << ADS_PGA_POS)									|
						(ADS_DR_250 << ADS_DR_POS)									|
						(ADS_COMP_QUE_DIS << ADS_COMP_QUE_POS))			&
						(1111111011111111<<ADS_MODE_POS)); // ADS_MODE_CONT
  TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, sizeof(config_frame), callback_twi);
}
/**
 *@fn ads_get_value ()
 *@brief Get conversion value
 *@return conversion value
 */
int16_t ads_get_value(void){
  int16_t conversion_read = 0;
  TWI_READ_DATA(ADS, ADS_TWI_ADDRESS, ADS_ADDR_CONV_REG, (uint8_t *)&conversion_read_frame, ADS_REG_SIZE, callback_twi);
	conversion_read = conversion_read_frame;
  return conversion_read;
}
/**@@fn ads_change_gain ()
 * @brief 			Amplifier will be set with provided gain
 *
 * @param[in] 	new_gain	0,1,2,4,8,16 gain value. Different values will return false.
 * @return  		if returned @ref error_write true it means that gain change succeded. Else, if returned false
 * 							it means that gain value is not valid.
 */
bool ads_change_gain(uint16_t new_gain){
	uint16_t new_gain_value = 0;
	bool error_write = false;
	bool check_write_config = false;

	if 		(new_gain == 0){
		new_gain_value = ADS_PGA_6;
	}
	else if (new_gain == 1){
		new_gain_value = ADS_PGA_4;
	}
	else if (new_gain == 2){
		new_gain_value = ADS_PGA_2;
	}
	else if (new_gain == 4){
		new_gain_value = ADS_PGA_1;
	}
	else if (new_gain == 8){
		new_gain_value = ADS_PGA_00_512;
	}
	else if (new_gain == 16){
		new_gain_value = ADS_PGA_01_256;
	}

	else {	return false;
	}
	config_frame = ((ADS_SINGLE_SHOT_CONV << ADS_OS_POS) |
					  (new_gain_value << ADS_PGA_POS));
	check_write_config = TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, ADS_REG_SIZE, callback_twi);

	if (check_write_config == 1)
		error_write = false;
	else {
		error_write = true;
	}
	return error_write;
}

/**@@fn ads_change_data_rate ()
 * @brief Function change data rate value
 *
 * @param[in] rate_code 	1-6 value of data rate. Different values will return false.
 *
 * @return  	if returned @ref error_write true it means that data rate succeded. Else, if returned false
 * 						it means that data rate value is not valid.
 */
bool ads_change_data_rate(uint16_t rate_code)
{
	uint8_t new_rate_value = 0;
	bool error_write = false;
	bool check_write_config = false;

	if (rate_code == 0) {
		new_rate_value = ADS_DR_8;
	}
	else if (rate_code == 1){
		new_rate_value = ADS_DR_16;
	}
	else if (rate_code == 2) {
		new_rate_value = ADS_DR_32;
	}
	else if (rate_code == 3) {
		new_rate_value = ADS_DR_64;
	}
	else if (rate_code == 4) {
		new_rate_value = ADS_DR_128;
	}
	else if (rate_code == 5) {
		new_rate_value = ADS_DR_250;
	}
	else if (rate_code == 6) {
		new_rate_value = ADS_DR_475;
	}
	else if (rate_code == 6) {
		new_rate_value = ADS_DR_860;
	}
	else {
		return false;
	}
	config_frame = ((ADS_COMP_QUE_DIS << ADS_COMP_QUE_POS) |
					   (new_rate_value << ADS_DR_POS));
	check_write_config = TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, (uint8_t *)&config_frame, ADS_REG_SIZE, callback_twi);

	if (!callback_twi()) {
			printf("TWI is busy");
			error_write = true;
	}
	else {
			error_write = false;
	}
	return error_write;
}


