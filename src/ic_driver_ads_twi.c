/**
 * @file    ic_driver_flash.c
 * @Author  iza
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "ic_driver_ads_twi.h"
#include "ic_driver_twi.h"
#include <stdio.h>

uint16_t config_frame;
uint16_t convertion_read_frame[QUAN_FRAME];
//static ic_return_val_e ret_val; //przyrownywac do twi_send data



typedef void (*twi_cb)(void);

void callback_twi(void *context){
//	UNUSED_VARIABLE(context);
	printf("TWI callback\r\n");
}

/**bool split_frame(uint16_t *data, uint32_t data_length){
	bool split_frame = FALSE;

	split_frame = TRUE;
	return split_frame;
}*/

static ads_send_data(uint16_t data, reg_addr){
	uint8_t frame[1] = ((uint16_t)config_frame >> 8)& 0b11111111;
	uint8_t frame[2] =  (uint16_t)config_frame & 0b11111111;
	if(callback_twi != 0){


	}
}

bool ads_init(){
				bool check_init = false;
				uint16_t check_value = 0;
				uint16_t check_frame[QUAN_FRAME] = {0};

				/**Check communication I2C*/
				TWI_READ_DATA(ADS, ADS_TWI_ADDRESS, ADS_LO_THRESH, check_frame, ADS_REG_SIZE, callback_twi);
				while (QUAN_FRAME != 0){
										check_value |= check_frame[QUAN_FRAME];
										QUAN_FRAME--;
										}
				if(check_value == ADS_LO_THRESH) 	check_init = true; //ADS_LO_THRESH value = 8000h
				else								check_init = false;
				config_frame = ((ADS_SINGLE_SHOT_CONV << ADS_OS_POS) |
								   (ADS_PGA_1 << ADS_PGA_POS) 			|
								   (ADS_DR_860 << ADS_DR_POS) 		|
								   (ADS_COMP_QUE_DIS << ADS_COMP_QUE_POS));
				TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, *data, ADS_REG_SIZE, callback_twi);
				return check_init;
				}

void ads_deinit(){
			 	 return;
}

void ads_power_down(){
				config_frame = (ADS_MODE_POW_D << ADS_MODE_POS) |
								  (0b00000000 << 0);
				TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, data, ADS_REG_SIZE, callback_twi);
}

void ads_power_up(void){
					config_frame = ((ADS_SINGLE_SHOT_CONV << ADS_OS_POS) |
									   (ADS_PGA_4 << ADS_PGA_POS)			|
									   (ADS_DR_250 << ADS_DR_POS)		|
									   (ADS_COMP_QUE_DIS << ADS_COMP_QUE_POS));
				TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, data, len, callback_twi);

}
/** get conversion value
 * return conversion value
 */
int16_t ads_get_value(){
		int16_t convertion_read = 0;
		TWI_READ_DATA(ADS, ADS_TWI_ADDRESS, ADS_ADDR_CONV_REG, conversion_read_frame, ADS_REG_SIZE, callback_twi);

		//convertion_read_frame[0] =
		convertion_read = convertion_read_frame;

		return convertion_read;
}

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
	check_write_config = TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, data, ADS_REG_SIZE, callback_twi);

	if (check_write_config == 1)
		error_write = false;
	else
		error_write = true;
	return error_write;
}

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
	check_write_config = TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, data, ADS_REG_SIZE, callback_twi);

	if (check_write_config == 1)
			error_write = false;
	else {
			error_write = true;
	}
	return error_write;


}


