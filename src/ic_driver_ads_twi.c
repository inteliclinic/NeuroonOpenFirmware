/**
 * @file    ic_driver_flash.c
 * @Author  iza
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */
#include <stdint.h>
#include <stddef.h>
#include "ic_driver_ads_twi.h"
#include "ic_driver_twi.h"


uint8_t config_frame[QUAN_FRAME];
uint8_t conversion_read_frame[QUAN_FRAME];


int main(void){
TWI_REGISTER (ADS);
TWI_INIT(ADS);

bool ads_init(){
				bool check_init = FALSE;
				uint16_t check_value = 0;
				uint8_t check_frame[QUAN_FRAME] = {0};

				/**Check communication I2C*/
				TWI_READ_DATA(ADS, ADS_TWI_ADDRESS, ADS_LO_THRESH, uint8_t *data, uint32_t data_length, callback);
				while (QUAN_FRAME != 0){
										check_value += check_frame[QUAN_FRAME];
										QUAN_FRAME--;
										}
				if(check_value == ADS_LO_THRESH) 	check_init = TRUE; //ADS_LO_THRESH value = 8000h
				else								check_init = FALSE;
				config_frame[0] = ADS_SINGLE_SHOT_CONV | ADS_PGA_FSR_02_256;
				config_frame[1] = 		ADS_DR_SPS_860 | ADS_COMP_QUE_11;
				TWI_SEND_DATA(ADS, ADS_TWI_ADDRESS, uint8_t *data, uint32_t data_length, callback);
				return check_init;
				}


}
}
