#ifndef SRC_IC_DRIVER_ADS_TWI_H_
#define SRC_IC_DRIVER_ADS_TWI_H_

/**
 * @file		ic_driver_ads_twi.h
 * @author	Izabela Poirer
 * @date		August, 2017
 * @brief		ADS1115 driver header
 */

/**SLAVE ADDRESS*/
#define ADS_TWI_ADDRESS						0b1001000

#define ADS_ADDR_RES_POS 					2			//Reserved - always write 0h

/**REGISTER ADDRESS POINTER*/
#define ADS_ADDR_POS 							0
#define ADS_ADDR_CONV_REG 				0b00	//00 : Conversion register
#define ADS_ADDR_CONF_REG					0b01	//01 : Config register
#define ADS_ADDR_LO_REG						0b10	//10 : Lo_thresh register
#define ADS_ADDR_HI_REG						0b11	//11 : Hi_thresh register

/** CONVERSION REGISTER*/
#define ADS_REG_FIELD_POS					0			//16-bit conversion result


/**CONFIGURATION REGISTER*/
#define ADS_SINGLE_SHOT_CONV			0b1

/** Operational status or single-shot conversion start
*This bit determines the operational status of the device.
*OS can only be written when in power-down state and has no effect when
*a conversion is ongoing*/
#define ADS_OS_POS								15
#define ADS_OS_R0									0b0		//when reading 0 : Device is currently performing  a conversion
#define ADS_OS_R1									0b1 	//when reading 1 : Device is not currently performing a conversion
#define ADS_OS_W0									0b0		//When writing 0 : No effect
#define ADS_OS_W1									0b1   //When writing 1 : Start a single conversion (when in power-down state)


/** Input multiplexer configuration (ADS1115 only)
*These bits configure the input multiplexer.*/
#define ADS_MUX_POS								12
#define ADS_MUX_P0_N1							0b000	//000 : AINP = AIN0 and AINN = AIN1 (default)
#define ADS_MUX_P0_N3							0b001 //001 : AINP = AIN0 and AINN = AIN3
#define ADS_MUX_P1_N3							0b010	//010 : AINP = AIN1 and AINN = AIN3
#define ADS_MUX_P2_N3							0b011	//011 : AINP = AIN2 and AINN = AIN3
#define ADS_MUX_P0_NGND						0b100	//100 : AINP = AIN0 and AINN = GND
#define ADS_MUX_P1_NGND						0b101	//101 : AINP = AIN1 and AINN = GND
#define ADS_MUX_P2_NGND						0b110	//110 : AINP = AIN2 and AINN = GND
#define ADS_MUX_P3_NGND						0b111	//111 : AINP = AIN3 and AINN = GND

/**Programmable gain amplifier configuration
*These bits set the FSR of the programmable gain amplifier.*/
#define ADS_PGA_POS								9
#define ADS_PGA_6									0b000	//000 : FSR = ±6.144V
#define ADS_PGA_4									0b001	//001 : FSR = ±4.096V
#define ADS_PGA_2									0b010	//010 : FSR = ±2.048V (default)
#define ADS_PGA_1									0b011	//011 : FSR = ±1.024V
#define ADS_PGA_00_512						0b100	//100 : FSR = ±0.512V
#define ADS_PGA_01_256						0b101	//101 : FSR = ±0.256V
#define ADS_PGA_02_256						0b110	//110 : FSR = ±0.256V
#define ADS_PGA_03_256						0b111	//111 : FSR = ±0.256V

/** Device operating mode
*This bit controls the operating mode.*/
	#define ADS_MODE_POS						8
	#define ADS_MODE_CONT						0b0		//0 : Continuous-conversion mode
	#define ADS_MODE_POW_D					0b1		//1 : Single-shot mode or power-down state (default)

/** Data rate
*These bits control the data rate setting*/
#define ADS_DR_POS								5
#define ADS_DR_8									0b000	//000 : 8   SPS
#define ADS_DR_16									0b001	//001 : 16  SPS
#define ADS_DR_32									0b010	//010 : 32  SPS
#define ADS_DR_64									0b011	//011 : 64  SPS
#define ADS_DR_128								0b100	//100 : 128 SPS
#define ADS_DR_250								0b101	//101 : 250 SPS
#define ADS_DR_475								0b110	//110 : 475 SPS
#define ADS_DR_860								0b111	//111 : 860 SPS

/** Comparator mode
*This bit configures the comparator operating mode.*/
#define ADS_COMP_MODE_POS					4
#define ADS_COMP_MODE_0						0b0 	//0 :  Traditional comparator (default)
#define ADS_COMP_MODE_1						0b1		//1 : Window comparator

/** Comparator polarity
*This bit controls the polarity of the ALERT/RDY pin.*/
#define ADS_COMP_POL_POS					3
#define ADS_COMP_POL_0						0b0 	//0 : Active low (default)
#define ADS_COMP_POL_1						0b1		//1 : Active high

/** Latching comparator
*This bit controls whether the ALERT/RDY pin latches after being asserted or clears
*after conversions are within the margin of the upper and lower threshold values.*/
#define ADS_COMP_LAT_POS					3
#define ADS_COMP_LAT_0						0b0 	//0 : Nonlatching comparator . The ALERT/RDY pin does not latch when asserted (default)
#define ADS_COMP_LAT_1						0b1		//1 : Latching comparator. The asserted ALERT/RDY pin remains latched until conversion
																				//		data are read by the master or an appropriate SMBus alert response is sent by the master.
																				//		The device responds with its address, and it is the lowest address currently asserting the ALERT/RDY bus line.

/** Comparator queue and disable
*These bits perform two functions.
*When set to 11, the comparator is disabled and the ALERT/RDY pin is set to a high-impedance state.
*When set to any other value, the ALERT/RDY pin and the comparator function are enabled,
*and the set value determines the number of successive conversions exceeding  the upper or lower
*threshold required before asserting  the ALERT/RDY pin. */
#define ADS_COMP_QUE_POS				3
#define ADS_COMP_QUE_00					0b00 	//00 : Assert after one conversion
#define ADS_COMP_QUE_01					0b01	//01 : Assert after two conversions
#define ADS_COMP_QUE_10					0b10	//10 : Assert after four conversions
#define ADS_COMP_QUE_DIS				0b11	//11 : 11 : Disable comparator and set ALERT/RDY pin to high-impedance (default)


/**Lo_thresh and Hi_thresh
 * Register Field Descriptions*/
#define ADS_LO_THRESH						0x8000//Low threshold value 0b1000000000000000
#define ADS_HI_THRESH						0x7FFF//High threshold value

#define ADS_REG_SIZE						2



bool ads_init(void);
void ads_deinit();
void ads_power_down();
void ads_power_up(void);
void callback_twi(void *context);

int16_t ads_get_value();

bool ads_change_gain(uint16_t new_gain);
bool ads_change_data_rate(uint16_t rate_code);


#endif /* SRC_IC_DRIVER_ADS_TWI_H_ */
