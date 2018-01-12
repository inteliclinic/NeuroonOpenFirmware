/**
 * @file    ic_driver_bq27742_definitions.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    December, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_BQ27742_DEFINITIONS_H
#define IC_DRIVER_BQ27742_DEFINITIONS_H

							/** COMMANDS */
/** Standard commands */
#define BQ27742_CONTROL                     0x00
#define BQ27742_AT_RATE                     0x02
#define BQ27742_UNFILTERED_SOC              0x04
#define BQ27742_TEMPERATURE                 0x06
#define BQ27742_VOLTAGE                     0x08
#define BQ27742_FLAGS                       0x0A
#define BQ27742_NOM_AVAILABLE_CAPACITY      0x0C
#define BQ27742_FULL_AVAILABLE_CAPACITY     0x0E
#define BQ27742_REMAINING_CAPACITY          0x10
#define BQ27742_FULL_CHARGE_CAPACITY        0x12
#define BQ27742_AVERAGE_CURRENT             0x14
#define BQ27742_TIME_TO_EMPTY               0x16
#define BQ27742_FILTERED_FCC                0x18
#define BQ27742_SAFETY_STATUS               0x1A
#define BQ27742_UNFILTERED_FCC              0x1C
#define BQ27742_IMAX                        0x1E
#define BQ27742_UNFILTERED_RM               0x20
#define BQ27742_FILTERED_RM                 0x22
#define BQ27742_BTPSOC1_SET                 0x24
#define BQ27742_BTPSOC1_CLEAR               0x26
#define BQ27742_INTERNAL_TEMPERATURE        0x28
#define BQ27742_CYCLE_COUNT                 0x2A
#define BQ27742_STATE_OF_CHARGE	            0x2C
#define BQ27742_STATE_OF_HEALTH	            0x2E
#define BQ27742_CHARGING_VOLTAGE            0x30
#define BQ27742_CHARGING_CURRENT            0x32
#define BQ27742_PASSED_CHARGE               0x34
#define BQ27742_DODO                        0x36
#define BQ27742_SELF_DISCHARGE_CURRENT      0x38

/** Extended commands */
#define BQ27742_PACK_CONFIGRATION           0x3A
#define BQ27742_DESIGN_CAPACITY             0x3C
#define BQ27742_DATA_FLASH_CLASS            0x3E 		/** 1B command */
#define BQ27742_DATA_FLASH_BLOCK            0x3F		/** 1B command */
#define BQ27742_BLOCK_DATA                  0x40		/** 0x40 to 0x5F*/
#define BQ27742_BLOCK_DATA_CS               0x60		/** 1B command */
#define BQ27742_BLOCK_DATA_CONTROL          0x61		/** 1B command */
#define BQ27742_DOD_AT_EOC                  0x62
#define BQ27742_QSTART	                    0x64
#define BQ27742_FAST_Q_MAX                  0x66
#define BQ27742_PROTECT_STATUS              0x6D		/** 1B command */
#define BQ27742_SIMULTANEOUS_CURRENT        0x70
#define BQ27742_FET_TEST                    0x74
#define BQ27742_AVERAGE_POWER               0x76
#define BQ27742_PROTECTOR_STATE             0x78		/** 1B command */
#define BQ27742_AN_COUNTER                  0x79		/** 1B command */
#define BQ27742_AN_CURRENNT_LSB             0x7A		/** 1B command */
#define BQ27742_AN_CURRENT_MSB              0x7B		/** 1B command */
#define BQ27742_AN_VCELL_LSB                0x7C		/** 1B command */
#define BQ27742_AN_VCELL_MSB                0x7D		/** 1B command */
#define BQ27742_AN_TEMP_LSB                 0x7E		/** 1B command */
#define BQ27742_AN_TEMP_MSB                 0x7F		/** 1B command */

/** Control() subcommands */
#define BQ27742_CONTROL_STATUS              0x0000
#define BQ27742_DEVICE_TYPE                 0x0001
#define BQ27742_FW_VERSION                  0x0002
#define BQ27742_HW_VERSION                  0x0003
#define BQ27742_PROTECTOR_VERSION           0x0004
#define BQ27742_RESET_DATA                  0x0005
#define BQ27742_PREV_MACWRTIE               0x0007
#define BQ27742_CHEM_ID                     0x0008
#define BQ27742_BOARD_OFFSET                0x0009
#define BQ27742_CC_OFFSET                   0x000A
#define BQ27742_DF_VERSION                  0x000C
#define BQ27742_SET_FULLSLEEP               0x0010
#define BQ27742_SET_SHUTDOWN                0x0013
#define BQ27742_CLEAR_SHUTDOWN              0x0014
#define BQ27742_STATIC_CHEM_CHKSUM          0x0017
#define BQ27742_ALL_DF_CHKSUM               0x0018
#define BQ27742_STATIC_DF_CHKSUM            0x0019
#define BQ27742_PRTOECTOR_CHKSUM            0x001A
#define BQ27742_SEALED	                    0x0020
#define BQ27742_IT_ENABLE                   0x0021
#define BQ27742_IMAX_INT_CLEAR              0x0023
#define BQ27742_START_FET_TEST              0x0024
#define BQ27742_CAL_ENABLE                  0x002D
#define BQ27742_RESET	                    0x0041
#define BQ27742_EXIT_CAL                    0x0080
#define BQ27742_ENTER_CAL                   0x0081
#define BQ27742_OFFSET_CAL                  0x0082

/** REGISTER VALUES */
/** CONTROL STATUS */
typedef enum {
  BQ_SLEEP = 4,
  BQ_FULLSLEEP = 5,
  BQ_SHUTDOWN_EN = 7,
  BQ_SEALED_MODE = 13,
  BQ_FULL_SEALED_MODE = 14,
}control_status_flags;

#define BQ27742_DEVICE_TYPE_VALUE					0x0742
#define BQ27742_FW_VERSION_VALUE					0x0103

/** FETTest */
#define BQ27742_DSG_DIS								0x01
#define BQ27742_CHG_DIS								0x02
#define BQ27742_RECEN								0x04

/** FLAGS */
typedef enum {
  DSG = 0,
  SOCF,
  SOC1,
  CHG,
  IMAX,
  CHG_SUS = 7,
  FC = 9,
  CHG_INH = 10,
  BATLOW,
  BATHI
}general_flags;

/** SAFTY STATUS */
typedef enum {
  UVP_S = 0,
  OVP_S,
  OTD_S,
  OTC_S,
  TDD_S,
  ISD_S,
  INV_PROT_CHKSUM_S = 7
}safety_flags;

/** PROTECTOR STATUS */
typedef enum {
  OCC_P = 0,
  OCD_P,
  SCD_P,
  OVP_P,
  UVP_P,
  CVM_P,
  DSG_OFF,
  CHG_OFF
}protector_flags;

						/** DATA FLASH CONFIGURATION */
/** PARAMETER */
#define BQ27742_DATA_FLASH_BLOCK_SIZE       32
/** Flash block */
#define BQ27742_DATA_FLASH_BLOCK_0          0x00
#define BQ27742_REGISTER_WRITE_DELAY        2000 //powinien by c 2000
#define BQ27742_REGISTER_READ_DELAY         1000

/** SUBCLASS ID */
#define BQ27742_SAFETY_CLASS_ID                 0x02
#define BQ27742_CHARGE_CLASS_ID                 0x22
#define BQ27742_CHARGE_TERMINATION_CLASS_ID     0x24
#define BQ27742_JEITA_CLASS_ID                  0x27
#define BQ27742_DATA_CLASS_ID                   0x30
#define BQ27742_DISCHARGE_CLASS_ID              0x31
#define BQ27742_MANUFACTURER_DATA_CLASS_ID      0x38
#define BQ27742_INTEGRITY_CLASS_ID	        0x39
#define BQ27742_MANUFACTURER_INFO_CLASS_ID      0x3A
#define BQ27742_LIFETIME_DATA_CLASS_ID	        0x3B
#define BQ27742_LIFETIME_TEMP_SAMPLES_CLASS_ID  0x3C
#define BQ27742_REGISTERS_CLASS_ID              0x40
#define BQ27742_LIFETIME_RES_CLASS_ID           0x40
#define BQ27742_POWER_CLASS_ID                  0x44
#define BQ27742_IT_CFG_CLASS_ID                 0x50
#define BQ27742_CURRENT_TRESHOLD_CLASS_ID       0x51
#define BQ27742_STATE_CLASS_ID                  0x52
#define BQ27742_CHEM_ID_CLASS_ID                0x53
#define BQ27742_R_A0_CLASS_ID                   0x58
#define BQ27742_R_A0X_CLASS_ID                  0x59
#define BQ27742_CALIBRATION_CLASS_ID            0x68
#define BQ27742_CURRENT_CLASS_ID                0x6B
#define BQ27742_CODES_CLASS_ID                0x70

#define BQ27742_SAFETY_CLASS_SIZE           22
#define BQ27742_OV_PROT_THRESHOLD           4200
#define BQ27742_OV_PROT_DELAY               1
#define BQ27742_OV_PROT_RECOVERY            4100
#define BQ27742_UV_PROT_THRESHOLD           3100
#define BQ27742_UV_PROT_DELAY               5
#define BQ27742_UV_PROT_RECOVERY            3200
#define BQ27742_BODY_DIODE_THRESHOLD        60
#define BQ27742_OT_CHG                      500
#define BQ27742_OT_CHG_TIME                 5
#define BQ27742_OT_CHG_RECOVERY             450
#define BQ27742_OT_DSG                      500
#define BQ27742_OT_DSG_TIME                 5
#define BQ27742_OT_DSG_RECOVERY             450
//34
#define BQ27742_CHARGE_CLASS_SIZE           2
#define BQ27742_CHARGING_VOLTAGE_           4100
//36
#define BQ27742_TAPER_CURRENT	            150
#define BQ27742_MIN_TAPER_CAPACITY          25
#define BQ27742_TAPER_VOLTAGE	            100
#define BQ27742_CURRENT_TAPER_WINDOW        40
#define BQ27742_TCA_SET	                    -1
#define BQ27742_TCA_CLEAR_PROCENT           98
#define BQ27742_FC_SET_PROCENT              -1
#define BQ27742_FC_CLEAR_PROCENT            98
#define BQ27742_DODATEOC_DELTA_T            50
//39
#define BQ27742_T1_TEMP                     0
#define BQ27742_T2_TEMP                     10
#define BQ27742_T3_TEMP                     45
#define BQ27742_T4_TEMP                     50
#define BQ27742_T5_TEMP                     60
#define BQ27742_TEMP_HYS                    1
#define BQ27742_T1_T2_CHG_VOLTAGE           4350
#define BQ27742_T2_T3_CHG_VOLTAGE           4350
#define BQ27742_T3_T4_CHG_VOLTAGE           4300
#define BQ27742_T4_T5_CHG_VOLTAGE           4250
#define BQ27742_T1_T2_CHG_CURRENT           50
#define BQ27742_T2_T3_CHG_CURRENT           80
#define BQ27742_T3_T4_CHG_CURRENT           80
#define BQ27742_T4_T5_CHG_CURRENT           80
//48
#define BQ27742_DESIGN_VOLTAGE              3600
#define BQ27742_CYCLE_COUNT_                0
#define BQ27742_CC_THRESHOLD                450
#define BQ27742_DESIGN_CAPACITY_            500
#define BQ27742_DESIGN_ENERGY               1850
#define BQ27742_SOH_LOAD_I                  -400
#define BQ27742_TDD_SOH_PERCENT             80
#define BQ27742_ISD_CURRENT                 10
#define BQ27742_ISD_I_FILTER                127
#define BQ27742_MIN_ISD_TIME                7
#define BQ27742_DESIGN_ENERGY_SCALE         1
//49
#define BQ27742_SOC1_SET_THRESHOLD          150
#define BQ27742_SOC1_CLEAR_THRESHOLD        175
#define BQ27742_SOCF_SET_THRESHOLD          75
#define BQ27742_SOCF_CLEAR_THRESHOLD        100
#define BQ27742_BL_SET_VOLT_THRESHOLD       3200
#define BQ27742_BL_SET_VOLT_TIME            2
#define BQ27742_BL_CLEAR_VOLT_THRESHOLD     3300
#define BQ27742_BH_SET_VOLT_THRESHOLD       4000
#define BQ27742_BH_VOLT_TIME                2
#define BQ27742_BH_CLEAR_VOLT_THRESHOLD     3900
//56
#define BQ27742_PACK_LOT_CODE               0x0
#define BQ27742_PCB_LOT_CODE                0x0
#define BQ27742_FIRMWARE_VERSION            0x0
#define BQ27742_HARDWARE_REVISION           0x0
#define BQ27742_CELL_REVISION               0x0
#define BQ27742_DF_CONFIG_VERSION           0x0
//57
#define BQ27742_ALL_DF_CHECKSUM	            0x0
#define BQ27742_STATIC_CHEM_DF_CHECKSUM     0x7A9F
#define BQ27742_STATIC_DF_CHECKSUM          0x0
#define BQ27742_PROT_CHECKSUM               0x9
//59
#define BQ27742_LIFETIME_MAX_TEMP           315
#define BQ27742_LIFETIME_MIN_TEMP           220
#define BQ27742_LIFETIME_MAX_PACK_VOLTAGE   4087
#define BQ27742_LIFETIME_MIN_PACK_VOLTAGE   2781
#define BQ27742_LIFETIME_MAX_CHG_CURRENT    203
#define BQ27742_LIFETIME_MAX_DSG_CURRENT    -255
//60
#define BQ27742_LT_FLASH_CNT		    35
#define BQ27742_LT_AFE_STATUS               0x0
//64
#define BQ27742_PACK_CONFIGURATION          0x97F
#define BQ27742_PACK_CONFIGURATION_B        0xA7
#define BQ27742_PACK_CONFIGURATION_C        0xB9
#define BQ27742_PACK_CONFIGURATION_D        0xC3
#define BQ27742_PROT_OC_CONFIG              0x9
#define BQ27742_PROT_OV_CONFIG              0x0
//66
#define BQ27742_LT_TEMP_RES                 10
#define BQ27742_LT_V_RES                    25
#define BQ27742_LT_CUR_RES                  100
#define BQ27742_LT_UPDATE_TIME              60
//68
#define BQ27742_FLASH_UPDATE_OK_VOLTAGE	    2800
#define BQ27742_SLEEP_CURRENT               15
#define BQ27742_SHUTDOWN_V                  0
#define BQ27742_FS_WAIT                     0
//58
#define BQ27742_BLOCK_A_0                   0x0
#define BQ27742_BLOCK_A_1                   0x0
#define BQ27742_BLOCK_A_2                   0x0
#define BQ27742_BLOCK_A_3                   0x0
#define BQ27742_BLOCK_A_4                   0x0
#define BQ27742_BLOCK_A_5                   0x0
#define BQ27742_BLOCK_A_6                   0x0
#define BQ27742_BLOCK_A_7                   0x0
#define BQ27742_BLOCK_A_8                   0x0
#define BQ27742_BLOCK_A_9                   0x0
#define BQ27742_BLOCK_A_10                  0x0
#define BQ27742_BLOCK_A_11                  0x0
#define BQ27742_BLOCK_A_12                  0x0
#define BQ27742_BLOCK_A_13                  0x0
#define BQ27742_BLOCK_A_14                  0x0
#define BQ27742_BLOCK_A_15                  0x0
#define BQ27742_BLOCK_A_16                  0x0
#define BQ27742_BLOCK_A_17                  0x0
#define BQ27742_BLOCK_A_18                  0x0
#define BQ27742_BLOCK_A_19                  0x0
#define BQ27742_BLOCK_A_20                  0x0
#define BQ27742_BLOCK_A_21                  0x0
#define BQ27742_BLOCK_A_22                  0x0
#define BQ27742_BLOCK_A_23                  0x0
#define BQ27742_BLOCK_A_24                  0x0
#define BQ27742_BLOCK_A_25                  0x0
#define BQ27742_BLOCK_A_26                  0x0
#define BQ27742_BLOCK_A_27                  0x0
#define BQ27742_BLOCK_A_28                  0x0
#define BQ27742_BLOCK_A_29                  0x0
#define BQ27742_BLOCK_A_30                  0x0
#define BQ27742_BLOCK_A_31                  0x0
#define BQ27742_BLOCK_B_0                   0x0
#define BQ27742_BLOCK_B_1                   0x0
#define BQ27742_BLOCK_B_2                   0x0
#define BQ27742_BLOCK_B_3                   0x0
#define BQ27742_BLOCK_B_4                   0x0
#define BQ27742_BLOCK_B_5                   0x0
#define BQ27742_BLOCK_B_6                   0x0
#define BQ27742_BLOCK_B_7                   0x0
#define BQ27742_BLOCK_B_8                   0x0
#define BQ27742_BLOCK_B_9                   0x0
#define BQ27742_BLOCK_B_10                  0x0
#define BQ27742_BLOCK_B_11                  0x0
#define BQ27742_BLOCK_B_12                  0x0
#define BQ27742_BLOCK_B_13                  0x0
#define BQ27742_BLOCK_B_14                  0x0
#define BQ27742_BLOCK_B_15                  0x0
#define BQ27742_BLOCK_B_16                  0x0
#define BQ27742_BLOCK_B_17                  0x0
#define BQ27742_BLOCK_B_18                  0x0
#define BQ27742_BLOCK_B_19                  0x0
#define BQ27742_BLOCK_B_20                  0x0
#define BQ27742_BLOCK_B_21                  0x0
#define BQ27742_BLOCK_B_22                  0x0
#define BQ27742_BLOCK_B_23                  0x0
#define BQ27742_BLOCK_B_24                  0x0
#define BQ27742_BLOCK_B_25                  0x0
#define BQ27742_BLOCK_B_26                  0x0
#define BQ27742_BLOCK_B_27                  0x0
#define BQ27742_BLOCK_B_28                  0x0
#define BQ27742_BLOCK_B_29                  0x0
#define BQ27742_BLOCK_B_30                  0x0
#define BQ27742_BLOCK_B_31                  0x0
//80
#define BQ27742_LOAD_SELECT                 1
#define BQ27742_LOAD_MODE                   1
#define BQ27742_MAX_RES_FACTOR              15
#define BQ27742_MIN_RES_FACTOR              5
#define BQ27742_RA_FILTER                   800
#define BQ27742_RES_V_DROP                  50
#define BQ27742_FAST_QMAX_START_DOD_PROCENT 92
#define BQ27742_FAST_QMAX_END_DOD_PROCENT   96
#define BQ27742_FAST_QMAX_STAR_VOLT_DELTA   200
#define BQ27742_FAST_QMAX_CURRENT_THRESHOLD 4
#define BQ27742_QMAX_CAPACITY_ERR           15
#define BQ27742_MAX_QMAX_CHANGE             30
#define BQ27742_TERMINATE_VOLTAGE           3000
#define BQ27742_TERM_V_DELTA                200
#define BQ27742_RESRELAX_TIME               500
#define BQ27742_USER_RATE_MA                0
#define BQ27742_USER_RATE_PWR               0
#define BQ27742_RESERVE_CAP_MAH             0
#define BQ27742_MAX_DELTAV                  200
#define BQ27742_MIN_DELTAV                  0
#define BQ27742_MAX_SIM_RATE                1
#define BQ27742_MIN_SIM_RATE                20
#define BQ27742_RA_MAX_DELTA	            54
#define BQ27742_TRACE_RESISTANCE            0
#define BQ27742_DOWNSTREAM_RESISTANCE       0
#define BQ27742_QMAX_MAX_DELTA_PROCENT      5
#define BQ27742_QMAX_BOUND_PROCENT	    130
#define BQ27742_DELTAV_MAX_DELTA            10
#define BQ27742_MAX_RES_SCALE               5000
#define BQ27742_MIN_RES_SCALE               200
#define BQ27742_FAST_SCALE_START_SOC        10
#define BQ27742_FAST_SCALE_LOAD_SELECT      3
#define BQ27742_CHARGE_HYS_V_SHIFT          40
#define BQ27742_RASCL_OCV_RST_TEMP_THRESH   15
#define BQ27742_MAX_ALLOWED_CURRENT         8500
#define BQ27742_MAX_CURRENT_PULSE_DURATION  10
#define BQ27742_MAX_CURRENT_INTERRUPT_STEP  500
//81
#define BQ27742_DSG_CURRENT_THRESHOLD       60
#define BQ27742_CHG_CURRENT_THRESHOLD       75
#define BQ27742_QUIT_CURRENT                40
#define BQ27742_DSG_RELAX_TIME              60
#define BQ27742_CHG_RELAX_TIME              60
#define BQ27742_QUIT_RELAX_TIME             1
#define BQ27742_MAX_IR_CORRECT              400 //nie działa!!!!!!
//82
#define BQ27742_QMAX_CELL_0                 500
#define BQ27742_UPDATE_STATUS               0x2
#define BQ27742_V_AT_CHG_TERM               4076
#define BQ27742_AVG_I_LAST_RUN              -172
#define BQ27742_AVG_P_LAST_RUN              -579
#define BQ27742_DELTA_VOLTAGE               9
#define BQ27742_T_RISE                      50
#define BQ27742_T_TIME_CONSTANT             1000
//83
#define BQ27742_CHEM_ID_                    0x185
//88
#define BQ27742_CELL0_R_A_FLAG              0x555
#define BQ27742_CELL0_R_A_0                 41
#define BQ27742_CELL0_R_A_1                 44
#define BQ27742_CELL0_R_A_2                 47
#define BQ27742_CELL0_R_A_3                 64
#define BQ27742_CELL0_R_A_4                 47
#define BQ27742_CELL0_R_A_5                 45
#define BQ27742_CELL0_R_A_6                 46
#define BQ27742_CELL0_R_A_7                 45
#define BQ27742_CELL0_R_A_8                 45
#define BQ27742_CELL0_R_A_9                 47
#define BQ27742_CELL0_R_A_10                45
#define BQ27742_CELL0_R_A_11                52
#define BQ27742_CELL0_R_A_12                1640
#define BQ27742_CELL0_R_A_13                758
#define BQ27742_CELL0_R_A_14                13550
//89
#define BQ27742_XCELL0_R_A_FLAG             0xFFFF
#define BQ27742_XCELL0_R_A_0                41
#define BQ27742_XCELL0_R_A_1                44
#define BQ27742_XCELL0_R_A_2                47
#define BQ27742_XCELL0_R_A_3                64
#define BQ27742_XCELL0_R_A_4                47
#define BQ27742_XCELL0_R_A_5                45
#define BQ27742_XCELL0_R_A_6                46
#define BQ27742_XCELL0_R_A_7                45
#define BQ27742_XCELL0_R_A_8                45
#define BQ27742_XCELL0_R_A_9                47
#define BQ27742_XCELL0_R_A_10               45
#define BQ27742_XCELL0_R_A_11               52
#define BQ27742_XCELL0_R_A_12               85
#define BQ27742_XCELL0_R_A_13               95
#define BQ27742_XCELL0_R_A_14               6150
//104
// SZUKAJ ZMIENNYCH W PLIKU .c !!!!!!!!!!!!!!!!!!!!!!!

#define BQ27742_CC_OFFSET_                  1454
#define BQ27742_BOARD_OFFSET_               -68
#define BQ27742_INT_TEMP_OFFSET             0
#define BQ27742_EXT_TEMP_OFFSET             0
#define BQ27742_PACK_V_OFFSET               -99
//107
#define BQ27742_FILTER                      239
#define BQ27742_DEADBAND                    5
#define BQ27742_CC_DEADBAND                 34
//112
//liczby są za duże na define, trzeba zmieniać bezpośrednio w fukncji
#define BQ27742_SEALED_TO_UNSEALED          0x36720414
#define BQ27742_UNSEALED_TO_FULL            0xFFFFFFFF
#define BQ27742_AUTHEN_KEY3                 0x101234567
#define BQ27742_AUTHEN_KEY2                 0x89ABCDEF
#define BQ27742_AUTHEN_KEY1                 0xFEDCBA98
#define BQ27742_AUTHEN_KEY0                 0x76543210

#endif /* !IC_DRIVER_BQ27742_DEFINITIONS_H */
