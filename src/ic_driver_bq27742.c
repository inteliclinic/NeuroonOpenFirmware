/**
 * @file    ic_driver_bq27742.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    December, 2017
 * @brief   Brief description
 *
 * Description
 */

/*#include "ic_driver_bq27742.h"*/
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include "ic_driver_twi.h"
#include "ic_service_ltc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
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
#define BQ27742_SAFETY_CLASS                0x02
#define BQ27742_CHARGE_CLASS                0x22
#define BQ27742_CHARGE_TERMINATION_CLASS    0x24
#define BQ27742_DATA_CLASS                  0x30
#define BQ27742_DISCHARGE_CLASS             0x31
#define BQ27742_MANUFACTURER_DATA_CLASS     0x38
#define BQ27742_INTEGRITY_CLASS	            0x39
#define BQ27742_MANUFACTURER_INFO1_CLASS    0x3A
#define BQ27742_LIFETIME_DATA_CLASS	    0x3B
#define BQ27742_LIFETIME_TEMP_SAMPLES_CLASS 0x3C
#define BQ27742_REGISTERS_CLASS             0x40
#define BQ27742_LIFETIME_RES_CLASS          0x40
#define BQ27742_POWER_CLASS                 0x44
#define BQ27742_IT_CFG_CLASS                0x50
#define BQ27742_CURRENT_TRESHOLD_CLASS      0x51
#define BQ27742_CALIBRATION_CLASS           0x68

#define BQ27742_SAFETY_CLASS_SIZE           22
#define BQ27742_SAFETY_CLASS_ID	            2
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
#define BQ27742_CHARGE_CLASS_ID             34
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

const float BQ27742_CC_GAIN=4.768/54.931;
const float BQ27742_CC_DELTA=5677445.0/54.810;

/*#define FORCE_ENDIANNESS __ORDER_LITTLE_ENDIAN__*/
#define FORCE_ENDIANNESS __ORDER_BIG_ENDIAN__

#define SWAP_2BYTES(val) (                                    \
    (((*(typeof(val) *)&(val)) >> CHAR_BIT) & 0x00FF) |       \
    (((*(typeof(val) *)&(val)) << CHAR_BIT) & 0xFF00) )

#define SWAP_4BYTES(val) (                                    \
    (((*(typeof(val) *)&(val)) >> CHAR_BIT*3) & 0x000000FF) | \
    (((*(typeof(val) *)&(val)) >> CHAR_BIT)   & 0x0000FF00) | \
    (((*(typeof(val) *)&(val)) << CHAR_BIT)   & 0x00FF0000) | \
    (((*(typeof(val) *)&(val)) << CHAR_BIT*3) & 0xFF000000) )

#if FORCE_ENDIANNESS == __BYTE_ORDER__
  #define ENSURE_ENDIANNESS(T) _Generic((T),                    \
      float: *(uint32_t *)&T,                                   \
      default: T                                                \
      )
#else
  #define ENSURE_ENDIANNESS(T) _Generic((T),                    \
      uint8_t: T,                                               \
      int8_t: T,                                                \
      uint16_t: SWAP_2BYTES(T),                                 \
      uint32_t: SWAP_4BYTES(T),                                 \
      int16_t: SWAP_2BYTES(T),                                  \
      int32_t: SWAP_4BYTES(T),                                  \
      float: SWAP_4BYTES(T),                                    \
      default: 0                                                \
      )
#endif

#define bq_subclass(structure_name) union{                      \
  struct structure_name data_structure;                         \
  uint8_t raw[sizeof(struct structure_name)];                   \
}bq_subclass_ ## structure_name;

#define DEFINE_DATA_STRUCT(name, code)                          \
  union{                                                        \
    struct __attribute__((packed)) name##_s{                    \
      struct __attribute__((packed))                            \
      code payload;                                             \
    }data_struct;                                               \
    unsigned char data[sizeof(struct name##_s)];                \
  }name##_instance

#define GET_RAW_DATA(name) name##_instance.data

#define GET_DATA_STRUCT(name) name##_instance.data_struct.payload

#define BQ_SEND_DATA(subclass_id, flash_block, flash_data_access, block_data_access, data, size) do{  \
  uint8_t _reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};                                           \
  set_bq_register(BQ27742_BLOCK_DATA_CONTROL, flash_data_access);                                     \
  set_bq_register(BQ27742_DATA_FLASH_CLASS, subclass_id);                                             \
  set_bq_register(BQ27742_DATA_FLASH_BLOCK, flash_block);                                             \
  set_bq_register_var(block_data_access, data, size);                                                 \
  TWI_READ_DATA(BQ, BQ27742_BLOCK_DATA, _reg_value, sizeof(_reg_value), NULL, NULL);                  \
  uint32_t _flash_data_sum = 0;                                                                       \
  for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++) _flash_data_sum += _reg_value[i];           \
  uint8_t checksum = 255 - (uint8_t)(_flash_data_sum);                                                \
  set_bq_register(BQ27742_BLOCK_DATA_CS, checksum);                                                   \
  vTaskDelay(pdMS_TO_TICKS(BQ27742_REGISTER_WRITE_DELAY));                                            \
}while(0)

TWI_REGISTER(BQ, IC_BQ27742_TWI_ADDRESS);


static ic_return_val_e set_bq_register_var(uint8_t reg_addr, uint8_t *reg_val, size_t len){
  uint8_t _buffer[len+1];
  _buffer[0] = reg_addr;

  for(int i = 1; i<len+1; ++i){
    _buffer[i] = *reg_val++;
  }

  return TWI_SEND_DATA(BQ, _buffer, sizeof(_buffer), NULL, NULL);
}

static ic_return_val_e set_bq_register(uint8_t reg_addr, uint8_t reg_val){
  return set_bq_register_var(reg_addr, &reg_val, sizeof(reg_val));
}

/**
 * @fn bq27742_data_flash_write(uint8_t subclass_id, uint8_t flash_block, uint8_t offset, uint16_t value)
 * @brief write data to the data flash
 *  param[in] subclass_id: id number of configuration class
 *  param[in] flash_block: number of page in flash block (block more than 32B)
 *  param[in] offset: index into the BlockDate() command space
 *  param[in] value: data to write
 *  param[in] reg size: size of register (1 - 1B; 2 - 2B) depends on the size of value to write
 */
static void bq27742_data_flash_write(
    uint8_t subclass_id,
    uint8_t flash_block,
    uint8_t offset,
    uint8_t *value,
    size_t len)
{
  BQ_SEND_DATA(
      subclass_id,                    // subclass_ID
      flash_block,                    // flash_block
      0x00,                           // flash_data_access
      (BQ27742_BLOCK_DATA + offset),  // block_data_access
      value,                          // data
      len);                           // size
}

float fast_power(float base, int16_t index){
  if (index<0)
    return fast_power(base, ++index) * 1/base;
  return index == 0 ? 1 : fast_power(base, --index) * base;
}

uint32_t float_to_bq_format (float val)
{
  const float _2pow_25 = 2.98023224e-8;
  union {
    uint32_t raw;
    uint8_t  byte[4];
  } ret_val;

  int16_t exp=0;
  float mod_val;
  float tmp_val;

  mod_val=fabsf(val);

  tmp_val=mod_val*(1.0 + _2pow_25);

  if(tmp_val <0.5)
  {
    do
    {
      tmp_val*=2;
      exp--;
    } while(tmp_val<0.5);
  }
  else if(tmp_val>=1.0)
  {
    do
    {
      tmp_val/=2;
      exp++;
    } while(tmp_val>=1.0);
  }

  if(exp>127)
    exp=127;
  else if(exp<-128)
    exp=-128;

  tmp_val=(fast_power(2.0,(8-exp))*mod_val)-128.0;

  ret_val.byte[3]=exp+128;
  ret_val.byte[2]=(uint8_t)tmp_val;
  tmp_val=256*(tmp_val-ret_val.byte[2]);
  ret_val.byte[1]=(uint8_t)tmp_val;
  tmp_val=256*(tmp_val-ret_val.byte[1]);
  ret_val.byte[0]=(uint8_t)tmp_val;

  if(val<0)
    ret_val.byte[2] |= 0x80;

  return ret_val.raw;
}


void bq27742_program_flash_subclass_data_104()
{
  DEFINE_DATA_STRUCT(data_104, {
    uint32_t cc_gain;
    uint32_t cc_delta;
    uint16_t cc_offset;
    uint8_t board_offset;
    uint8_t int_temp_offset;
    uint8_t ext_temp_offset;
    uint8_t pack_v_offset;
  });

  GET_DATA_STRUCT(data_104).cc_gain         = float_to_bq_format(BQ27742_CC_GAIN);
  GET_DATA_STRUCT(data_104).cc_delta        = float_to_bq_format(BQ27742_CC_DELTA);
  GET_DATA_STRUCT(data_104).cc_offset       = BQ27742_CC_OFFSET_;
  GET_DATA_STRUCT(data_104).board_offset    = BQ27742_BOARD_OFFSET_;
  GET_DATA_STRUCT(data_104).int_temp_offset = BQ27742_INT_TEMP_OFFSET;
  GET_DATA_STRUCT(data_104).ext_temp_offset = BQ27742_EXT_TEMP_OFFSET;
  GET_DATA_STRUCT(data_104).pack_v_offset   = BQ27742_PACK_V_OFFSET;

  GET_DATA_STRUCT(data_104).cc_gain =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).cc_gain);
  GET_DATA_STRUCT(data_104).cc_delta =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).cc_delta);
  GET_DATA_STRUCT(data_104).cc_offset =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).cc_offset);
  GET_DATA_STRUCT(data_104).board_offset =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).board_offset);
  GET_DATA_STRUCT(data_104).int_temp_offset =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).int_temp_offset);
  GET_DATA_STRUCT(data_104).ext_temp_offset =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).ext_temp_offset);
  GET_DATA_STRUCT(data_104).pack_v_offset =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(data_104).pack_v_offset);

  set_bq_register(BQ27742_BLOCK_DATA_CONTROL, 0x00);
  set_bq_register(BQ27742_DATA_FLASH_CLASS, 104);
  set_bq_register(BQ27742_DATA_FLASH_BLOCK, 0x00);

  uint8_t _reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};

  TWI_SEND_DATA(BQ, GET_RAW_DATA(data_104), sizeof(GET_RAW_DATA(data_104)), NULL, NULL);
  TWI_READ_DATA(BQ, BQ27742_BLOCK_DATA, _reg_value, sizeof(_reg_value), NULL, NULL);

  uint32_t _flash_date_sum = 0;
  for (int i = 0; i < sizeof(_reg_value); ++i)
    _flash_date_sum +=_reg_value[i];

  uint8_t _checksum = 255 - (uint8_t)(_flash_date_sum);
  set_bq_register(BQ27742_BLOCK_DATA_CS, _checksum);

  vTaskDelay(pdMS_TO_TICKS(BQ27742_REGISTER_WRITE_DELAY));
}

/**
 * @fn bq27742_read_control_data (uint16_t subcommand)
 * @brief read data from  control register
 *  param[in] subcommand: data subcommand code
 *  param[out] read control data
 */
uint16_t bq27742_read_control_data (uint16_t subcommand)
{
  uint16_t return_value =0;

  set_bq_register_var(BQ27742_CONTROL, (uint8_t *)&subcommand, sizeof(subcommand));
  vTaskDelay(pdMS_TO_TICKS(1));

  TWI_READ_DATA(BQ, BQ27742_CONTROL, (uint8_t *)&return_value, sizeof(return_value), NULL, NULL);

  return return_value;
}

/**
 * @fn bq27742_control_status_read ()
 * @brief read data from control status register
 *  param[out] register value
 */
uint16_t bq27742_control_status_read(void)
{
  return bq27742_read_control_data(BQ27742_CONTROL_STATUS);
}

bool bq27742_unsaled_set(void)
{
  uint8_t key1 [] = {0x14, 0x04};
  uint8_t key2 [] = {0x72, 0x36};

  set_bq_register_var(BQ27742_CONTROL, key1, sizeof(key1));
  vTaskDelay(pdMS_TO_TICKS(1));
  set_bq_register_var(BQ27742_CONTROL, key2, sizeof(key2));
  vTaskDelay(pdMS_TO_TICKS(1));

  return (bq27742_control_status_read() & (1<<BQ_FULL_SEALED_MODE)) ? true : false;
}

/**
 * @fn  bq27742_full_access_set()
 * @brief put device into full access state
 *  param[out] return true if device enter full access state
*/
bool bq27742_full_access_set(void)
{
  uint8_t key1 [] = {0xFF,0xFF};
  uint8_t key2 [] = {0xFF,0xFF};

  set_bq_register_var(BQ27742_CONTROL, key1, sizeof(key1));
  vTaskDelay(pdMS_TO_TICKS(BQ27742_REGISTER_WRITE_DELAY));
  set_bq_register_var(BQ27742_CONTROL, key2, sizeof(key2));
  vTaskDelay(pdMS_TO_TICKS(BQ27742_REGISTER_WRITE_DELAY));

  return (bq27742_control_status_read() & (1<<BQ_FULL_SEALED_MODE)) ? true : false;
}

void bq27742_program_flash_subclass_safety(){
  DEFINE_DATA_STRUCT(safety, {
      uint16_t  ov_prot_trshold;
      uint8_t   ov_prot_delay;
      uint16_t  ov_prot_recovery;
      uint16_t  uv_prot_threshold;
      uint8_t   uv_prot_delay;
      uint16_t  uv_prot_recovery;
      uint16_t  body_diode_threshold;
      uint16_t  ot_chg;
      uint8_t   ot_chg_time;
      uint16_t  ot_chg_recovery;
      uint16_t  ot_dsg;
      uint8_t   ot_dsg_time;
      uint16_t  ot_dsg_recovery;
      });

  GET_DATA_STRUCT(safety).ov_prot_trshold       = BQ27742_OV_PROT_THRESHOLD;
  GET_DATA_STRUCT(safety).ov_prot_delay         = BQ27742_OV_PROT_DELAY;
  GET_DATA_STRUCT(safety).ov_prot_recovery      = BQ27742_OV_PROT_RECOVERY;
  GET_DATA_STRUCT(safety).uv_prot_threshold     = BQ27742_UV_PROT_THRESHOLD;
  GET_DATA_STRUCT(safety).uv_prot_delay         = BQ27742_UV_PROT_DELAY;
  GET_DATA_STRUCT(safety).uv_prot_recovery      = BQ27742_UV_PROT_RECOVERY;
  GET_DATA_STRUCT(safety).body_diode_threshold  = BQ27742_BODY_DIODE_THRESHOLD;
  GET_DATA_STRUCT(safety).ot_chg                = BQ27742_OT_CHG;
  GET_DATA_STRUCT(safety).ot_chg_time           = BQ27742_OT_CHG_TIME;
  GET_DATA_STRUCT(safety).ot_chg_recovery       = BQ27742_OT_CHG_RECOVERY;
  GET_DATA_STRUCT(safety).ot_dsg                = BQ27742_OT_DSG;
  GET_DATA_STRUCT(safety).ot_dsg_time           = BQ27742_OT_DSG_TIME;
  GET_DATA_STRUCT(safety).ot_dsg_recovery       = BQ27742_OT_DSG_RECOVERY;

  GET_DATA_STRUCT(safety).ov_prot_trshold       = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ov_prot_trshold);
  GET_DATA_STRUCT(safety).ov_prot_delay         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ov_prot_delay);
  GET_DATA_STRUCT(safety).ov_prot_recovery      = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ov_prot_recovery);
  GET_DATA_STRUCT(safety).uv_prot_threshold     = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).uv_prot_threshold);
  GET_DATA_STRUCT(safety).uv_prot_delay         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).uv_prot_delay);
  GET_DATA_STRUCT(safety).uv_prot_recovery      = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).uv_prot_recovery);
  GET_DATA_STRUCT(safety).body_diode_threshold  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).body_diode_threshold);
  GET_DATA_STRUCT(safety).ot_chg                = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ot_chg);
  GET_DATA_STRUCT(safety).ot_chg_time           = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ot_chg_time);
  GET_DATA_STRUCT(safety).ot_chg_recovery       = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ot_chg_recovery);
  GET_DATA_STRUCT(safety).ot_dsg                = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ot_dsg);
  GET_DATA_STRUCT(safety).ot_dsg_time           = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ot_dsg_time);
  GET_DATA_STRUCT(safety).ot_dsg_recovery       = ENSURE_ENDIANNESS(GET_DATA_STRUCT(safety).ot_dsg_recovery);

  bq27742_data_flash_write(
      BQ27742_SAFETY_CLASS_ID,
      0x00,
      0x00,
      GET_RAW_DATA(safety),
      sizeof(GET_RAW_DATA(safety))
      );
}

void bq27742_program_flash_subclass_charge(){

  DEFINE_DATA_STRUCT(charge, {
      uint16_t charging_voltage;
      });

  GET_DATA_STRUCT(charge).charging_voltage         = BQ27742_CHARGING_VOLTAGE_;

  GET_DATA_STRUCT(charge).charging_voltage         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge).charging_voltage);

  bq27742_data_flash_write(
      BQ27742_CHARGE_CLASS_ID,
      0x00,
      0x00,
      GET_RAW_DATA(charge),
      sizeof(GET_RAW_DATA(charge))
      );

}

void bq27742_program_flash_subclass_charge_termination()
{
  DEFINE_DATA_STRUCT(charge_termination,  {
      uint16_t  taper_current;
      uint16_t  min_taper_capacity;
      uint16_t  taper_voltage;
      uint8_t   current_taper_window;
      uint8_t   tca_set;
      uint8_t   tca_clear_procent;
      uint8_t   fc_set_procent;
      uint8_t   fc_clear_procent;
      uint16_t  dodateoc_delta_t;
      });

  GET_DATA_STRUCT(charge_termination).taper_current         = BQ27742_TAPER_CURRENT;
  GET_DATA_STRUCT(charge_termination).min_taper_capacity    = BQ27742_MIN_TAPER_CAPACITY;
  GET_DATA_STRUCT(charge_termination).taper_voltage         = BQ27742_TAPER_VOLTAGE;
  GET_DATA_STRUCT(charge_termination).current_taper_window  = BQ27742_CURRENT_TAPER_WINDOW;
  GET_DATA_STRUCT(charge_termination).tca_set               = BQ27742_TCA_SET;
  GET_DATA_STRUCT(charge_termination).tca_clear_procent     = BQ27742_TCA_CLEAR_PROCENT;
  GET_DATA_STRUCT(charge_termination).fc_set_procent        = BQ27742_FC_SET_PROCENT;
  GET_DATA_STRUCT(charge_termination).fc_clear_procent      = BQ27742_FC_CLEAR_PROCENT;
  GET_DATA_STRUCT(charge_termination).dodateoc_delta_t      = BQ27742_DODATEOC_DELTA_T;

  GET_DATA_STRUCT(charge_termination).taper_current         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).taper_current);
  GET_DATA_STRUCT(charge_termination).min_taper_capacity    = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).min_taper_capacity);
  GET_DATA_STRUCT(charge_termination).taper_voltage         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).taper_voltage);
  GET_DATA_STRUCT(charge_termination).current_taper_window  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).current_taper_window);
  GET_DATA_STRUCT(charge_termination).tca_set               = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).tca_set);
  GET_DATA_STRUCT(charge_termination).tca_clear_procent     = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).tca_clear_procent);
  GET_DATA_STRUCT(charge_termination).fc_set_procent        = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).fc_set_procent);
  GET_DATA_STRUCT(charge_termination).fc_clear_procent      = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).fc_clear_procent);
  GET_DATA_STRUCT(charge_termination).dodateoc_delta_t      = ENSURE_ENDIANNESS(GET_DATA_STRUCT(charge_termination).dodateoc_delta_t);

  bq27742_data_flash_write(
      36,
      0x00,
      0x00,
      GET_RAW_DATA(charge_termination),
      sizeof(GET_RAW_DATA(charge_termination)));
}

void bq27742_program_flash_subclass_JEITA()
{
  #define BQ_JEITA_SUBCLASS_ID 0x27

  DEFINE_DATA_STRUCT(jeita, {
      uint8_t  t1_temp;
      uint8_t  t2_temp;
      uint8_t  t3_temp;
      uint8_t  t4_temp;
      uint8_t  t5_temp;
      uint8_t  temp_hys;
      uint16_t t1_t2_chg_voltage;
      uint16_t t2_t3_chg_voltage;
      uint16_t t3_t4_chg_voltage;
      uint16_t t4_t5_chg_voltage;
      uint8_t  t1_t2_chg_current;
      uint8_t  t2_t3_chg_current;
      uint8_t  t3_t4_chg_current;
      uint8_t  t4_t5_chg_current;
      });

  GET_DATA_STRUCT(jeita).t1_temp            = BQ27742_T1_TEMP;
  GET_DATA_STRUCT(jeita).t2_temp            = BQ27742_T2_TEMP;
  GET_DATA_STRUCT(jeita).t3_temp            = BQ27742_T3_TEMP;
  GET_DATA_STRUCT(jeita).t4_temp            = BQ27742_T4_TEMP;
  GET_DATA_STRUCT(jeita).t5_temp            = BQ27742_T5_TEMP;
  GET_DATA_STRUCT(jeita).temp_hys           = BQ27742_TEMP_HYS;
  GET_DATA_STRUCT(jeita).t1_t2_chg_voltage  = BQ27742_T1_T2_CHG_VOLTAGE;
  GET_DATA_STRUCT(jeita).t2_t3_chg_voltage  = BQ27742_T2_T3_CHG_VOLTAGE;
  GET_DATA_STRUCT(jeita).t3_t4_chg_voltage  = BQ27742_T3_T4_CHG_VOLTAGE;
  GET_DATA_STRUCT(jeita).t4_t5_chg_voltage  = BQ27742_T4_T5_CHG_VOLTAGE;
  GET_DATA_STRUCT(jeita).t1_t2_chg_current  = BQ27742_T1_T2_CHG_CURRENT;
  GET_DATA_STRUCT(jeita).t2_t3_chg_current  = BQ27742_T2_T3_CHG_CURRENT;
  GET_DATA_STRUCT(jeita).t3_t4_chg_current  = BQ27742_T3_T4_CHG_CURRENT;
  GET_DATA_STRUCT(jeita).t4_t5_chg_current  = BQ27742_T4_T5_CHG_CURRENT;

  GET_DATA_STRUCT(jeita).t1_temp            = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t1_temp);
  GET_DATA_STRUCT(jeita).t2_temp            = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t2_temp);
  GET_DATA_STRUCT(jeita).t3_temp            = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t3_temp);
  GET_DATA_STRUCT(jeita).t4_temp            = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t4_temp);
  GET_DATA_STRUCT(jeita).t5_temp            = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t5_temp);
  GET_DATA_STRUCT(jeita).temp_hys           = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).temp_hys);
  GET_DATA_STRUCT(jeita).t1_t2_chg_voltage  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t1_t2_chg_voltage);
  GET_DATA_STRUCT(jeita).t2_t3_chg_voltage  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t2_t3_chg_voltage);
  GET_DATA_STRUCT(jeita).t3_t4_chg_voltage  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t3_t4_chg_voltage);
  GET_DATA_STRUCT(jeita).t4_t5_chg_voltage  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t4_t5_chg_voltage);
  GET_DATA_STRUCT(jeita).t1_t2_chg_current  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t1_t2_chg_current);
  GET_DATA_STRUCT(jeita).t2_t3_chg_current  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t2_t3_chg_current);
  GET_DATA_STRUCT(jeita).t3_t4_chg_current  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t3_t4_chg_current);
  GET_DATA_STRUCT(jeita).t4_t5_chg_current  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(jeita).t4_t5_chg_current);

  bq27742_data_flash_write(
      BQ_JEITA_SUBCLASS_ID,
      0x00,
      0x00,
      GET_RAW_DATA(jeita),
      sizeof(GET_RAW_DATA(jeita)));
}


void bq27742_program_flash_subclass_data()
{

  #define BQ_DATA_SUBCLASS 0x26
  //trzeba tak zrobić, bo jest przerwa w bloku
  uint16_t _bq27742_design_voltage = BQ27742_DESIGN_VOLTAGE;
  _bq27742_design_voltage = ENSURE_ENDIANNESS(_bq27742_design_voltage);

  bq27742_data_flash_write(
      BQ_DATA_SUBCLASS,
      0,
      0,
      (uint8_t *)&_bq27742_design_voltage,
      sizeof(_bq27742_design_voltage));

  DEFINE_DATA_STRUCT(data, {
    uint16_t  cycle_count;
    uint16_t  cc_threshold;
    uint16_t  design_capacity;
    uint16_t  design_energy;
    uint16_t  soh_load_i;
    uint8_t   tdd_soh_percent;
    uint16_t  isd_current;
    uint8_t   isd_i_filter;
    uint8_t   min_isd_time;
    uint8_t   design_energy_scale;
  });

  GET_DATA_STRUCT(data).cycle_count         = BQ27742_CYCLE_COUNT_;
  GET_DATA_STRUCT(data).cc_threshold        = BQ27742_CC_THRESHOLD;
  GET_DATA_STRUCT(data).design_capacity     = BQ27742_DESIGN_CAPACITY_;
  GET_DATA_STRUCT(data).design_energy       = BQ27742_DESIGN_ENERGY;
  GET_DATA_STRUCT(data).soh_load_i          = BQ27742_SOH_LOAD_I;
  GET_DATA_STRUCT(data).tdd_soh_percent     = BQ27742_TDD_SOH_PERCENT;
  GET_DATA_STRUCT(data).isd_current         = BQ27742_ISD_CURRENT;
  GET_DATA_STRUCT(data).isd_i_filter        = BQ27742_ISD_I_FILTER;
  GET_DATA_STRUCT(data).min_isd_time        = BQ27742_MIN_ISD_TIME;
  GET_DATA_STRUCT(data).design_energy_scale = BQ27742_DESIGN_ENERGY_SCALE;

  GET_DATA_STRUCT(data).cycle_count          = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).cycle_count);
  GET_DATA_STRUCT(data).cc_threshold         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).cc_threshold);
  GET_DATA_STRUCT(data).design_capacity      = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).design_capacity);
  GET_DATA_STRUCT(data).design_energy        = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).design_energy);
  GET_DATA_STRUCT(data).soh_load_i           = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).soh_load_i);
  GET_DATA_STRUCT(data).tdd_soh_percent      = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).tdd_soh_percent);
  GET_DATA_STRUCT(data).isd_current          = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).isd_current);
  GET_DATA_STRUCT(data).isd_i_filter         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).isd_i_filter);
  GET_DATA_STRUCT(data).min_isd_time         = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).min_isd_time);
  GET_DATA_STRUCT(data).design_energy_scale  = ENSURE_ENDIANNESS(GET_DATA_STRUCT(data).design_energy_scale);

  bq27742_data_flash_write(
      BQ_DATA_SUBCLASS,
      0x00,
      0x08,
      GET_RAW_DATA(data),
      sizeof(GET_RAW_DATA(data)));
}

void bq27742_program_flash_subclass_discharge()
{
  DEFINE_DATA_STRUCT(discharge, {
      uint16_t soc1_set_threshold;
      uint16_t soc1_clear_threshold;
      uint16_t socf_set_threshold;
      uint16_t socf_clear_threshold;
      uint16_t bl_set_volt_threshold;
      uint8_t  bl_set_volt_time;
      uint16_t bl_clear_volt_threshold;
      uint16_t bh_set_volt_threshold;
      uint8_t  bh_volt_time;
      uint16_t bh_clear_volt_threshold;
      });

  GET_DATA_STRUCT(discharge).soc1_set_threshold       = BQ27742_SOC1_SET_THRESHOLD;
  GET_DATA_STRUCT(discharge).soc1_clear_threshold     = BQ27742_SOC1_CLEAR_THRESHOLD;
  GET_DATA_STRUCT(discharge).socf_set_threshold       = BQ27742_SOCF_SET_THRESHOLD;
  GET_DATA_STRUCT(discharge).socf_clear_threshold     = BQ27742_SOCF_CLEAR_THRESHOLD;
  GET_DATA_STRUCT(discharge).bl_set_volt_threshold    = BQ27742_BL_SET_VOLT_THRESHOLD;
  GET_DATA_STRUCT(discharge).bl_set_volt_time         = BQ27742_BL_SET_VOLT_TIME;
  GET_DATA_STRUCT(discharge).bl_clear_volt_threshold  = BQ27742_BL_CLEAR_VOLT_THRESHOLD;
  GET_DATA_STRUCT(discharge).bh_set_volt_threshold    = BQ27742_BH_SET_VOLT_THRESHOLD;
  GET_DATA_STRUCT(discharge).bh_volt_time             = BQ27742_BH_VOLT_TIME;
  GET_DATA_STRUCT(discharge).bh_clear_volt_threshold  = BQ27742_BH_CLEAR_VOLT_THRESHOLD;

  GET_DATA_STRUCT(discharge).soc1_set_threshold       =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).soc1_set_threshold);
  GET_DATA_STRUCT(discharge).soc1_clear_threshold     =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).soc1_clear_threshold);
  GET_DATA_STRUCT(discharge).socf_set_threshold       =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).socf_set_threshold);
  GET_DATA_STRUCT(discharge).socf_clear_threshold     =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).socf_clear_threshold);
  GET_DATA_STRUCT(discharge).bl_set_volt_threshold    =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).bl_set_volt_threshold);
  GET_DATA_STRUCT(discharge).bl_set_volt_time         =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).bl_set_volt_time);
  GET_DATA_STRUCT(discharge).bl_clear_volt_threshold  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).bl_clear_volt_threshold);
  GET_DATA_STRUCT(discharge).bh_set_volt_threshold    =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).bh_set_volt_threshold);
  GET_DATA_STRUCT(discharge).bh_volt_time             =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).bh_volt_time);
  GET_DATA_STRUCT(discharge).bh_clear_volt_threshold  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(discharge).bh_clear_volt_threshold);

  bq27742_data_flash_write(
      BQ27742_DISCHARGE_CLASS,      // subclass_id
      0x00,                         // flash_block
      0x00,                         // offset
      GET_RAW_DATA(discharge),           // value
      sizeof(GET_RAW_DATA(discharge)));  // len
}

void bq27742_program_flash_subclass_manufacturer_data(){
  DEFINE_DATA_STRUCT(manu_data, {
      uint16_t pack_lot_code;
      uint16_t pcb_lot_code;
      uint16_t firmware_version;
      uint16_t hardware_revision;
      uint16_t cell_revision;
      uint16_t df_config_version;
      });

  GET_DATA_STRUCT(manu_data).pack_lot_code      = BQ27742_PACK_LOT_CODE;
  GET_DATA_STRUCT(manu_data).pcb_lot_code       = BQ27742_PCB_LOT_CODE;
  GET_DATA_STRUCT(manu_data).firmware_version   = BQ27742_FIRMWARE_VERSION;
  GET_DATA_STRUCT(manu_data).hardware_revision  = BQ27742_HARDWARE_REVISION;
  GET_DATA_STRUCT(manu_data).cell_revision      = BQ27742_CELL_REVISION;
  GET_DATA_STRUCT(manu_data).df_config_version  = BQ27742_DF_CONFIG_VERSION;

  GET_DATA_STRUCT(manu_data).pack_lot_code      =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(manu_data).pack_lot_code);
  GET_DATA_STRUCT(manu_data).pcb_lot_code       =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(manu_data).pcb_lot_code);
  GET_DATA_STRUCT(manu_data).firmware_version   =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(manu_data).firmware_version);
  GET_DATA_STRUCT(manu_data).hardware_revision  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(manu_data).hardware_revision);
  GET_DATA_STRUCT(manu_data).cell_revision      =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(manu_data).cell_revision);
  GET_DATA_STRUCT(manu_data).df_config_version  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(manu_data).df_config_version);

  bq27742_data_flash_write(
      BQ27742_MANUFACTURER_DATA_CLASS,  // subclass_id
      0x00,                             // flash_block
      0x00,                             // offset
      GET_RAW_DATA(manu_data),          // value
      sizeof(GET_RAW_DATA(manu_data))); // len
}

void bq27742_program_flash_subclass_integrity_data()
{
  DEFINE_DATA_STRUCT(integrity_data, {
    uint16_t all_df_checksum;
    uint16_t static_chem_df_checksum;
    uint16_t static_df_checksum;
    uint16_t prot_checksum;
  });

  GET_DATA_STRUCT(integrity_data).all_df_checksum          = BQ27742_ALL_DF_CHECKSUM;
  GET_DATA_STRUCT(integrity_data).static_chem_df_checksum  = BQ27742_STATIC_CHEM_DF_CHECKSUM;
  GET_DATA_STRUCT(integrity_data).static_df_checksum       = BQ27742_STATIC_DF_CHECKSUM;
  GET_DATA_STRUCT(integrity_data).prot_checksum            = BQ27742_PROT_CHECKSUM;

  GET_DATA_STRUCT(integrity_data).all_df_checksum          =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(integrity_data).all_df_checksum);
  GET_DATA_STRUCT(integrity_data).static_chem_df_checksum  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(integrity_data).static_chem_df_checksum);
  GET_DATA_STRUCT(integrity_data).static_df_checksum       =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(integrity_data).static_df_checksum);
  GET_DATA_STRUCT(integrity_data).prot_checksum            =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(integrity_data).prot_checksum);

  bq27742_data_flash_write(
      BQ27742_INTEGRITY_CLASS,                // subclass_id
      0x00,                                   // flash_block
      0x06,                                   // offset
      GET_RAW_DATA(integrity_data),                // value
      sizeof(GET_RAW_DATA(integrity_data)));  // len
}

void bq27742_program_flash_subclass_lifetime_data()
{
  DEFINE_DATA_STRUCT(lifetime_data, {
    uint16_t max_temp;
    uint16_t min_temp;
    uint16_t max_pack_voltage;
    uint16_t min_pack_voltage;
    uint16_t max_chg_current;
    uint16_t max_dsg_current;
  });

  GET_DATA_STRUCT(lifetime_data).max_temp          = BQ27742_LIFETIME_MAX_TEMP;
  GET_DATA_STRUCT(lifetime_data).min_temp          = BQ27742_LIFETIME_MIN_TEMP;
  GET_DATA_STRUCT(lifetime_data).max_pack_voltage  = BQ27742_LIFETIME_MAX_PACK_VOLTAGE;
  GET_DATA_STRUCT(lifetime_data).min_pack_voltage  = BQ27742_LIFETIME_MIN_PACK_VOLTAGE;
  GET_DATA_STRUCT(lifetime_data).max_chg_current   = BQ27742_LIFETIME_MAX_CHG_CURRENT;
  GET_DATA_STRUCT(lifetime_data).max_dsg_current   = BQ27742_LIFETIME_MAX_DSG_CURRENT;

  GET_DATA_STRUCT(lifetime_data).max_temp          =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_data).max_temp);
  GET_DATA_STRUCT(lifetime_data).min_temp          =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_data).min_temp);
  GET_DATA_STRUCT(lifetime_data).max_pack_voltage  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_data).max_pack_voltage);
  GET_DATA_STRUCT(lifetime_data).min_pack_voltage  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_data).min_pack_voltage);
  GET_DATA_STRUCT(lifetime_data).max_chg_current   =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_data).max_chg_current);
  GET_DATA_STRUCT(lifetime_data).max_dsg_current   =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_data).max_dsg_current);

  bq27742_data_flash_write(
      BQ27742_LIFETIME_DATA_CLASS,          // subclass_id
      0x00,                                 // flash_block
      0x00,                                 // offset
      GET_RAW_DATA(lifetime_data),          // value
      sizeof(GET_RAW_DATA(lifetime_data))); // len
}

void bq27742_program_flash_subclass_lifetime_temp_samples()
{
  DEFINE_DATA_STRUCT(lifetime_temp_samples, {
      uint16_t lt_flash_cnt;
      uint8_t  lt_afe_status;
      });

  GET_DATA_STRUCT(lifetime_temp_samples).lt_flash_cnt   = BQ27742_LT_FLASH_CNT;
  GET_DATA_STRUCT(lifetime_temp_samples).lt_afe_status  = BQ27742_LT_AFE_STATUS;

  GET_DATA_STRUCT(lifetime_temp_samples).lt_flash_cnt   =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_temp_samples).lt_flash_cnt);
  GET_DATA_STRUCT(lifetime_temp_samples).lt_afe_status  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_temp_samples).lt_afe_status);

  bq27742_data_flash_write(
      BQ27742_LIFETIME_TEMP_SAMPLES_CLASS,  // subclass_id
      0x00,                                 // flash_block
      0x00,                                 // offset
      GET_RAW_DATA(lifetime_temp_samples),          // value
      sizeof(GET_RAW_DATA(lifetime_temp_samples))); // len
}

void bq27742_program_flash_subclass_registers()
{
  DEFINE_DATA_STRUCT(registers, {
      uint16_t pack_configuration;
      uint8_t  pack_configuration_b;
      uint8_t  pack_configuration_c;
      uint8_t  pack_configuration_d;
      uint8_t  prot_oc_config;
      uint8_t  prot_ov_config;
      });

  GET_DATA_STRUCT(registers).pack_configuration    = BQ27742_PACK_CONFIGURATION;
  GET_DATA_STRUCT(registers).pack_configuration_b  = BQ27742_PACK_CONFIGURATION_B;
  GET_DATA_STRUCT(registers).pack_configuration_c  = BQ27742_PACK_CONFIGURATION_C;
  GET_DATA_STRUCT(registers).pack_configuration_d  = BQ27742_PACK_CONFIGURATION_D;
  GET_DATA_STRUCT(registers).prot_oc_config        = BQ27742_PROT_OC_CONFIG;
  GET_DATA_STRUCT(registers).prot_ov_config        = BQ27742_PROT_OV_CONFIG;

  GET_DATA_STRUCT(registers).pack_configuration    =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(registers).pack_configuration);
  GET_DATA_STRUCT(registers).pack_configuration_b  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(registers).pack_configuration_b);
  GET_DATA_STRUCT(registers).pack_configuration_c  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(registers).pack_configuration_c);
  GET_DATA_STRUCT(registers).pack_configuration_d  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(registers).pack_configuration_d);
  GET_DATA_STRUCT(registers).prot_oc_config        =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(registers).prot_oc_config);
  GET_DATA_STRUCT(registers).prot_ov_config        =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(registers).prot_ov_config);

  bq27742_data_flash_write(
      BQ27742_REGISTERS_CLASS,          // subclass_id
      0x00,                             // flash_block
      0x00,                             // offset
      GET_RAW_DATA(registers),          // value
      sizeof(GET_RAW_DATA(registers))); // len
}

void bq27742_program_flash_subclass_lifetime_resolution()
{
  uint8_t subclass_id=66;
  uint8_t flash_block=0;
  uint8_t offset=0;

  DEFINE_DATA_STRUCT(lifetime_res, {
      uint8_t lt_temp_res;
      uint8_t lt_v_res;
      uint8_t lt_cur_res;
      uint16_t lt_update_time;
      uint16_t flash_update_ok_voltage;
      });

  GET_DATA_STRUCT(lifetime_res).lt_temp_res             = BQ27742_LT_TEMP_RES;
  GET_DATA_STRUCT(lifetime_res).lt_v_res                = BQ27742_LT_V_RES;
  GET_DATA_STRUCT(lifetime_res).lt_cur_res              = BQ27742_LT_CUR_RES;
  GET_DATA_STRUCT(lifetime_res).lt_update_time          = BQ27742_LT_UPDATE_TIME;
  GET_DATA_STRUCT(lifetime_res).flash_update_ok_voltage = BQ27742_FLASH_UPDATE_OK_VOLTAGE;

  GET_DATA_STRUCT(lifetime_res).lt_temp_res             =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_res).lt_temp_res);
  GET_DATA_STRUCT(lifetime_res).lt_v_res                =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_res).lt_v_res);
  GET_DATA_STRUCT(lifetime_res).lt_cur_res              =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_res).lt_cur_res);
  GET_DATA_STRUCT(lifetime_res).lt_update_time          =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_res).lt_update_time);
  GET_DATA_STRUCT(lifetime_res).flash_update_ok_voltage =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(lifetime_res).flash_update_ok_voltage);

  bq27742_data_flash_write(
      BQ27742_LIFETIME_RES_CLASS,           // subclass_id
      0x00,                                 // flash_block
      0x00,                                 // offset
      GET_RAW_DATA(lifetime_res),           // value
      sizeof(GET_RAW_DATA(lifetime_res)));  // len
}

void bq27742_program_flash_subclass_power()
{
  uint8_t subclass_id=68;
  uint8_t flash_block=0;
  uint8_t offset=0;

  DEFINE_DATA_STRUCT(power ,{
      uint16_t flash_update_ok_voltage;
      uint16_t sleep_current;
      });

  GET_DATA_STRUCT(power).flash_update_ok_voltage  = BQ27742_FLASH_UPDATE_OK_VOLTAGE;
  GET_DATA_STRUCT(power).sleep_current            = BQ27742_SLEEP_CURRENT;

  GET_DATA_STRUCT(power).flash_update_ok_voltage  =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(power).flash_update_ok_voltage);
  GET_DATA_STRUCT(power).sleep_current            =
    ENSURE_ENDIANNESS(GET_DATA_STRUCT(power).sleep_current);

  bq27742_data_flash_write(
      BQ27742_POWER_CLASS,          // subclass_id
      0x00,                         // flash_block
      0x00,                         // offset
      GET_RAW_DATA(power),          // value
      sizeof(GET_RAW_DATA(power))); // len

  uint16_t _bq27742_shutdown_v = BQ27742_SHUTDOWN_V;
  _bq27742_shutdown_v = ENSURE_ENDIANNESS(_bq27742_shutdown_v);

  uint8_t _bq27742_fs_wait = BQ27742_FS_WAIT;

  bq27742_data_flash_write(
      BQ27742_POWER_CLASS,
      0x00,
      0x0A,
      &_bq27742_shutdown_v,
      sizeof(_bq27742_shutdown_v));

  bq27742_data_flash_write(
      BQ27742_POWER_CLASS,
      0x00,
      0x0C,
      &_bq27742_fs_wait,
      sizeof(_bq27742_fs_wait));
}

void bq27742_program_flash_subclass_manufacturer_info1()
//TODO rewrite using subclass
{
  uint8_t _data[BQ27742_DATA_FLASH_BLOCK_SIZE] = {[0 ... sizeof(_data)-1] = 0}

  bq27742_data_flash_write(
      BQ27742_MANUFACTURER_INFO1_CLASS, // subclass_id
      0x00,                             // flash_block
      0x00,                             // offset
      _data,                            // value
      sizeof(_data));                   // len

	uint8_t subclass_id=58;
	uint8_t flash_block=0;
	uint8_t offset=0;

	uint8_t reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};
	uint32_t flash_date_sum = 0;
	uint8_t checksum = 0;
	uint8_t flash_date_access = 0x00;
	uint8_t block_date_address = BQ27742_BLOCK_DATA + offset;

	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CONTROL, &flash_date_access, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_CLASS, &subclass_id, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_DATA_FLASH_BLOCK, &flash_block, 1);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, block_date_address, write_value, write_value_size);
	TWI_ReadReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA, reg_value, 32);
	WDT_RR();
	for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++)
		flash_date_sum +=reg_value[i];

	checksum = 255 - (uint8_t)(flash_date_sum);
	TWI_WriteReg (TWI_BQ27742_ADDRESS, BQ27742_BLOCK_DATA_CS, &checksum, 1);
	ic_delay_ms(BQ27742_REGISTER_WRITE_DELAY);
	WDT_RR();

}
















