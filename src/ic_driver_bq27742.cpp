/**
 * @file    ic_driver_bq27742.cpp
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    December, 2017
 * @brief   Brief description
 *
 * Description
 */

/*#include "ic_driver_bq27742.h"*/
#include <typeinfo>
#include <cstdint>
#include <climits>
#include <cmath>

#include "ic_driver_twi.h"
#include "ic_service_ltc.h"

#include "ic_driver_bq27742_definitions.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

const float BQ27742_CC_GAIN=4.768/54.931;
const float BQ27742_CC_DELTA=5677445.0/54.810;

/*#define FORCE_ENDIANNESS __ORDER_LITTLE_ENDIAN__*/
#define FORCE_ENDIANNESS __ORDER_BIG_ENDIAN__

template <class T>
static constexpr auto swapBytes(T val){
  return sizeof(T)==1 ? val :
    sizeof(T)==2 ? ((val>>CHAR_BIT) & 0x00FF) | ((val << CHAR_BIT) & 0xFF00) :
      ((val>>CHAR_BIT*3) & 0x000000FF) | ((val>>CHAR_BIT) & 0x0000FF00) |
      ((val<<CHAR_BIT) & 0x00FF0000) | ((val<<CHAR_BIT*3) & 0xFF000000);
}

static constexpr float swapBytes(float val){
  auto _local = swapBytes(*reinterpret_cast<uint32_t *>(&val));
  return *reinterpret_cast<float *>(&_local);
}

//template <class T>
//void bq27742_program_flash_subclass(T s){
//}

//constexpr float SWAP_BYTES(float &val){
  //auto _local_var_ptr = reinterpret_cast<uint32_t *>(&val);
  //SWAP_BYTES(*_local_var_ptr);
  //return *_local_var_ptr;
//}

#if FORCE_ENDIANNESS == __BYTE_ORDER__
template <class T>
constexpr T ensureEndianness (T a) {
  return a;
}
#else
template <class T>
constexpr T ensureEndianness (T a) {
  return swapBytes(a);
}
#endif

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

static ic_return_val_e set_bq_register_var(const uint8_t reg_addr, const uint8_t *reg_val, const size_t len){
  uint8_t _buffer[len+1];
  _buffer[0] = reg_addr;

  for(auto i = 1ul; i<len+1; ++i){
    _buffer[i] = *reg_val++;
  }

  return TWI_SEND_DATA(BQ, _buffer, sizeof(_buffer), NULL, NULL);
}

static ic_return_val_e set_bq_register(const uint8_t reg_addr, const uint8_t reg_val){
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
    const uint8_t *value,
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

constexpr uint32_t float_to_bq_format (float val)
{
  const float _2pow_25 = 2.98023224e-8;
  union {
    uint32_t raw;
    uint8_t  byte[4];
  } ret_val = {
raw : 0};

  int16_t exp=0;
  float mod_val = 0.0;
  float tmp_val = 0.0;

  mod_val=std::abs(val);

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

  tmp_val=(std::pow(2.0,(8-exp))*mod_val)-128.0;

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

namespace bqSubclassData{

  #define BQ_STRUCT static const struct __attribute__((packed))

  BQ_STRUCT{
      uint32_t cc_gain;
      uint32_t cc_delta;
      uint16_t cc_offset;
      uint8_t board_offset;
      uint8_t int_temp_offset;
      uint8_t ext_temp_offset;
      uint8_t pack_v_offset;
  }data104 ={
    cc_gain         :ensureEndianness(float_to_bq_format(BQ27742_CC_GAIN)),
    cc_delta        :ensureEndianness(float_to_bq_format(BQ27742_CC_DELTA)),
    cc_offset       :ensureEndianness(static_cast<uint16_t>(BQ27742_CC_OFFSET_)),
    board_offset    :ensureEndianness(static_cast<uint8_t>(BQ27742_BOARD_OFFSET_)),
    int_temp_offset :ensureEndianness(static_cast<uint8_t>(BQ27742_INT_TEMP_OFFSET)),
    ext_temp_offset :ensureEndianness(static_cast<uint8_t>(BQ27742_EXT_TEMP_OFFSET)),
    pack_v_offset   :ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_V_OFFSET))
  };

  BQ_STRUCT{
    uint16_t  ov_prot_treshold;
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
  }safety = {
    ov_prot_treshold      : ensureEndianness(static_cast<uint16_t>(BQ27742_OV_PROT_THRESHOLD)),
    ov_prot_delay         : ensureEndianness(static_cast<uint8_t>(BQ27742_OV_PROT_DELAY)),
    ov_prot_recovery      : ensureEndianness(static_cast<uint16_t>(BQ27742_OV_PROT_RECOVERY)),
    uv_prot_threshold     : ensureEndianness(static_cast<uint16_t>(BQ27742_UV_PROT_THRESHOLD)),
    uv_prot_delay         : ensureEndianness(static_cast<uint8_t>(BQ27742_UV_PROT_DELAY)),
    uv_prot_recovery      : ensureEndianness(static_cast<uint16_t>(BQ27742_UV_PROT_RECOVERY)),
    body_diode_threshold  : ensureEndianness(static_cast<uint16_t>(BQ27742_BODY_DIODE_THRESHOLD)),
    ot_chg                : ensureEndianness(static_cast<uint16_t>(BQ27742_OT_CHG)),
    ot_chg_time           : ensureEndianness(static_cast<uint8_t>(BQ27742_OT_CHG_TIME)),
    ot_chg_recovery       : ensureEndianness(static_cast<uint16_t>(BQ27742_OT_CHG_RECOVERY)),
    ot_dsg                : ensureEndianness(static_cast<uint16_t>(BQ27742_OT_DSG)),
    ot_dsg_time           : ensureEndianness(static_cast<uint8_t>(BQ27742_OT_DSG_TIME)),
    ot_dsg_recovery       : ensureEndianness(static_cast<uint16_t>(BQ27742_OT_DSG_RECOVERY))
  };

  BQ_STRUCT{
    uint16_t charging_voltage;
  }charge = {
    charging_voltage :ensureEndianness(static_cast<uint16_t>(BQ27742_CHARGING_VOLTAGE_))
  };

  BQ_STRUCT{
    uint16_t  taper_current;
    uint16_t  min_taper_capacity;
    uint16_t  taper_voltage;
    uint8_t   current_taper_window;
    uint8_t   tca_set;
    uint8_t   tca_clear_procent;
    uint8_t   fc_set_procent;
    uint8_t   fc_clear_procent;
    uint16_t  dodateoc_delta_t;
  }charge_termination = {
    taper_current         : ensureEndianness(static_cast<uint16_t>(BQ27742_TAPER_CURRENT)),
    min_taper_capacity    : ensureEndianness(static_cast<uint16_t>(BQ27742_MIN_TAPER_CAPACITY)),
    taper_voltage         : ensureEndianness(static_cast<uint16_t>(BQ27742_TAPER_VOLTAGE)),
    current_taper_window  : ensureEndianness(static_cast<uint8_t>(BQ27742_CURRENT_TAPER_WINDOW)),
    tca_set               : ensureEndianness(static_cast<uint8_t>(BQ27742_TCA_SET)),
    tca_clear_procent     : ensureEndianness(static_cast<uint8_t>(BQ27742_TCA_CLEAR_PROCENT)),
    fc_set_procent        : ensureEndianness(static_cast<uint8_t>(BQ27742_FC_SET_PROCENT)),
    fc_clear_procent      : ensureEndianness(static_cast<uint8_t>(BQ27742_FC_CLEAR_PROCENT)),
    dodateoc_delta_t      : ensureEndianness(static_cast<uint16_t>(BQ27742_DODATEOC_DELTA_T))
  };

  BQ_STRUCT{
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
  }jeita = {
    t1_temp            : ensureEndianness(static_cast<uint8_t>(BQ27742_T1_TEMP)),
    t2_temp            : ensureEndianness(static_cast<uint8_t>(BQ27742_T2_TEMP)),
    t3_temp            : ensureEndianness(static_cast<uint8_t>(BQ27742_T3_TEMP)),
    t4_temp            : ensureEndianness(static_cast<uint8_t>(BQ27742_T4_TEMP)),
    t5_temp            : ensureEndianness(static_cast<uint8_t>(BQ27742_T5_TEMP)),
    temp_hys           : ensureEndianness(static_cast<uint8_t>(BQ27742_TEMP_HYS)),
    t1_t2_chg_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_T1_T2_CHG_VOLTAGE)),
    t2_t3_chg_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_T2_T3_CHG_VOLTAGE)),
    t3_t4_chg_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_T3_T4_CHG_VOLTAGE)),
    t4_t5_chg_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_T4_T5_CHG_VOLTAGE)),
    t1_t2_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T1_T2_CHG_CURRENT)),
    t2_t3_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T2_T3_CHG_CURRENT)),
    t3_t4_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T3_T4_CHG_CURRENT)),
    t4_t5_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T4_T5_CHG_CURRENT))
  };

  static const uint16_t design_voltage = ensureEndianness(static_cast<uint16_t>(BQ27742_DESIGN_VOLTAGE));

  BQ_STRUCT{
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
  }data = {
    cycle_count         : ensureEndianness(static_cast<uint16_t>(BQ27742_CYCLE_COUNT_)),
    cc_threshold        : ensureEndianness(static_cast<uint16_t>(BQ27742_CC_THRESHOLD)),
    design_capacity     : ensureEndianness(static_cast<uint16_t>(BQ27742_DESIGN_CAPACITY_)),
    design_energy       : ensureEndianness(static_cast<uint16_t>(BQ27742_DESIGN_ENERGY)),
    soh_load_i          : ensureEndianness(static_cast<uint16_t>(BQ27742_SOH_LOAD_I)),
    tdd_soh_percent     : ensureEndianness(static_cast<uint8_t>(BQ27742_TDD_SOH_PERCENT)),
    isd_current         : ensureEndianness(static_cast<uint16_t>(BQ27742_ISD_CURRENT)),
    isd_i_filter        : ensureEndianness(static_cast<uint8_t>(BQ27742_ISD_I_FILTER)),
    min_isd_time        : ensureEndianness(static_cast<uint8_t>(BQ27742_MIN_ISD_TIME)),
    design_energy_scale : ensureEndianness(static_cast<uint8_t>(BQ27742_DESIGN_ENERGY_SCALE))
  };

  BQ_STRUCT{
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
  }discharge = {
    soc1_set_threshold       : ensureEndianness(static_cast<uint16_t>(BQ27742_SOC1_SET_THRESHOLD)),
    soc1_clear_threshold     : ensureEndianness(static_cast<uint16_t>(BQ27742_SOC1_CLEAR_THRESHOLD)),
    socf_set_threshold       : ensureEndianness(static_cast<uint16_t>(BQ27742_SOCF_SET_THRESHOLD)),
    socf_clear_threshold     : ensureEndianness(static_cast<uint16_t>(BQ27742_SOCF_CLEAR_THRESHOLD)),
    bl_set_volt_threshold    : ensureEndianness(static_cast<uint16_t>(BQ27742_BL_SET_VOLT_THRESHOLD)),
    bl_set_volt_time         : ensureEndianness(static_cast<uint8_t>(BQ27742_BL_SET_VOLT_TIME)),
    bl_clear_volt_threshold  : ensureEndianness(static_cast<uint16_t>(BQ27742_BL_CLEAR_VOLT_THRESHOLD)),
    bh_set_volt_threshold    : ensureEndianness(static_cast<uint16_t>(BQ27742_BH_SET_VOLT_THRESHOLD)),
    bh_volt_time             : ensureEndianness(static_cast<uint8_t>(BQ27742_BH_VOLT_TIME)),
    bh_clear_volt_threshold  : ensureEndianness(static_cast<uint16_t>(BQ27742_BH_CLEAR_VOLT_THRESHOLD))
  };

  BQ_STRUCT{
    uint16_t pack_lot_code;
    uint16_t pcb_lot_code;
    uint16_t firmware_version;
    uint16_t hardware_revision;
    uint16_t cell_revision;
    uint16_t df_config_version;
  }manu_data = {
    pack_lot_code      : ensureEndianness(BQ27742_PACK_LOT_CODE),
    pcb_lot_code       : ensureEndianness(BQ27742_PCB_LOT_CODE),
    firmware_version   : ensureEndianness(BQ27742_FIRMWARE_VERSION),
    hardware_revision  : ensureEndianness(BQ27742_HARDWARE_REVISION),
    cell_revision      : ensureEndianness(BQ27742_CELL_REVISION),
    df_config_version  : ensureEndianness(BQ27742_DF_CONFIG_VERSION)
  };

  BQ_STRUCT{
    uint16_t all_df_checksum;
    uint16_t static_chem_df_checksum;
    uint16_t static_df_checksum;
    uint16_t prot_checksum;
  }integrity_data = {
    all_df_checksum          : ensureEndianness(static_cast<uint16_t>(BQ27742_ALL_DF_CHECKSUM)),
    static_chem_df_checksum  : ensureEndianness(static_cast<uint16_t>(BQ27742_STATIC_CHEM_DF_CHECKSUM&0xFFFF)),
    static_df_checksum       : ensureEndianness(static_cast<uint16_t>(BQ27742_STATIC_DF_CHECKSUM)),
    prot_checksum            : ensureEndianness(static_cast<uint16_t>(BQ27742_PROT_CHECKSUM))
  };

  BQ_STRUCT{
    uint16_t max_temp;
    uint16_t min_temp;
    uint16_t max_pack_voltage;
    uint16_t min_pack_voltage;
    uint16_t max_chg_current;
    uint16_t max_dsg_current;
  }lifetime_data = {
    max_temp          : ensureEndianness(static_cast<uint16_t>(BQ27742_LIFETIME_MAX_TEMP)),
    min_temp          : ensureEndianness(static_cast<uint16_t>(BQ27742_LIFETIME_MIN_TEMP)),
    max_pack_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_LIFETIME_MAX_PACK_VOLTAGE)),
    min_pack_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_LIFETIME_MIN_PACK_VOLTAGE)),
    max_chg_current   : ensureEndianness(static_cast<uint16_t>(BQ27742_LIFETIME_MAX_CHG_CURRENT)),
    max_dsg_current   : ensureEndianness(static_cast<uint16_t>(BQ27742_LIFETIME_MAX_DSG_CURRENT))
  };

  BQ_STRUCT{
    uint16_t lt_flash_cnt;
    uint8_t  lt_afe_status;
  }lifetime_temp_samples = {
    lt_flash_cnt   : ensureEndianness(static_cast<uint16_t>(BQ27742_LT_FLASH_CNT)),
    lt_afe_status  : ensureEndianness(BQ27742_LT_AFE_STATUS)
  };

  BQ_STRUCT{
    uint16_t pack_configuration;
    uint8_t  pack_configuration_b;
    uint8_t  pack_configuration_c;
    uint8_t  pack_configuration_d;
    uint8_t  prot_oc_config;
    uint8_t  prot_ov_config;
  }registers = {
    pack_configuration    : ensureEndianness(static_cast<uint16_t>(BQ27742_PACK_CONFIGURATION)),
    pack_configuration_b  : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_CONFIGURATION_B)),
    pack_configuration_c  : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_CONFIGURATION_C)),
    pack_configuration_d  : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_CONFIGURATION_D)),
    prot_oc_config        : ensureEndianness(static_cast<uint8_t>(BQ27742_PROT_OC_CONFIG)),
    prot_ov_config        : ensureEndianness(static_cast<uint8_t>(BQ27742_PROT_OV_CONFIG))
  };

  BQ_STRUCT{
    uint8_t lt_temp_res;
    uint8_t lt_v_res;
    uint8_t lt_cur_res;
    uint16_t lt_update_time;
    uint16_t flash_update_ok_voltage;
  }lifetime_res = {
    lt_temp_res             : ensureEndianness(static_cast<uint8_t>(BQ27742_LT_TEMP_RES)),
    lt_v_res                : ensureEndianness(static_cast<uint8_t>(BQ27742_LT_V_RES)),
    lt_cur_res              : ensureEndianness(static_cast<uint8_t>(BQ27742_LT_CUR_RES)),
    lt_update_time          : ensureEndianness(static_cast<uint16_t>(BQ27742_LT_UPDATE_TIME)),
    flash_update_ok_voltage : ensureEndianness(static_cast<uint16_t>(BQ27742_FLASH_UPDATE_OK_VOLTAGE))
  };

  BQ_STRUCT{
    uint16_t flash_update_ok_voltage;
    uint16_t sleep_current;
  }power = {
    flash_update_ok_voltage  : ensureEndianness(static_cast<uint16_t>(BQ27742_FLASH_UPDATE_OK_VOLTAGE)),
    sleep_current            : ensureEndianness(static_cast<uint16_t>(BQ27742_SLEEP_CURRENT))
  };

  static const uint16_t shutdown_v = ensureEndianness(static_cast<uint16_t>(BQ27742_SHUTDOWN_V));

  static const uint8_t fs_wait = ensureEndianness(static_cast<uint8_t>(BQ27742_FS_WAIT));

  BQ_STRUCT{
    uint8_t block_a_0;
    uint8_t block_a_1;
    uint8_t block_a_2;
    uint8_t block_a_3;
    uint8_t block_a_4;
    uint8_t block_a_5;
    uint8_t block_a_6;
    uint8_t block_a_7;
    uint8_t block_a_8;
    uint8_t block_a_9;
    uint8_t block_a_10;
    uint8_t block_a_11;
    uint8_t block_a_12;
    uint8_t block_a_13;
    uint8_t block_a_14;
    uint8_t block_a_15;
    uint8_t block_a_16;
    uint8_t block_a_17;
    uint8_t block_a_18;
    uint8_t block_a_19;
    uint8_t block_a_20;
    uint8_t block_a_21;
    uint8_t block_a_22;
    uint8_t block_a_23;
    uint8_t block_a_24;
    uint8_t block_a_25;
    uint8_t block_a_26;
    uint8_t block_a_27;
    uint8_t block_a_28;
    uint8_t block_a_29;
    uint8_t block_a_30;
    uint8_t block_a_31;
  }manufacturer_info1 = {
    block_a_0   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_0)),
    block_a_1   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_1)),
    block_a_2   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_2)),
    block_a_3   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_3)),
    block_a_4   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_4)),
    block_a_5   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_5)),
    block_a_6   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_6)),
    block_a_7   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_7)),
    block_a_8   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_8)),
    block_a_9   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_9)),
    block_a_10  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_10)),
    block_a_11  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_11)),
    block_a_12  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_12)),
    block_a_13  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_13)),
    block_a_14  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_14)),
    block_a_15  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_15)),
    block_a_16  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_16)),
    block_a_17  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_17)),
    block_a_18  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_18)),
    block_a_19  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_19)),
    block_a_20  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_20)),
    block_a_21  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_21)),
    block_a_22  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_22)),
    block_a_23  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_23)),
    block_a_24  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_24)),
    block_a_25  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_25)),
    block_a_26  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_26)),
    block_a_27  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_27)),
    block_a_28  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_28)),
    block_a_29  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_29)),
    block_a_30  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_30)),
    block_a_31  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_A_31))
  };

  BQ_STRUCT{
    uint8_t block_b_0;
    uint8_t block_b_1;
    uint8_t block_b_2;
    uint8_t block_b_3;
    uint8_t block_b_4;
    uint8_t block_b_5;
    uint8_t block_b_6;
    uint8_t block_b_7;
    uint8_t block_b_8;
    uint8_t block_b_9;
    uint8_t block_b_10;
    uint8_t block_b_11;
    uint8_t block_b_12;
    uint8_t block_b_13;
    uint8_t block_b_14;
    uint8_t block_b_15;
    uint8_t block_b_16;
    uint8_t block_b_17;
    uint8_t block_b_18;
    uint8_t block_b_19;
    uint8_t block_b_20;
    uint8_t block_b_21;
    uint8_t block_b_22;
    uint8_t block_b_23;
    uint8_t block_b_24;
    uint8_t block_b_25;
    uint8_t block_b_26;
    uint8_t block_b_27;
    uint8_t block_b_28;
    uint8_t block_b_29;
    uint8_t block_b_30;
    uint8_t block_b_31;
  }manufacturer_info2 = {
    block_b_0   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_0)),
    block_b_1   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_1)),
    block_b_2   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_2)),
    block_b_3   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_3)),
    block_b_4   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_4)),
    block_b_5   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_5)),
    block_b_6   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_6)),
    block_b_7   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_7)),
    block_b_8   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_8)),
    block_b_9   : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_9)),
    block_b_10  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_10)),
    block_b_11  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_11)),
    block_b_12  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_12)),
    block_b_13  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_13)),
    block_b_14  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_14)),
    block_b_15  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_15)),
    block_b_16  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_16)),
    block_b_17  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_17)),
    block_b_18  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_18)),
    block_b_19  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_19)),
    block_b_20  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_20)),
    block_b_21  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_21)),
    block_b_22  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_22)),
    block_b_23  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_23)),
    block_b_24  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_24)),
    block_b_25  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_25)),
    block_b_26  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_26)),
    block_b_27  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_27)),
    block_b_28  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_28)),
    block_b_29  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_29)),
    block_b_30  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_30)),
    block_b_31  : ensureEndianness(static_cast<uint8_t>(BQ27742_BLOCK_B_31))
  };

  static const uint8_t   load_select                  = ensureEndianness(static_cast<uint8_t>(BQ27742_LOAD_SELECT));
  static const uint8_t   load_mode                    = ensureEndianness(static_cast<uint8_t>(BQ27742_LOAD_MODE));
  static const uint8_t   max_res_factor               = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_RES_FACTOR));
  static const uint8_t   min_res_factor               = ensureEndianness(static_cast<uint8_t>(BQ27742_MIN_RES_FACTOR));
  static const uint16_t  ra_filter                    = ensureEndianness(static_cast<uint16_t>(BQ27742_RA_FILTER));
  static const uint16_t  res_v_drop                   = ensureEndianness(static_cast<uint16_t>(BQ27742_RES_V_DROP));
  static const uint8_t   fast_qmax_start_dod_procent  = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_QMAX_START_DOD_PROCENT));
  static const uint8_t   fast_qmax_end_dod_procent    = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_QMAX_END_DOD_PROCENT));
  static const uint16_t  fast_qmax_star_volt_delta    = ensureEndianness(static_cast<uint16_t>(BQ27742_FAST_QMAX_STAR_VOLT_DELTA));
  static const uint16_t  fast_qmax_current_threshold  = ensureEndianness(static_cast<uint16_t>(BQ27742_FAST_QMAX_CURRENT_THRESHOLD));
  static const uint8_t   qmax_capacity_err            = ensureEndianness(static_cast<uint8_t>(BQ27742_QMAX_CAPACITY_ERR));
  static const uint8_t   max_qmax_change              = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_QMAX_CHANGE));
  static const uint16_t  terminate_voltage            = ensureEndianness(static_cast<uint16_t>(BQ27742_TERMINATE_VOLTAGE));
  static const uint16_t  term_v_delta                 = ensureEndianness(static_cast<uint16_t>(BQ27742_TERM_V_DELTA));
  static const uint16_t  resrelax_time                = ensureEndianness(static_cast<uint16_t>(BQ27742_RESRELAX_TIME));
  static const uint16_t  user_rate_ma                 = ensureEndianness(static_cast<uint16_t>(BQ27742_USER_RATE_MA));
  static const uint16_t  user_rate_pwr                = ensureEndianness(static_cast<uint16_t>(BQ27742_USER_RATE_PWR));
  static const uint16_t  reserve_cap_mah              = ensureEndianness(static_cast<uint16_t>(BQ27742_RESERVE_CAP_MAH));
  static const uint16_t  max_deltav                   = ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_DELTAV));
  static const uint16_t  min_deltav                   = ensureEndianness(static_cast<uint16_t>(BQ27742_MIN_DELTAV));
  static const uint8_t   max_sim_rate                 = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_SIM_RATE));
  static const uint8_t   min_sim_rate                 = ensureEndianness(static_cast<uint8_t>(BQ27742_MIN_SIM_RATE));
  static const uint16_t  ra_max_delta                 = ensureEndianness(static_cast<uint16_t>(BQ27742_RA_MAX_DELTA));
  static const uint16_t  trace_resistance             = ensureEndianness(static_cast<uint16_t>(BQ27742_TRACE_RESISTANCE));
  static const uint16_t  downstream_resistance        = ensureEndianness(static_cast<uint16_t>(BQ27742_DOWNSTREAM_RESISTANCE));
  static const uint8_t   qmax_max_delta_procent       = ensureEndianness(static_cast<uint8_t>(BQ27742_QMAX_MAX_DELTA_PROCENT));
  static const uint8_t   qmax_bound_procent           = ensureEndianness(static_cast<uint8_t>(BQ27742_QMAX_BOUND_PROCENT));
  static const uint16_t  deltav_max_delta             = ensureEndianness(static_cast<uint16_t>(BQ27742_DELTAV_MAX_DELTA));
  static const uint16_t  max_res_scale                = ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_RES_SCALE));
  static const uint16_t  min_res_scale                = ensureEndianness(static_cast<uint16_t>(BQ27742_MIN_RES_SCALE));
  static const uint8_t   fast_scale_start_soc         = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_SCALE_START_SOC));
  static const uint8_t   fast_scale_load_select       = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_SCALE_LOAD_SELECT));
  static const uint16_t  charge_hys_v_shift           = ensureEndianness(static_cast<uint16_t>(BQ27742_CHARGE_HYS_V_SHIFT));
  static const uint8_t   rascl_ocv_rst_temp_thresh    = ensureEndianness(static_cast<uint8_t>(BQ27742_RASCL_OCV_RST_TEMP_THRESH));
  static const uint16_t  max_allowed_current          = ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_ALLOWED_CURRENT));
  static const uint8_t   max_current_pulse_duration   = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_CURRENT_PULSE_DURATION));
  static const uint16_t  max_current_interrupt_step   = ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_CURRENT_INTERRUPT_STEP));

  BQ_STRUCT{
    uint16_t dsg_current_threshold;
    uint16_t chg_current_threshold;
    uint16_t quit_current;
    uint16_t dsg_relax_time;
    uint8_t chg_relax_time;
    uint8_t quit_relax_time;
    uint16_t max_ir_correct;
  }current_thresholds = {
    dsg_current_threshold : ensureEndianness(static_cast<uint16_t>(BQ27742_DSG_CURRENT_THRESHOLD)),
    chg_current_threshold : ensureEndianness(static_cast<uint16_t>(BQ27742_CHG_CURRENT_THRESHOLD)),
    quit_current          : ensureEndianness(static_cast<uint16_t>(BQ27742_QUIT_CURRENT)),
    dsg_relax_time        : ensureEndianness(static_cast<uint16_t>(BQ27742_DSG_RELAX_TIME)),
    chg_relax_time        : ensureEndianness(static_cast<uint8_t>(BQ27742_CHG_RELAX_TIME)),
    quit_relax_time       : ensureEndianness(static_cast<uint8_t>(BQ27742_QUIT_RELAX_TIME)),
    max_ir_correct        : ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_IR_CORRECT))
  };

  BQ_STRUCT{
    uint16_t qmax_cell_0;
    uint8_t  update_status;
    uint16_t v_at_chg_term;
    uint16_t avg_i_last_run;
    uint16_t avg_p_last_run;
    uint16_t delta_voltage;
    uint16_t t_rise;
    uint16_t t_time_constant;
  }state = {
    qmax_cell_0      : ensureEndianness(static_cast<uint16_t>(BQ27742_QMAX_CELL_0)),
    update_status    : ensureEndianness(static_cast<uint8_t>(BQ27742_UPDATE_STATUS)),
    v_at_chg_term    : ensureEndianness(static_cast<uint16_t>(BQ27742_V_AT_CHG_TERM)),
    avg_i_last_run   : ensureEndianness(static_cast<uint16_t>(BQ27742_AVG_I_LAST_RUN)),
    avg_p_last_run   : ensureEndianness(static_cast<uint16_t>(BQ27742_AVG_P_LAST_RUN)),
    delta_voltage    : ensureEndianness(static_cast<uint16_t>(BQ27742_DELTA_VOLTAGE)),
    t_rise           : ensureEndianness(static_cast<uint16_t>(BQ27742_T_RISE)),
    t_time_constant  : ensureEndianness(static_cast<uint16_t>(BQ27742_T_TIME_CONSTANT))
  };

  static const uint16_t chem_id = BQ27742_CHEM_ID_;

  BQ_STRUCT{
    uint16_t cell0_r_a_flag;
    uint16_t cell0_r_a_0;
    uint16_t cell0_r_a_1;
    uint16_t cell0_r_a_2;
    uint16_t cell0_r_a_3;
    uint16_t cell0_r_a_4;
    uint16_t cell0_r_a_5;
    uint16_t cell0_r_a_6;
    uint16_t cell0_r_a_7;
    uint16_t cell0_r_a_8;
    uint16_t cell0_r_a_9;
    uint16_t cell0_r_a_10;
    uint16_t cell0_r_a_11;
    uint16_t cell0_r_a_12;
    uint16_t cell0_r_a_13;
    uint16_t cell0_r_a_14;
  }r_a0 = {
    cell0_r_a_flag  : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_FLAG)),
    cell0_r_a_0     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_0)),
    cell0_r_a_1     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_1)),
    cell0_r_a_2     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_2)),
    cell0_r_a_3     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_3)),
    cell0_r_a_4     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_4)),
    cell0_r_a_5     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_5)),
    cell0_r_a_6     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_6)),
    cell0_r_a_7     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_7)),
    cell0_r_a_8     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_8)),
    cell0_r_a_9     : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_9)),
    cell0_r_a_10    : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_10)),
    cell0_r_a_11    : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_11)),
    cell0_r_a_12    : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_12)),
    cell0_r_a_13    : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_13)),
    cell0_r_a_14    : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_14))
  };

  BQ_STRUCT{
    uint16_t xcell0_r_a_flag;
    uint16_t xcell0_r_a_0;
    uint16_t xcell0_r_a_1;
    uint16_t xcell0_r_a_2;
    uint16_t xcell0_r_a_3;
    uint16_t xcell0_r_a_4;
    uint16_t xcell0_r_a_5;
    uint16_t xcell0_r_a_6;
    uint16_t xcell0_r_a_7;
    uint16_t xcell0_r_a_8;
    uint16_t xcell0_r_a_9;
    uint16_t xcell0_r_a_10;
    uint16_t xcell0_r_a_11;
    uint16_t xcell0_r_a_12;
    uint16_t xcell0_r_a_13;
    uint16_t xcell0_r_a_14;
  }r_a0x = {
    xcell0_r_a_flag  : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_FLAG)),
    xcell0_r_a_0     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_0)),
    xcell0_r_a_1     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_1)),
    xcell0_r_a_2     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_2)),
    xcell0_r_a_3     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_3)),
    xcell0_r_a_4     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_4)),
    xcell0_r_a_5     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_5)),
    xcell0_r_a_6     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_6)),
    xcell0_r_a_7     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_7)),
    xcell0_r_a_8     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_8)),
    xcell0_r_a_9     : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_9)),
    xcell0_r_a_10    : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_10)),
    xcell0_r_a_11    : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_11)),
    xcell0_r_a_12    : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_12)),
    xcell0_r_a_13    : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_13)),
    xcell0_r_a_14    : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_14))
  };

  BQ_STRUCT {
    uint32_t cc_gain;
    uint32_t cc_delta;
    uint16_t cc_offset;
    uint8_t board_offset;
    uint8_t int_temp_offset;
    uint8_t ext_temp_offset;
    uint8_t pack_v_offset;
  }calibration = {
    cc_gain         : ensureEndianness(static_cast<uint32_t>(float_to_bq_format(BQ27742_CC_GAIN))),
    cc_delta        : ensureEndianness(static_cast<uint32_t>(float_to_bq_format(BQ27742_CC_DELTA))),
    cc_offset       : ensureEndianness(static_cast<uint16_t>(BQ27742_CC_OFFSET_)),
    board_offset    : ensureEndianness(static_cast<uint8_t>(BQ27742_BOARD_OFFSET_)),
    int_temp_offset : ensureEndianness(static_cast<uint8_t>(BQ27742_INT_TEMP_OFFSET)),
    ext_temp_offset : ensureEndianness(static_cast<uint8_t>(BQ27742_EXT_TEMP_OFFSET)),
    pack_v_offset   : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_V_OFFSET))
  };

  BQ_STRUCT {
    uint8_t filter;
    uint8_t deadband;
    uint8_t cc_deadband;
  }current = {
    filter      : ensureEndianness(static_cast<uint8_t>(BQ27742_FILTER)),
    deadband    : ensureEndianness(static_cast<uint8_t>(BQ27742_DEADBAND)),
    cc_deadband : ensureEndianness(static_cast<uint8_t>(BQ27742_CC_DEADBAND))
  };

  BQ_STRUCT {
    uint32_t sealed_to_unsealed;
    uint32_t unsealed_to_full;
    uint32_t authen_key3;
    uint32_t authen_key2;
    uint32_t authen_key1;
    uint32_t authen_key0;
  } codes = {
    sealed_to_unsealed  : ensureEndianness(static_cast<uint32_t>(BQ27742_SEALED_TO_UNSEALED)),
    unsealed_to_full    : ensureEndianness(static_cast<uint32_t>(BQ27742_UNSEALED_TO_FULL)),
    authen_key3         : ensureEndianness(static_cast<uint32_t>(BQ27742_AUTHEN_KEY3)),
    authen_key2         : ensureEndianness(static_cast<uint32_t>(BQ27742_AUTHEN_KEY2)),
    authen_key1         : ensureEndianness(static_cast<uint32_t>(BQ27742_AUTHEN_KEY1)),
    authen_key0         : ensureEndianness(static_cast<uint32_t>(BQ27742_AUTHEN_KEY0))
  };
}

template <class T>
static void bq27742_program_flash(
    T &dataStruct,
    const uint8_t address,
    const uint8_t flashBlock = 0x00,
    const uint8_t offset = 0x00)
{
  bq27742_data_flash_write(
      address,
      flashBlock,
      offset,
      reinterpret_cast<const uint8_t *>(&dataStruct),
      sizeof(dataStruct)
      );
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
    ensureEndianness(GET_DATA_STRUCT(data_104).cc_gain);
  GET_DATA_STRUCT(data_104).cc_delta =
    ensureEndianness(GET_DATA_STRUCT(data_104).cc_delta);
  GET_DATA_STRUCT(data_104).cc_offset =
    ensureEndianness(GET_DATA_STRUCT(data_104).cc_offset);
  GET_DATA_STRUCT(data_104).board_offset =
    ensureEndianness(GET_DATA_STRUCT(data_104).board_offset);
  GET_DATA_STRUCT(data_104).int_temp_offset =
    ensureEndianness(GET_DATA_STRUCT(data_104).int_temp_offset);
  GET_DATA_STRUCT(data_104).ext_temp_offset =
    ensureEndianness(GET_DATA_STRUCT(data_104).ext_temp_offset);
  GET_DATA_STRUCT(data_104).pack_v_offset =
    ensureEndianness(GET_DATA_STRUCT(data_104).pack_v_offset);

  set_bq_register(BQ27742_BLOCK_DATA_CONTROL, 0x00);
  set_bq_register(BQ27742_DATA_FLASH_CLASS, 104);
  set_bq_register(BQ27742_DATA_FLASH_BLOCK, 0x00);

  uint8_t _reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};

  TWI_SEND_DATA(BQ, GET_RAW_DATA(data_104), sizeof(GET_RAW_DATA(data_104)), NULL, NULL);
  TWI_READ_DATA(BQ, BQ27742_BLOCK_DATA, _reg_value, sizeof(_reg_value), NULL, NULL);

  uint32_t _flash_date_sum = 0;
  for (auto i = 0ul; i < sizeof(_reg_value); ++i)
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

static bool bq27742_unsaled_set(void)
{
  uint8_t key1 [] = {0x14, 0x04};
  uint8_t key2 [] = {0x72, 0x36};

  set_bq_register_var(BQ27742_CONTROL, key1, sizeof(key1));
  vTaskDelay(pdMS_TO_TICKS(1));
  set_bq_register_var(BQ27742_CONTROL, key2, sizeof(key2));
  vTaskDelay(pdMS_TO_TICKS(1));

  return (bq27742_control_status_read() & (1<<BQ_FULL_SEALED_MODE)) ? true : false;
}

static bool bq27742_sealed_set ()
{
	uint16_t check_value = 0;
	uint16_t sealed_value = 1<<BQ_FULL_SEALED_MODE | 1<<BQ_SEALED_MODE;

        uint16_t _seal = ensureEndianness(static_cast<uint16_t>(BQ27742_SEALED));

	bq27742_write_control_data(BQ27742_SEALED);
	ic_delay_ms(1);

	check_value = bq27742_control_status_read();

	if (check_value && sealed_value)
		return true;
	else
		return false;
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

  GET_DATA_STRUCT(safety).ov_prot_trshold       = ensureEndianness(GET_DATA_STRUCT(safety).ov_prot_trshold);
  GET_DATA_STRUCT(safety).ov_prot_delay         = ensureEndianness(GET_DATA_STRUCT(safety).ov_prot_delay);
  GET_DATA_STRUCT(safety).ov_prot_recovery      = ensureEndianness(GET_DATA_STRUCT(safety).ov_prot_recovery);
  GET_DATA_STRUCT(safety).uv_prot_threshold     = ensureEndianness(GET_DATA_STRUCT(safety).uv_prot_threshold);
  GET_DATA_STRUCT(safety).uv_prot_delay         = ensureEndianness(GET_DATA_STRUCT(safety).uv_prot_delay);
  GET_DATA_STRUCT(safety).uv_prot_recovery      = ensureEndianness(GET_DATA_STRUCT(safety).uv_prot_recovery);
  GET_DATA_STRUCT(safety).body_diode_threshold  = ensureEndianness(GET_DATA_STRUCT(safety).body_diode_threshold);
  GET_DATA_STRUCT(safety).ot_chg                = ensureEndianness(GET_DATA_STRUCT(safety).ot_chg);
  GET_DATA_STRUCT(safety).ot_chg_time           = ensureEndianness(GET_DATA_STRUCT(safety).ot_chg_time);
  GET_DATA_STRUCT(safety).ot_chg_recovery       = ensureEndianness(GET_DATA_STRUCT(safety).ot_chg_recovery);
  GET_DATA_STRUCT(safety).ot_dsg                = ensureEndianness(GET_DATA_STRUCT(safety).ot_dsg);
  GET_DATA_STRUCT(safety).ot_dsg_time           = ensureEndianness(GET_DATA_STRUCT(safety).ot_dsg_time);
  GET_DATA_STRUCT(safety).ot_dsg_recovery       = ensureEndianness(GET_DATA_STRUCT(safety).ot_dsg_recovery);

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

  GET_DATA_STRUCT(charge).charging_voltage         = ensureEndianness(GET_DATA_STRUCT(charge).charging_voltage);

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

  GET_DATA_STRUCT(charge_termination).taper_current         = ensureEndianness(GET_DATA_STRUCT(charge_termination).taper_current);
  GET_DATA_STRUCT(charge_termination).min_taper_capacity    = ensureEndianness(GET_DATA_STRUCT(charge_termination).min_taper_capacity);
  GET_DATA_STRUCT(charge_termination).taper_voltage         = ensureEndianness(GET_DATA_STRUCT(charge_termination).taper_voltage);
  GET_DATA_STRUCT(charge_termination).current_taper_window  = ensureEndianness(GET_DATA_STRUCT(charge_termination).current_taper_window);
  GET_DATA_STRUCT(charge_termination).tca_set               = ensureEndianness(GET_DATA_STRUCT(charge_termination).tca_set);
  GET_DATA_STRUCT(charge_termination).tca_clear_procent     = ensureEndianness(GET_DATA_STRUCT(charge_termination).tca_clear_procent);
  GET_DATA_STRUCT(charge_termination).fc_set_procent        = ensureEndianness(GET_DATA_STRUCT(charge_termination).fc_set_procent);
  GET_DATA_STRUCT(charge_termination).fc_clear_procent      = ensureEndianness(GET_DATA_STRUCT(charge_termination).fc_clear_procent);
  GET_DATA_STRUCT(charge_termination).dodateoc_delta_t      = ensureEndianness(GET_DATA_STRUCT(charge_termination).dodateoc_delta_t);

  bq27742_data_flash_write(
      36,
      0x00,
      0x00,
      GET_RAW_DATA(charge_termination),
      sizeof(GET_RAW_DATA(charge_termination)));
}

void bq27742_program_flash_subclass_JEITA()
{
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

  GET_DATA_STRUCT(jeita).t1_temp            = ensureEndianness(GET_DATA_STRUCT(jeita).t1_temp);
  GET_DATA_STRUCT(jeita).t2_temp            = ensureEndianness(GET_DATA_STRUCT(jeita).t2_temp);
  GET_DATA_STRUCT(jeita).t3_temp            = ensureEndianness(GET_DATA_STRUCT(jeita).t3_temp);
  GET_DATA_STRUCT(jeita).t4_temp            = ensureEndianness(GET_DATA_STRUCT(jeita).t4_temp);
  GET_DATA_STRUCT(jeita).t5_temp            = ensureEndianness(GET_DATA_STRUCT(jeita).t5_temp);
  GET_DATA_STRUCT(jeita).temp_hys           = ensureEndianness(GET_DATA_STRUCT(jeita).temp_hys);
  GET_DATA_STRUCT(jeita).t1_t2_chg_voltage  = ensureEndianness(GET_DATA_STRUCT(jeita).t1_t2_chg_voltage);
  GET_DATA_STRUCT(jeita).t2_t3_chg_voltage  = ensureEndianness(GET_DATA_STRUCT(jeita).t2_t3_chg_voltage);
  GET_DATA_STRUCT(jeita).t3_t4_chg_voltage  = ensureEndianness(GET_DATA_STRUCT(jeita).t3_t4_chg_voltage);
  GET_DATA_STRUCT(jeita).t4_t5_chg_voltage  = ensureEndianness(GET_DATA_STRUCT(jeita).t4_t5_chg_voltage);
  GET_DATA_STRUCT(jeita).t1_t2_chg_current  = ensureEndianness(GET_DATA_STRUCT(jeita).t1_t2_chg_current);
  GET_DATA_STRUCT(jeita).t2_t3_chg_current  = ensureEndianness(GET_DATA_STRUCT(jeita).t2_t3_chg_current);
  GET_DATA_STRUCT(jeita).t3_t4_chg_current  = ensureEndianness(GET_DATA_STRUCT(jeita).t3_t4_chg_current);
  GET_DATA_STRUCT(jeita).t4_t5_chg_current  = ensureEndianness(GET_DATA_STRUCT(jeita).t4_t5_chg_current);

  bq27742_data_flash_write(
      BQ27742_JEITA_CLASS_ID,
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
  _bq27742_design_voltage = ensureEndianness(_bq27742_design_voltage);

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

  GET_DATA_STRUCT(data).cycle_count          = ensureEndianness(GET_DATA_STRUCT(data).cycle_count);
  GET_DATA_STRUCT(data).cc_threshold         = ensureEndianness(GET_DATA_STRUCT(data).cc_threshold);
  GET_DATA_STRUCT(data).design_capacity      = ensureEndianness(GET_DATA_STRUCT(data).design_capacity);
  GET_DATA_STRUCT(data).design_energy        = ensureEndianness(GET_DATA_STRUCT(data).design_energy);
  GET_DATA_STRUCT(data).soh_load_i           = ensureEndianness(GET_DATA_STRUCT(data).soh_load_i);
  GET_DATA_STRUCT(data).tdd_soh_percent      = ensureEndianness(GET_DATA_STRUCT(data).tdd_soh_percent);
  GET_DATA_STRUCT(data).isd_current          = ensureEndianness(GET_DATA_STRUCT(data).isd_current);
  GET_DATA_STRUCT(data).isd_i_filter         = ensureEndianness(GET_DATA_STRUCT(data).isd_i_filter);
  GET_DATA_STRUCT(data).min_isd_time         = ensureEndianness(GET_DATA_STRUCT(data).min_isd_time);
  GET_DATA_STRUCT(data).design_energy_scale  = ensureEndianness(GET_DATA_STRUCT(data).design_energy_scale);

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
    ensureEndianness(GET_DATA_STRUCT(discharge).soc1_set_threshold);
  GET_DATA_STRUCT(discharge).soc1_clear_threshold     =
    ensureEndianness(GET_DATA_STRUCT(discharge).soc1_clear_threshold);
  GET_DATA_STRUCT(discharge).socf_set_threshold       =
    ensureEndianness(GET_DATA_STRUCT(discharge).socf_set_threshold);
  GET_DATA_STRUCT(discharge).socf_clear_threshold     =
    ensureEndianness(GET_DATA_STRUCT(discharge).socf_clear_threshold);
  GET_DATA_STRUCT(discharge).bl_set_volt_threshold    =
    ensureEndianness(GET_DATA_STRUCT(discharge).bl_set_volt_threshold);
  GET_DATA_STRUCT(discharge).bl_set_volt_time         =
    ensureEndianness(GET_DATA_STRUCT(discharge).bl_set_volt_time);
  GET_DATA_STRUCT(discharge).bl_clear_volt_threshold  =
    ensureEndianness(GET_DATA_STRUCT(discharge).bl_clear_volt_threshold);
  GET_DATA_STRUCT(discharge).bh_set_volt_threshold    =
    ensureEndianness(GET_DATA_STRUCT(discharge).bh_set_volt_threshold);
  GET_DATA_STRUCT(discharge).bh_volt_time             =
    ensureEndianness(GET_DATA_STRUCT(discharge).bh_volt_time);
  GET_DATA_STRUCT(discharge).bh_clear_volt_threshold  =
    ensureEndianness(GET_DATA_STRUCT(discharge).bh_clear_volt_threshold);

  bq27742_data_flash_write(
      BQ27742_DISCHARGE_CLASS_ID,      // subclass_id
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
    ensureEndianness(GET_DATA_STRUCT(manu_data).pack_lot_code);
  GET_DATA_STRUCT(manu_data).pcb_lot_code       =
    ensureEndianness(GET_DATA_STRUCT(manu_data).pcb_lot_code);
  GET_DATA_STRUCT(manu_data).firmware_version   =
    ensureEndianness(GET_DATA_STRUCT(manu_data).firmware_version);
  GET_DATA_STRUCT(manu_data).hardware_revision  =
    ensureEndianness(GET_DATA_STRUCT(manu_data).hardware_revision);
  GET_DATA_STRUCT(manu_data).cell_revision      =
    ensureEndianness(GET_DATA_STRUCT(manu_data).cell_revision);
  GET_DATA_STRUCT(manu_data).df_config_version  =
    ensureEndianness(GET_DATA_STRUCT(manu_data).df_config_version);

  bq27742_data_flash_write(
      BQ27742_MANUFACTURER_DATA_CLASS_ID,  // subclass_id
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
    ensureEndianness(GET_DATA_STRUCT(integrity_data).all_df_checksum);
  GET_DATA_STRUCT(integrity_data).static_chem_df_checksum  =
    ensureEndianness(GET_DATA_STRUCT(integrity_data).static_chem_df_checksum);
  GET_DATA_STRUCT(integrity_data).static_df_checksum       =
    ensureEndianness(GET_DATA_STRUCT(integrity_data).static_df_checksum);
  GET_DATA_STRUCT(integrity_data).prot_checksum            =
    ensureEndianness(GET_DATA_STRUCT(integrity_data).prot_checksum);

  bq27742_data_flash_write(
      BQ27742_INTEGRITY_CLASS_ID,                // subclass_id
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
    ensureEndianness(GET_DATA_STRUCT(lifetime_data).max_temp);
  GET_DATA_STRUCT(lifetime_data).min_temp          =
    ensureEndianness(GET_DATA_STRUCT(lifetime_data).min_temp);
  GET_DATA_STRUCT(lifetime_data).max_pack_voltage  =
    ensureEndianness(GET_DATA_STRUCT(lifetime_data).max_pack_voltage);
  GET_DATA_STRUCT(lifetime_data).min_pack_voltage  =
    ensureEndianness(GET_DATA_STRUCT(lifetime_data).min_pack_voltage);
  GET_DATA_STRUCT(lifetime_data).max_chg_current   =
    ensureEndianness(GET_DATA_STRUCT(lifetime_data).max_chg_current);
  GET_DATA_STRUCT(lifetime_data).max_dsg_current   =
    ensureEndianness(GET_DATA_STRUCT(lifetime_data).max_dsg_current);

  bq27742_data_flash_write(
      BQ27742_LIFETIME_DATA_CLASS_ID,          // subclass_id
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
    ensureEndianness(GET_DATA_STRUCT(lifetime_temp_samples).lt_flash_cnt);
  GET_DATA_STRUCT(lifetime_temp_samples).lt_afe_status  =
    ensureEndianness(GET_DATA_STRUCT(lifetime_temp_samples).lt_afe_status);

  bq27742_data_flash_write(
      BQ27742_LIFETIME_TEMP_SAMPLES_CLASS_ID,  // subclass_id
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
    ensureEndianness(GET_DATA_STRUCT(registers).pack_configuration);
  GET_DATA_STRUCT(registers).pack_configuration_b  =
    ensureEndianness(GET_DATA_STRUCT(registers).pack_configuration_b);
  GET_DATA_STRUCT(registers).pack_configuration_c  =
    ensureEndianness(GET_DATA_STRUCT(registers).pack_configuration_c);
  GET_DATA_STRUCT(registers).pack_configuration_d  =
    ensureEndianness(GET_DATA_STRUCT(registers).pack_configuration_d);
  GET_DATA_STRUCT(registers).prot_oc_config        =
    ensureEndianness(GET_DATA_STRUCT(registers).prot_oc_config);
  GET_DATA_STRUCT(registers).prot_ov_config        =
    ensureEndianness(GET_DATA_STRUCT(registers).prot_ov_config);

  bq27742_data_flash_write(
      BQ27742_REGISTERS_CLASS_ID,          // subclass_id
      0x00,                             // flash_block
      0x00,                             // offset
      GET_RAW_DATA(registers),          // value
      sizeof(GET_RAW_DATA(registers))); // len
}

void bq27742_program_flash_subclass_lifetime_resolution()
{
  //uint8_t subclass_id=66;
  //uint8_t flash_block=0;
  //uint8_t offset=0;

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
    ensureEndianness(GET_DATA_STRUCT(lifetime_res).lt_temp_res);
  GET_DATA_STRUCT(lifetime_res).lt_v_res                =
    ensureEndianness(GET_DATA_STRUCT(lifetime_res).lt_v_res);
  GET_DATA_STRUCT(lifetime_res).lt_cur_res              =
    ensureEndianness(GET_DATA_STRUCT(lifetime_res).lt_cur_res);
  GET_DATA_STRUCT(lifetime_res).lt_update_time          =
    ensureEndianness(GET_DATA_STRUCT(lifetime_res).lt_update_time);
  GET_DATA_STRUCT(lifetime_res).flash_update_ok_voltage =
    ensureEndianness(GET_DATA_STRUCT(lifetime_res).flash_update_ok_voltage);

  bq27742_data_flash_write(
      BQ27742_LIFETIME_RES_CLASS_ID,           // subclass_id
      0x00,                                 // flash_block
      0x00,                                 // offset
      GET_RAW_DATA(lifetime_res),           // value
      sizeof(GET_RAW_DATA(lifetime_res)));  // len
}

void bq27742_program_flash_subclass_power()
{
  DEFINE_DATA_STRUCT(power ,{
      uint16_t flash_update_ok_voltage;
      uint16_t sleep_current;
      });

  GET_DATA_STRUCT(power).flash_update_ok_voltage  = BQ27742_FLASH_UPDATE_OK_VOLTAGE;
  GET_DATA_STRUCT(power).sleep_current            = BQ27742_SLEEP_CURRENT;

  GET_DATA_STRUCT(power).flash_update_ok_voltage  =
    ensureEndianness(GET_DATA_STRUCT(power).flash_update_ok_voltage);
  GET_DATA_STRUCT(power).sleep_current            =
    ensureEndianness(GET_DATA_STRUCT(power).sleep_current);

  bq27742_data_flash_write(
      BQ27742_POWER_CLASS_ID,          // subclass_id
      0x00,                         // flash_block
      0x00,                         // offset
      GET_RAW_DATA(power),          // value
      sizeof(GET_RAW_DATA(power))); // len

  uint16_t _bq27742_shutdown_v = BQ27742_SHUTDOWN_V;
  _bq27742_shutdown_v = ensureEndianness(_bq27742_shutdown_v);

  uint8_t _bq27742_fs_wait = BQ27742_FS_WAIT;

  bq27742_data_flash_write(
      BQ27742_POWER_CLASS_ID,
      0x00,
      0x0A,
      reinterpret_cast<uint8_t *>(&_bq27742_shutdown_v),
      sizeof(_bq27742_shutdown_v));

  bq27742_data_flash_write(
      BQ27742_POWER_CLASS_ID,
      0x00,
      0x0C,
      reinterpret_cast<uint8_t *>(&_bq27742_fs_wait),
      sizeof(_bq27742_fs_wait));
}

static void bq27742_program_flash_subclass_if_cfg1(void){
  bq27742_program_flash(bqSubclassData::load_select,                 BQ27742_IT_CFG_CLASS_ID, 0x00, 0x00);
  bq27742_program_flash(bqSubclassData::load_mode,                   BQ27742_IT_CFG_CLASS_ID, 0x00, 0x01);
  bq27742_program_flash(bqSubclassData::max_res_factor,              BQ27742_IT_CFG_CLASS_ID, 0x00, 0x11);
  bq27742_program_flash(bqSubclassData::min_res_factor,              BQ27742_IT_CFG_CLASS_ID, 0x00, 0x12);
  bq27742_program_flash(bqSubclassData::ra_filter,                   BQ27742_IT_CFG_CLASS_ID, 0x00, 0x14);
  bq27742_program_flash(bqSubclassData::res_v_drop,                  BQ27742_IT_CFG_CLASS_ID, 0x00, 0x16);
  bq27742_program_flash(bqSubclassData::fast_qmax_start_dod_procent, BQ27742_IT_CFG_CLASS_ID, 0x01, 0x07);
  bq27742_program_flash(bqSubclassData::fast_qmax_end_dod_procent,   BQ27742_IT_CFG_CLASS_ID, 0x01, 0x08);
  bq27742_program_flash(bqSubclassData::fast_qmax_star_volt_delta,   BQ27742_IT_CFG_CLASS_ID, 0x01, 0x09);
  bq27742_program_flash(bqSubclassData::fast_qmax_current_threshold, BQ27742_IT_CFG_CLASS_ID, 0x01, 0x0B);
  bq27742_program_flash(bqSubclassData::qmax_capacity_err,           BQ27742_IT_CFG_CLASS_ID, 0x01, 0x1D);
  bq27742_program_flash(bqSubclassData::max_qmax_change,             BQ27742_IT_CFG_CLASS_ID, 0x01, 0x1E);
  bq27742_program_flash(bqSubclassData::terminate_voltage,           BQ27742_IT_CFG_CLASS_ID, 0x02, 0x00);
  bq27742_program_flash(bqSubclassData::term_v_delta,                BQ27742_IT_CFG_CLASS_ID, 0x02, 0x02);
  bq27742_program_flash(bqSubclassData::resrelax_time,               BQ27742_IT_CFG_CLASS_ID, 0x02, 0x05);
  bq27742_program_flash(bqSubclassData::user_rate_ma,                BQ27742_IT_CFG_CLASS_ID, 0x02, 0x09);
  bq27742_program_flash(bqSubclassData::user_rate_pwr,               BQ27742_IT_CFG_CLASS_ID, 0x02, 0x0B);
  bq27742_program_flash(bqSubclassData::reserve_cap_mah,             BQ27742_IT_CFG_CLASS_ID, 0x02, 0x0D);
  bq27742_program_flash(bqSubclassData::max_deltav,                  BQ27742_IT_CFG_CLASS_ID, 0x02, 0x14);
  bq27742_program_flash(bqSubclassData::min_deltav,                  BQ27742_IT_CFG_CLASS_ID, 0x02, 0x16);
  bq27742_program_flash(bqSubclassData::max_sim_rate,                BQ27742_IT_CFG_CLASS_ID, 0x02, 0x18);
  bq27742_program_flash(bqSubclassData::min_sim_rate,                BQ27742_IT_CFG_CLASS_ID, 0x02, 0x19);
  bq27742_program_flash(bqSubclassData::ra_max_delta,                BQ27742_IT_CFG_CLASS_ID, 0x02, 0x1A);
  bq27742_program_flash(bqSubclassData::trace_resistance,            BQ27742_IT_CFG_CLASS_ID, 0x02, 0x1C);
  bq27742_program_flash(bqSubclassData::downstream_resistance,       BQ27742_IT_CFG_CLASS_ID, 0x02, 0x1E);
  bq27742_program_flash(bqSubclassData::qmax_max_delta_procent,      BQ27742_IT_CFG_CLASS_ID, 0x03, 0x00);
  bq27742_program_flash(bqSubclassData::qmax_bound_procent,          BQ27742_IT_CFG_CLASS_ID, 0x03, 0x01);
  bq27742_program_flash(bqSubclassData::deltav_max_delta,            BQ27742_IT_CFG_CLASS_ID, 0x03, 0x02);
  bq27742_program_flash(bqSubclassData::max_res_scale,               BQ27742_IT_CFG_CLASS_ID, 0x03, 0x04);
  bq27742_program_flash(bqSubclassData::min_res_scale,               BQ27742_IT_CFG_CLASS_ID, 0x03, 0x06);
  bq27742_program_flash(bqSubclassData::fast_scale_start_soc,        BQ27742_IT_CFG_CLASS_ID, 0x03, 0x08);
  bq27742_program_flash(bqSubclassData::fast_scale_load_select,      BQ27742_IT_CFG_CLASS_ID, 0x03, 0x09);
  bq27742_program_flash(bqSubclassData::charge_hys_v_shift,          BQ27742_IT_CFG_CLASS_ID, 0x03, 0x0A);
  bq27742_program_flash(bqSubclassData::rascl_ocv_rst_temp_thresh,   BQ27742_IT_CFG_CLASS_ID, 0x03, 0x0C);
  bq27742_program_flash(bqSubclassData::max_allowed_current,         BQ27742_IT_CFG_CLASS_ID, 0x03, 0x0D);
  bq27742_program_flash(bqSubclassData::max_current_pulse_duration,  BQ27742_IT_CFG_CLASS_ID, 0x03, 0x0F);
  bq27742_program_flash(bqSubclassData::max_current_interrupt_step,  BQ27742_IT_CFG_CLASS_ID, 0x03, 0x10);
}

void ic_bq_flash_image(){
  bq27742_unsaled_set();
  bq27742_full_access_set();

  bq27742_program_flash(bqSubclassData::safety, BQ27742_SAFETY_CLASS_ID);
  bq27742_program_flash(bqSubclassData::charge, BQ27742_CHARGE_CLASS_ID);
  bq27742_program_flash(bqSubclassData::charge_termination, BQ27742_CHARGE_TERMINATION_CLASS_ID);
  bq27742_program_flash(bqSubclassData::jeita, BQ27742_JEITA_CLASS_ID);
  bq27742_program_flash(bqSubclassData::design_voltage, BQ27742_DATA_CLASS_ID);
  bq27742_program_flash(bqSubclassData::data, BQ27742_DATA_CLASS_ID, 0x00, 0x08);
  bq27742_program_flash(bqSubclassData::discharge, BQ27742_DISCHARGE_CLASS_ID);
  bq27742_program_flash(bqSubclassData::manu_data, BQ27742_MANUFACTURER_DATA_CLASS_ID);
  bq27742_program_flash(bqSubclassData::integrity_data, BQ27742_INTEGRITY_CLASS_ID);
  bq27742_program_flash(bqSubclassData::lifetime_data, BQ27742_LIFETIME_DATA_CLASS_ID);
  bq27742_program_flash(bqSubclassData::lifetime_temp_samples, BQ27742_LIFETIME_TEMP_SAMPLES_CLASS_ID);
  bq27742_program_flash(bqSubclassData::registers, BQ27742_REGISTERS_CLASS_ID);
  bq27742_program_flash(bqSubclassData::lifetime_res, BQ27742_LIFETIME_RES_CLASS_ID);
  bq27742_program_flash(bqSubclassData::power, BQ27742_POWER_CLASS_ID);
  bq27742_program_flash(bqSubclassData::shutdown_v, BQ27742_POWER_CLASS_ID, 0x00, 0x0A);
  bq27742_program_flash(bqSubclassData::fs_wait, BQ27742_POWER_CLASS_ID, 0x00, 0x0C);
  bq27742_program_flash(bqSubclassData::manufacturer_info1, BQ27742_MANUFACTURER_INFO_CLASS_ID);
  bq27742_program_flash(bqSubclassData::manufacturer_info2, BQ27742_MANUFACTURER_INFO_CLASS_ID, 0x01, 0x00);

  bq27742_program_flash_subclass_if_cfg1();

  bq27742_program_flash(bqSubclassData::current_thresholds, BQ27742_CURRENT_TRESHOLD_CLASS_ID);
  bq27742_program_flash(bqSubclassData::state, BQ27742_STATE_CLASS_ID);
  bq27742_program_flash(bqSubclassData::chem_id, BQ27742_CHEM_ID_CLASS_ID);
  bq27742_program_flash(bqSubclassData::chem_id, BQ27742_CHEM_ID_CLASS_ID);
  bq27742_program_flash(bqSubclassData::r_a0, BQ27742_R_A0_CLASS_ID);
  bq27742_program_flash(bqSubclassData::r_a0x, BQ27742_R_A0X_CLASS_ID);
  bq27742_program_flash(bqSubclassData::calibration, BQ27742_CALIBRATION_CLASS_ID);
  bq27742_program_flash(bqSubclassData::current, BQ27742_CURRENT_CLASS_ID);
  bq27742_program_flash(bqSubclassData::codes, BQ27742_CODES_CLASS_ID);
}
