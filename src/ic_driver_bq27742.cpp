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
#include <cstring>
#include <climits>
#include <cmath>

#include "ic_driver_twi.h"
#include "ic_service_ltc.h"

#include "FreeRTOS.h"
#include "ic_driver_bq27742_definitions.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


extern "C" {
#define NRF_LOG_MODULE_NAME "BQ"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
}

#include "nrf_delay.h"

#include "ic_driver_bq27742.h"

//const float BQ27742_CC_GAIN=4.768/54.931;
const float BQ27742_CC_GAIN=4.769/54.931;
const float BQ27742_CC_DELTA=5677445.0/54.810;

//#define FORCE_ENDIANNESS __ORDER_LITTLE_ENDIAN__
#define FORCE_ENDIANNESS __ORDER_BIG_ENDIAN__
//#define DEBUG_BQ

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

#define BQ27742_US_WAIT_TIME 66

#define BQ_SEND_DATA(subclass_id, flash_block, flash_data_access, block_data_access, data, size) do{  \
  uint8_t _reg_value [BQ27742_DATA_FLASH_BLOCK_SIZE] = {0};                                           \
  set_bq_register(BQ27742_BLOCK_DATA_CONTROL, (uint8_t)flash_data_access);                            \
  vTaskDelay(BQ27742_US_WAIT_TIME);                                                                   \
  set_bq_register(BQ27742_DATA_FLASH_CLASS, (uint8_t)subclass_id);                                    \
  vTaskDelay(BQ27742_US_WAIT_TIME);                                                                   \
  set_bq_register(BQ27742_DATA_FLASH_BLOCK, (uint8_t)flash_block);                                    \
  vTaskDelay(BQ27742_US_WAIT_TIME);                                                                   \
  set_bq_register(block_data_access, data, size);                                                     \
  nrf_delay_us(BQ27742_US_WAIT_TIME);                                                                 \
  TWI_READ_DATA(BQ, BQ27742_BLOCK_DATA, _reg_value, BQ27742_DATA_FLASH_BLOCK_SIZE, NULL, NULL);       \
  uint32_t _flash_data_sum = 0;                                                                       \
  for (int i = 0; i < BQ27742_DATA_FLASH_BLOCK_SIZE; i++){                                            \
    _flash_data_sum += _reg_value[i];                                                                 \
  }                                                                                                   \
  uint8_t _checksum = 255 - (uint8_t)(_flash_data_sum);                                               \
  set_bq_register(BQ27742_BLOCK_DATA_CS, _checksum);                                                  \
  vTaskDelay(pdMS_TO_TICKS(BQ27742_REGISTER_WRITE_DELAY));                                            \
}while(0)

TWI_REGISTER(BQ, IC_BQ27742_TWI_ADDRESS);

template <class T>
static ic_return_val_e set_bq_register(const uint8_t reg_addr, const T &&data){
  uint8_t _buffer[sizeof(T)+1];
  _buffer[0] = reg_addr;
  std::memcpy(&_buffer[1], &data, sizeof(T));

  #ifdef DEBUG_BQ
  return IC_SUCCESS;
  NRF_LOG_RAW_INFO("RR reg_addr: %d: of size: %d \n", reg_addr, sizeof(T));
  for(auto byte : _buffer)
    NRF_LOG_RAW_INFO("0x%02X ", byte);
  NRF_LOG_RAW_INFO("\n");
  #else
  return TWI_SEND_DATA(BQ, _buffer, sizeof(_buffer), NULL, NULL);
  #endif
}

template <class T>
static ic_return_val_e set_bq_register(const uint8_t reg_addr, const T &data){
  uint8_t _buffer[sizeof(T)+1];
  _buffer[0] = reg_addr;
  std::memcpy(&_buffer[1], &data, sizeof(T));

  #ifdef DEBUG_BQ
  return IC_SUCCESS;
  NRF_LOG_RAW_INFO("R reg_addr: %d: of size: %d \n", reg_addr, sizeof(T));
  for(auto byte : _buffer)
    NRF_LOG_RAW_INFO("0x%02X ", byte);
  NRF_LOG_RAW_INFO("\n");
  #else
  return TWI_SEND_DATA(BQ, _buffer, sizeof(_buffer), NULL, NULL);
  #endif
}

template <class T>
static ic_return_val_e set_bq_register(const uint8_t reg_addr, const T *data, size_t len){
  uint8_t _buffer[len+1];
  _buffer[0] = reg_addr;
  std::memcpy(&_buffer[1], data, len);

  #ifdef DEBUG_BQ
  return IC_SUCCESS;
  NRF_LOG_RAW_INFO("P reg_addr: %d: of size: %d \n", reg_addr, sizeof(_buffer));
  for(auto byte : _buffer)
    NRF_LOG_RAW_INFO("0x%02X ", byte);
  NRF_LOG_RAW_INFO("\n");
  #else
//  for(auto byte : _buffer)
//    NRF_LOG_RAW_INFO("0x%02X ", byte);
//  NRF_LOG_RAW_INFO("\n");
  #endif
  return TWI_SEND_DATA(BQ, _buffer, sizeof(_buffer), NULL, NULL);
}

template <class T>
static ic_return_val_e get_bq_register(const uint8_t reg_addr, T &data){
  uint8_t _buffer[sizeof(T)];

  TWI_READ_DATA(BQ, reg_addr , _buffer, sizeof(_buffer), NULL, NULL);

  std::memcpy(&data, _buffer , sizeof(T));

  #ifdef DEBUG_BQ
  NRF_LOG_RAW_INFO("R reg_addr: %d: of size: %d \n", reg_addr, sizeof(T));
  for(auto byte : _buffer)
    NRF_LOG_RAW_INFO("0x%02X ", byte);
  NRF_LOG_RAW_INFO("\n");
  return IC_SUCCESS;
  #else
  return IC_SUCCESS;
  #endif
}

template <class cbArg>
static ic_return_val_e get_bq_register(const uint8_t reg_addr, void(*cb)(cbArg)){
  static uint8_t _buffer[sizeof(cbArg)];

  static auto _lamda = [&cb](ic_return_val_e ret_val, void *p_context){
    for(auto _byte : _buffer)
      NRF_LOG_RAW_INFO("0x%02X ", _byte);
    NRF_LOG_RAW_INFO("\n");

    cb(*reinterpret_cast<cbArg *>(_buffer));
  };

  return TWI_READ_DATA(
      BQ,
      reg_addr
      ,_buffer,
      sizeof(_buffer),
      _lamda,
      NULL);
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
  NRF_LOG_RAW_INFO("Write data to(0x%02x) offset:%d\n",subclass_id,offset);
  for (uint8_t i = 0; i < len; i++){
    NRF_LOG_RAW_INFO("0x%02x ",value[i]);
  }
  NRF_LOG_RAW_INFO("\n");

  #ifndef DEBUG_BQ
  BQ_SEND_DATA(
      subclass_id,                    // subclass_ID
      flash_block,                    // flash_block
      (uint8_t)0x00,                           // flash_data_access
      (BQ27742_BLOCK_DATA + offset),  // block_data_access
      value,                          // data
      len);                           // size
  #else
  BQ_SEND_DATA_NOT(
      subclass_id,                    // subclass_ID
      flash_block,                    // flash_block
      (uint8_t)0x00,                           // flash_data_access
      (BQ27742_BLOCK_DATA + offset),  // block_data_access
      value,                          // data
      len);                           // size
  #endif
}

static float fast_power(float base, int16_t index){
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
  }
  ret_val = {
    raw : 0
  };

  int16_t exp=0;
  float mod_val = 0.0;
  float tmp_val = 0.0;

  mod_val=std::fabs(val);

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

  ret_val.byte[0]=exp+128;
  ret_val.byte[1]=(uint8_t)tmp_val;
  tmp_val=256*(tmp_val-ret_val.byte[1]);
  ret_val.byte[2]=(uint8_t)tmp_val;
  tmp_val=256*(tmp_val-ret_val.byte[2]);
  ret_val.byte[3]=(uint8_t)tmp_val;

  if(val<0)
    ret_val.byte[2] |= 0x80;

  return ret_val.raw;
}

namespace bqSubclassData{

  #define BQ_STRUCT static const struct __attribute__((packed))

  BQ_STRUCT{
    int16_t ov_prot_treshold;
    uint8_t ov_prot_delay;
    int16_t ov_prot_recovery;
    int16_t uv_prot_threshold;
    uint8_t uv_prot_delay;
    int16_t uv_prot_recovery;
    int16_t body_diode_threshold;
    int16_t ot_chg;
    uint8_t ot_chg_time;
    int16_t ot_chg_recovery;
    int16_t ot_dsg;
    uint8_t ot_dsg_time;
    int16_t ot_dsg_recovery;
  }safety = {
    ov_prot_treshold      : ensureEndianness(static_cast<int16_t>(BQ27742_OV_PROT_THRESHOLD)),
    ov_prot_delay         : ensureEndianness(static_cast<uint8_t>(BQ27742_OV_PROT_DELAY)),
    ov_prot_recovery      : ensureEndianness(static_cast<int16_t>(BQ27742_OV_PROT_RECOVERY)),
    uv_prot_threshold     : ensureEndianness(static_cast<int16_t>(BQ27742_UV_PROT_THRESHOLD)),
    uv_prot_delay         : ensureEndianness(static_cast<uint8_t>(BQ27742_UV_PROT_DELAY)),
    uv_prot_recovery      : ensureEndianness(static_cast<int16_t>(BQ27742_UV_PROT_RECOVERY)),
    body_diode_threshold  : ensureEndianness(static_cast<int16_t>(BQ27742_BODY_DIODE_THRESHOLD)),
    ot_chg                : ensureEndianness(static_cast<int16_t>(BQ27742_OT_CHG)),
    ot_chg_time           : ensureEndianness(static_cast<uint8_t>(BQ27742_OT_CHG_TIME)),
    ot_chg_recovery       : ensureEndianness(static_cast<int16_t>(BQ27742_OT_CHG_RECOVERY)),
    ot_dsg                : ensureEndianness(static_cast<int16_t>(BQ27742_OT_DSG)),
    ot_dsg_time           : ensureEndianness(static_cast<uint8_t>(BQ27742_OT_DSG_TIME)),
    ot_dsg_recovery       : ensureEndianness(static_cast<int16_t>(BQ27742_OT_DSG_RECOVERY))
  };

  BQ_STRUCT{
    int16_t charging_voltage;
    uint32_t cleaner;
  }charge = {
    charging_voltage :ensureEndianness(static_cast<int16_t>(BQ27742_CHARGING_VOLTAGE_VAL)),
    cleaner : 0x04
  };

  BQ_STRUCT{
    int16_t taper_current;
    int16_t min_taper_capacity;
    int16_t taper_voltage;
    uint8_t current_taper_window;
    int8_t  tca_set;
    int8_t  tca_clear_procent;
    int8_t  fc_set_procent;
    int8_t  fc_clear_procent;
    int16_t dodateoc_delta_t;
  }charge_termination = {
    taper_current         : ensureEndianness(static_cast<int16_t>(BQ27742_TAPER_CURRENT)),
    min_taper_capacity    : ensureEndianness(static_cast<int16_t>(BQ27742_MIN_TAPER_CAPACITY)),
    taper_voltage         : ensureEndianness(static_cast<int16_t>(BQ27742_TAPER_VOLTAGE)),
    current_taper_window  : ensureEndianness(static_cast<uint8_t>(BQ27742_CURRENT_TAPER_WINDOW)),
    tca_set               : ensureEndianness(static_cast<int8_t>(BQ27742_TCA_SET)),
    tca_clear_procent     : ensureEndianness(static_cast<int8_t>(BQ27742_TCA_CLEAR_PROCENT)),
    fc_set_procent        : ensureEndianness(static_cast<int8_t>(BQ27742_FC_SET_PROCENT)),
    fc_clear_procent      : ensureEndianness(static_cast<int8_t>(BQ27742_FC_CLEAR_PROCENT)),
    dodateoc_delta_t      : ensureEndianness(static_cast<int16_t>(BQ27742_DODATEOC_DELTA_T))
  };

  BQ_STRUCT{
    int8_t  t1_temp;
    int8_t  t2_temp;
    int8_t  t3_temp;
    int8_t  t4_temp;
    int8_t  t5_temp;
    int8_t  temp_hys;
    int16_t t1_t2_chg_voltage;
    int16_t t2_t3_chg_voltage;
    int16_t t3_t4_chg_voltage;
    int16_t t4_t5_chg_voltage;
    uint8_t t1_t2_chg_current;
    uint8_t t2_t3_chg_current;
    uint8_t t3_t4_chg_current;
    uint8_t t4_t5_chg_current;
  }jeita = {
    t1_temp            : ensureEndianness(static_cast<int8_t>(BQ27742_T1_TEMP)),
    t2_temp            : ensureEndianness(static_cast<int8_t>(BQ27742_T2_TEMP)),
    t3_temp            : ensureEndianness(static_cast<int8_t>(BQ27742_T3_TEMP)),
    t4_temp            : ensureEndianness(static_cast<int8_t>(BQ27742_T4_TEMP)),
    t5_temp            : ensureEndianness(static_cast<int8_t>(BQ27742_T5_TEMP)),
    temp_hys           : ensureEndianness(static_cast<int8_t>(BQ27742_TEMP_HYS)),
    t1_t2_chg_voltage  : ensureEndianness(static_cast<int16_t>(BQ27742_T1_T2_CHG_VOLTAGE)),
    t2_t3_chg_voltage  : ensureEndianness(static_cast<int16_t>(BQ27742_T2_T3_CHG_VOLTAGE)),
    t3_t4_chg_voltage  : ensureEndianness(static_cast<int16_t>(BQ27742_T3_T4_CHG_VOLTAGE)),
    t4_t5_chg_voltage  : ensureEndianness(static_cast<int16_t>(BQ27742_T4_T5_CHG_VOLTAGE)),
    t1_t2_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T1_T2_CHG_CURRENT)),
    t2_t3_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T2_T3_CHG_CURRENT)),
    t3_t4_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T3_T4_CHG_CURRENT)),
    t4_t5_chg_current  : ensureEndianness(static_cast<uint8_t>(BQ27742_T4_T5_CHG_CURRENT))
  };

  static const int16_t design_voltage = ensureEndianness(static_cast<uint16_t>(BQ27742_DESIGN_VOLTAGE));

  BQ_STRUCT{
    uint16_t  cycle_count;        // 8
    int16_t   cc_threshold;       // 10
    int16_t   design_capacity;    // 12
    int16_t   design_energy;      // 14
    int16_t   soh_load_i;         // 16
    uint8_t   tdd_soh_percent;    // 18
    uint16_t  isd_current;        // 19
    uint8_t   isd_i_filter;       // 21
    uint8_t   min_isd_time;       // 22
    uint8_t   design_energy_scale;// 23
  }data = {
    cycle_count         : ensureEndianness(static_cast<uint16_t>(BQ27742_CYCLE_COUNT_)),
    cc_threshold        : ensureEndianness(static_cast<int16_t>(BQ27742_CC_THRESHOLD)),
    design_capacity     : ensureEndianness(static_cast<int16_t>(BQ27742_DESIGN_CAPACITY_)),
    design_energy       : ensureEndianness(static_cast<int16_t>(BQ27742_DESIGN_ENERGY)),
    soh_load_i          : ensureEndianness(static_cast<int16_t>(BQ27742_SOH_LOAD_I)),
    tdd_soh_percent     : ensureEndianness(static_cast<uint8_t>(BQ27742_TDD_SOH_PERCENT)),
    isd_current         : ensureEndianness(static_cast<uint16_t>(BQ27742_ISD_CURRENT)),
    isd_i_filter        : ensureEndianness(static_cast<uint8_t>(BQ27742_ISD_I_FILTER)),
    min_isd_time        : ensureEndianness(static_cast<uint8_t>(BQ27742_MIN_ISD_TIME)),
    design_energy_scale : ensureEndianness(static_cast<uint8_t>(BQ27742_DESIGN_ENERGY_SCALE))
  };

  BQ_STRUCT{
    uint16_t  soc1_set_threshold;     // 0
    uint16_t  soc1_clear_threshold;   // 2
    uint16_t  socf_set_threshold;     // 4
    uint16_t  socf_clear_threshold;   // 6
    int16_t   bl_set_volt_threshold;  // 8
    uint8_t   bl_set_volt_time;       // 10
    int16_t   bl_clear_volt_threshold;// 11
    int16_t   bh_set_volt_threshold;  // 13
    uint8_t   bh_volt_time;           // 15
    int16_t   bh_clear_volt_threshold;// 16
  }discharge = {
    soc1_set_threshold       : ensureEndianness(static_cast<uint16_t>(BQ27742_SOC1_SET_THRESHOLD)),
    soc1_clear_threshold     : ensureEndianness(static_cast<uint16_t>(BQ27742_SOC1_CLEAR_THRESHOLD)),
    socf_set_threshold       : ensureEndianness(static_cast<uint16_t>(BQ27742_SOCF_SET_THRESHOLD)),
    socf_clear_threshold     : ensureEndianness(static_cast<uint16_t>(BQ27742_SOCF_CLEAR_THRESHOLD)),
    bl_set_volt_threshold    : ensureEndianness(static_cast<int16_t>(BQ27742_BL_SET_VOLT_THRESHOLD)),
    bl_set_volt_time         : ensureEndianness(static_cast<uint8_t>(BQ27742_BL_SET_VOLT_TIME)),
    bl_clear_volt_threshold  : ensureEndianness(static_cast<int16_t>(BQ27742_BL_CLEAR_VOLT_THRESHOLD)),
    bh_set_volt_threshold    : ensureEndianness(static_cast<int16_t>(BQ27742_BH_SET_VOLT_THRESHOLD)),
    bh_volt_time             : ensureEndianness(static_cast<uint8_t>(BQ27742_BH_VOLT_TIME)),
    bh_clear_volt_threshold  : ensureEndianness(static_cast<int16_t>(BQ27742_BH_CLEAR_VOLT_THRESHOLD))
  };

  BQ_STRUCT{
    uint16_t pack_lot_code;     // 0
    uint16_t pcb_lot_code;      // 2
    uint16_t firmware_version;  // 4
    uint16_t hardware_revision; // 6
    uint16_t cell_revision;     // 8
    uint16_t df_config_version; // 10
  }manu_data = {
    pack_lot_code      : ensureEndianness(static_cast<uint16_t>(BQ27742_PACK_LOT_CODE)),
    pcb_lot_code       : ensureEndianness(static_cast<uint16_t>(BQ27742_PCB_LOT_CODE)),
    firmware_version   : ensureEndianness(static_cast<uint16_t>(BQ27742_FIRMWARE_VERSION)),
    hardware_revision  : ensureEndianness(static_cast<uint16_t>(BQ27742_HARDWARE_REVISION)),
    cell_revision      : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL_REVISION)),
    df_config_version  : ensureEndianness(static_cast<uint16_t>(BQ27742_DF_CONFIG_VERSION))
  };

  BQ_STRUCT{
    uint16_t all_df_checksum;         // 6
    uint16_t static_chem_df_checksum; // 8
    uint16_t static_df_checksum;      // 10
    uint16_t prot_checksum;           // 12
  }integrity_data = {
    all_df_checksum          : ensureEndianness(static_cast<uint16_t>(BQ27742_ALL_DF_CHECKSUM)),
    static_chem_df_checksum  : ensureEndianness(static_cast<uint16_t>(BQ27742_STATIC_CHEM_DF_CHECKSUM&0xFFFF)),
    static_df_checksum       : ensureEndianness(static_cast<uint16_t>(BQ27742_STATIC_DF_CHECKSUM)),
    prot_checksum            : ensureEndianness(static_cast<uint16_t>(BQ27742_PROT_CHECKSUM))
  };

  BQ_STRUCT{
    int16_t max_temp;         // 0
    int16_t min_temp;         // 2
    int16_t max_pack_voltage; // 4
    int16_t min_pack_voltage; // 6
    int16_t max_chg_current;  // 8
    int16_t max_dsg_current;  // 10
  }lifetime_data = {
    max_temp          : ensureEndianness(static_cast<int16_t>(BQ27742_LIFETIME_MAX_TEMP)),
    min_temp          : ensureEndianness(static_cast<int16_t>(BQ27742_LIFETIME_MIN_TEMP)),
    max_pack_voltage  : ensureEndianness(static_cast<int16_t>(BQ27742_LIFETIME_MAX_PACK_VOLTAGE)),
    min_pack_voltage  : ensureEndianness(static_cast<int16_t>(BQ27742_LIFETIME_MIN_PACK_VOLTAGE)),
    max_chg_current   : ensureEndianness(static_cast<int16_t>(BQ27742_LIFETIME_MAX_CHG_CURRENT)),
    max_dsg_current   : ensureEndianness(static_cast<int16_t>(BQ27742_LIFETIME_MAX_DSG_CURRENT))
  };

  BQ_STRUCT{
    uint16_t lt_flash_cnt;  // 0
    uint8_t  lt_afe_status; // 2
  }lifetime_temp_samples = {
    lt_flash_cnt   : ensureEndianness(static_cast<uint16_t>(BQ27742_LT_FLASH_CNT)),
    lt_afe_status  : ensureEndianness(BQ27742_LT_AFE_STATUS)
  };

  BQ_STRUCT{
    uint16_t pack_configuration;  // 0
    uint8_t  pack_configuration_b;// 2
    uint8_t  pack_configuration_c;// 3
    uint8_t  pack_configuration_d;// 4
    uint8_t  prot_oc_config;      // 5
    uint8_t  prot_ov_config;      // 6
  }registers = {
    pack_configuration    : ensureEndianness(static_cast<uint16_t>(BQ27742_PACK_CONFIGURATION)),
    pack_configuration_b  : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_CONFIGURATION_B)),
    pack_configuration_c  : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_CONFIGURATION_C)),
    pack_configuration_d  : ensureEndianness(static_cast<uint8_t>(BQ27742_PACK_CONFIGURATION_D)),
    prot_oc_config        : ensureEndianness(static_cast<uint8_t>(BQ27742_PROT_OC_CONFIG)),
    prot_ov_config        : ensureEndianness(static_cast<uint8_t>(BQ27742_PROT_OV_CONFIG))
  };

  BQ_STRUCT{
    uint8_t lt_temp_res;    // 0
    uint8_t lt_v_res;       // 1
    uint8_t lt_cur_res;     // 2
    uint16_t lt_update_time;// 3
  }lifetime_res = {
    lt_temp_res             : ensureEndianness(static_cast<uint8_t>(BQ27742_LT_TEMP_RES)),
    lt_v_res                : ensureEndianness(static_cast<uint8_t>(BQ27742_LT_V_RES)),
    lt_cur_res              : ensureEndianness(static_cast<uint8_t>(BQ27742_LT_CUR_RES)),
    lt_update_time          : ensureEndianness(static_cast<uint16_t>(BQ27742_LT_UPDATE_TIME)),
  };

  BQ_STRUCT{
    int16_t flash_update_ok_voltage;// 0
    int16_t sleep_current;          // 2
  }power = {
    flash_update_ok_voltage : ensureEndianness(static_cast<int16_t>(BQ27742_FLASH_UPDATE_OK_VOLTAGE)),
    sleep_current           : ensureEndianness(static_cast<int16_t>(BQ27742_SLEEP_CURRENT)),
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

  static const uint8_t  load_select                  = ensureEndianness(static_cast<uint8_t>(BQ27742_LOAD_SELECT));
  static const uint8_t  load_mode                    = ensureEndianness(static_cast<uint8_t>(BQ27742_LOAD_MODE));
  static const uint8_t  max_res_factor               = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_RES_FACTOR));
  static const uint8_t  min_res_factor               = ensureEndianness(static_cast<uint8_t>(BQ27742_MIN_RES_FACTOR));
  static const uint16_t ra_filter                    = ensureEndianness(static_cast<uint16_t>(BQ27742_RA_FILTER));
  static const int16_t  res_v_drop                   = ensureEndianness(static_cast<int16_t>(BQ27742_RES_V_DROP));
  static const uint8_t  fast_qmax_start_dod_procent  = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_QMAX_START_DOD_PROCENT));
  static const uint8_t  fast_qmax_end_dod_procent    = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_QMAX_END_DOD_PROCENT));
  static const int16_t  fast_qmax_star_volt_delta    = ensureEndianness(static_cast<int16_t>(BQ27742_FAST_QMAX_STAR_VOLT_DELTA));
  static const uint16_t fast_qmax_current_threshold  = ensureEndianness(static_cast<uint16_t>(BQ27742_FAST_QMAX_CURRENT_THRESHOLD));
  static const uint8_t  qmax_capacity_err            = ensureEndianness(static_cast<uint8_t>(BQ27742_QMAX_CAPACITY_ERR));
  static const uint8_t  max_qmax_change              = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_QMAX_CHANGE));
  static const int16_t  terminate_voltage            = ensureEndianness(static_cast<int16_t>(BQ27742_TERMINATE_VOLTAGE));
  static const int16_t  term_v_delta                 = ensureEndianness(static_cast<int16_t>(BQ27742_TERM_V_DELTA));
  static const uint16_t resrelax_time                = ensureEndianness(static_cast<uint16_t>(BQ27742_RESRELAX_TIME));
  static const int16_t  user_rate_ma                 = ensureEndianness(static_cast<int16_t>(BQ27742_USER_RATE_MA));
  static const int16_t  user_rate_pwr                = ensureEndianness(static_cast<int16_t>(BQ27742_USER_RATE_PWR));
  static const int16_t  reserve_cap_mah              = ensureEndianness(static_cast<int16_t>(BQ27742_RESERVE_CAP_MAH));
  static const int16_t  max_deltav                   = ensureEndianness(static_cast<int16_t>(BQ27742_MAX_DELTAV));
  static const int16_t  min_deltav                   = ensureEndianness(static_cast<int16_t>(BQ27742_MIN_DELTAV));
  static const uint8_t  max_sim_rate                 = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_SIM_RATE));
  static const uint8_t  min_sim_rate                 = ensureEndianness(static_cast<uint8_t>(BQ27742_MIN_SIM_RATE));
  static const int16_t  ra_max_delta                 = ensureEndianness(static_cast<int16_t>(BQ27742_RA_MAX_DELTA));
  static const int16_t  trace_resistance             = ensureEndianness(static_cast<int16_t>(BQ27742_TRACE_RESISTANCE));
  static const int16_t  downstream_resistance        = ensureEndianness(static_cast<int16_t>(BQ27742_DOWNSTREAM_RESISTANCE));
  static const uint8_t  qmax_max_delta_procent       = ensureEndianness(static_cast<uint8_t>(BQ27742_QMAX_MAX_DELTA_PROCENT));
  static const uint8_t  qmax_bound_procent           = ensureEndianness(static_cast<uint8_t>(BQ27742_QMAX_BOUND_PROCENT));
  static const uint16_t deltav_max_delta             = ensureEndianness(static_cast<uint16_t>(BQ27742_DELTAV_MAX_DELTA));
  static const uint16_t max_res_scale                = ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_RES_SCALE));
  static const uint16_t min_res_scale                = ensureEndianness(static_cast<uint16_t>(BQ27742_MIN_RES_SCALE));
  static const uint8_t  fast_scale_start_soc         = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_SCALE_START_SOC));
  static const uint8_t  fast_scale_load_select       = ensureEndianness(static_cast<uint8_t>(BQ27742_FAST_SCALE_LOAD_SELECT));
  static const int16_t  charge_hys_v_shift           = ensureEndianness(static_cast<int16_t>(BQ27742_CHARGE_HYS_V_SHIFT));
  static const uint8_t  rascl_ocv_rst_temp_thresh    = ensureEndianness(static_cast<uint8_t>(BQ27742_RASCL_OCV_RST_TEMP_THRESH));
  static const uint16_t max_allowed_current          = ensureEndianness(static_cast<uint16_t>(BQ27742_MAX_ALLOWED_CURRENT));
  static const uint8_t  max_current_pulse_duration   = ensureEndianness(static_cast<uint8_t>(BQ27742_MAX_CURRENT_PULSE_DURATION));
  static const int16_t  max_current_interrupt_step   = ensureEndianness(static_cast<int16_t>(BQ27742_MAX_CURRENT_INTERRUPT_STEP));
  static const uint16_t relax_smooth_time            = ensureEndianness(static_cast<uint16_t>(BQ27742_RELAX_SMOOTH_TIME));

  BQ_STRUCT{
    int16_t dsg_current_threshold;// 0
    int16_t chg_current_threshold;// 2
    int16_t quit_current;         // 4
    uint16_t dsg_relax_time;      // 6
    uint8_t chg_relax_time;       // 8
    uint8_t quit_relax_time;      // 9
    int16_t max_ir_correct;       // 10
  }current_thresholds = {
    dsg_current_threshold : ensureEndianness(static_cast<int16_t>(BQ27742_DSG_CURRENT_THRESHOLD)),
    chg_current_threshold : ensureEndianness(static_cast<int16_t>(BQ27742_CHG_CURRENT_THRESHOLD)),
    quit_current          : ensureEndianness(static_cast<int16_t>(BQ27742_QUIT_CURRENT)),
    dsg_relax_time        : ensureEndianness(static_cast<uint16_t>(BQ27742_DSG_RELAX_TIME)),
    chg_relax_time        : ensureEndianness(static_cast<uint8_t>(BQ27742_CHG_RELAX_TIME)),
    quit_relax_time       : ensureEndianness(static_cast<uint8_t>(BQ27742_QUIT_RELAX_TIME)),
    max_ir_correct        : ensureEndianness(static_cast<int16_t>(BQ27742_MAX_IR_CORRECT))
  };

  BQ_STRUCT{
    int16_t qmax_cell_0;
    uint8_t  update_status;
    int16_t v_at_chg_term;
    int16_t avg_i_last_run;
    int16_t avg_p_last_run;
    int16_t delta_voltage;
    int16_t t_rise;
    int16_t t_time_constant;
  }state = {
    qmax_cell_0      : ensureEndianness(static_cast<int16_t>(BQ27742_QMAX_CELL_0)),
    update_status    : ensureEndianness(static_cast<uint8_t>(BQ27742_UPDATE_STATUS)),
    v_at_chg_term    : ensureEndianness(static_cast<int16_t>(BQ27742_V_AT_CHG_TERM)),
    avg_i_last_run   : ensureEndianness(static_cast<int16_t>(BQ27742_AVG_I_LAST_RUN)),
    avg_p_last_run   : ensureEndianness(static_cast<int16_t>(BQ27742_AVG_P_LAST_RUN)),
    delta_voltage    : ensureEndianness(static_cast<int16_t>(BQ27742_DELTA_VOLTAGE)),
    t_rise           : ensureEndianness(static_cast<int16_t>(BQ27742_T_RISE)),
    t_time_constant  : ensureEndianness(static_cast<int16_t>(BQ27742_T_TIME_CONSTANT))
  };

  static const uint16_t chem_id = ensureEndianness(static_cast<uint16_t>(BQ27742_CHEM_ID));

  BQ_STRUCT{
    uint16_t cell0_r_a_flag;
    int16_t cell0_r_a_0;
    int16_t cell0_r_a_1;
    int16_t cell0_r_a_2;
    int16_t cell0_r_a_3;
    int16_t cell0_r_a_4;
    int16_t cell0_r_a_5;
    int16_t cell0_r_a_6;
    int16_t cell0_r_a_7;
    int16_t cell0_r_a_8;
    int16_t cell0_r_a_9;
    int16_t cell0_r_a_10;
    int16_t cell0_r_a_11;
    int16_t cell0_r_a_12;
    int16_t cell0_r_a_13;
    int16_t cell0_r_a_14;
  }r_a0 = {
    cell0_r_a_flag  : ensureEndianness(static_cast<uint16_t>(BQ27742_CELL0_R_A_FLAG)),
    cell0_r_a_0     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_0)),
    cell0_r_a_1     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_1)),
    cell0_r_a_2     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_2)),
    cell0_r_a_3     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_3)),
    cell0_r_a_4     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_4)),
    cell0_r_a_5     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_5)),
    cell0_r_a_6     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_6)),
    cell0_r_a_7     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_7)),
    cell0_r_a_8     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_8)),
    cell0_r_a_9     : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_9)),
    cell0_r_a_10    : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_10)),
    cell0_r_a_11    : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_11)),
    cell0_r_a_12    : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_12)),
    cell0_r_a_13    : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_13)),
    cell0_r_a_14    : ensureEndianness(static_cast<int16_t>(BQ27742_CELL0_R_A_14))
  };

  BQ_STRUCT{
    uint16_t xcell0_r_a_flag;
    int16_t xcell0_r_a_0;
    int16_t xcell0_r_a_1;
    int16_t xcell0_r_a_2;
    int16_t xcell0_r_a_3;
    int16_t xcell0_r_a_4;
    int16_t xcell0_r_a_5;
    int16_t xcell0_r_a_6;
    int16_t xcell0_r_a_7;
    int16_t xcell0_r_a_8;
    int16_t xcell0_r_a_9;
    int16_t xcell0_r_a_10;
    int16_t xcell0_r_a_11;
    int16_t xcell0_r_a_12;
    int16_t xcell0_r_a_13;
    int16_t xcell0_r_a_14;
  }r_a0x = {
    xcell0_r_a_flag  : ensureEndianness(static_cast<uint16_t>(BQ27742_XCELL0_R_A_FLAG)),
    xcell0_r_a_0     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_0)),
    xcell0_r_a_1     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_1)),
    xcell0_r_a_2     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_2)),
    xcell0_r_a_3     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_3)),
    xcell0_r_a_4     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_4)),
    xcell0_r_a_5     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_5)),
    xcell0_r_a_6     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_6)),
    xcell0_r_a_7     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_7)),
    xcell0_r_a_8     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_8)),
    xcell0_r_a_9     : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_9)),
    xcell0_r_a_10    : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_10)),
    xcell0_r_a_11    : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_11)),
    xcell0_r_a_12    : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_12)),
    xcell0_r_a_13    : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_13)),
    xcell0_r_a_14    : ensureEndianness(static_cast<int16_t>(BQ27742_XCELL0_R_A_14))
  };

  BQ_STRUCT {
    uint32_t cc_gain;
    uint32_t cc_delta;
    int16_t cc_offset;
    int8_t board_offset;
    int8_t int_temp_offset;
    int8_t ext_temp_offset;
    int8_t pack_v_offset;
  }calibration = {
    cc_gain         : float_to_bq_format(BQ27742_CC_GAIN),
    cc_delta        : float_to_bq_format(BQ27742_CC_DELTA),
    cc_offset       : ensureEndianness(static_cast<int16_t>(BQ27742_CC_OFFSET_)),
    board_offset    : ensureEndianness(static_cast<int8_t>(BQ27742_BOARD_OFFSET_)),
    int_temp_offset : ensureEndianness(static_cast<int8_t>(BQ27742_INT_TEMP_OFFSET)),
    ext_temp_offset : ensureEndianness(static_cast<int8_t>(BQ27742_EXT_TEMP_OFFSET)),
    pack_v_offset   : ensureEndianness(static_cast<int8_t>(BQ27742_PACK_V_OFFSET))
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

static void lightRED(){
  ic_actuator_set_on_func(IC_LEFT_RED_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_GREEN_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_BLUE_LED, 0, 0, 63);
}

static void lightGREEN(){
  ic_actuator_set_on_func(IC_LEFT_GREEN_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_RED_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_BLUE_LED, 0, 0, 63);
}

static void lightBLUE(){
  ic_actuator_set_on_func(IC_LEFT_BLUE_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_GREEN_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_RED_LED, 0, 0, 63);
}

static uint8_t ledFuncPtr = 0;
static void(*ledFuncArray[])(void) = {lightRED, lightGREEN, lightBLUE};

static void lightOFF(){
  ic_actuator_set_off_func(IC_LEFT_BLUE_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_GREEN_LED, 0, 0, 63);
  ic_actuator_set_off_func(IC_LEFT_RED_LED, 0, 0, 63);
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
      sizeof(T)
      );
  ledFuncArray[ledFuncPtr]();
  if(++ledFuncPtr>=sizeof(ledFuncArray)/sizeof(ledFuncArray[0])){
    ledFuncPtr = 0;
  }
  vTaskDelay(pdMS_TO_TICKS(15));
}

/**
 * @fn bq27742_read_control_data (uint16_t subcommand)
 * @brief read data from  control register
 *  param[in] subcommand: data subcommand code
 *  param[out] read control data
 */
static uint16_t bq27742_read_control_data (uint16_t subcommand)
{
  uint16_t return_value =0;

  set_bq_register(BQ27742_CONTROL, subcommand);
  vTaskDelay(pdMS_TO_TICKS(1));

  #ifndef DEBUG_BQ
  TWI_READ_DATA(BQ, BQ27742_CONTROL, (uint8_t *)&return_value, sizeof(return_value), NULL, NULL);
  #endif

  return return_value;
}

/**
 * @fn bq27742_control_status_read ()
 * @brief read data from control status register
 *  param[out] register value
 */
static uint16_t bq27742_control_status_read(void)
{
  return bq27742_read_control_data(BQ27742_CONTROL_STATUS);
}

static bool bq27742_unsealed_set(void)
{
  uint32_t _unsealed = 0x36720414;

  set_bq_register(BQ27742_CONTROL, reinterpret_cast<uint8_t *>(&_unsealed), 2);
  vTaskDelay(pdMS_TO_TICKS(1));
  set_bq_register(BQ27742_CONTROL, &reinterpret_cast<uint8_t *>(&_unsealed)[2], 2);
  vTaskDelay(pdMS_TO_TICKS(1));
  auto _controlStatus = bq27742_control_status_read();
  return (!(_controlStatus & (1<<BQ_SEALED_MODE))) ? true : false;
}

static bool bq27742_sealed_set ()
{
  uint16_t _seal = BQ27742_SEALED;
  set_bq_register(BQ27742_CONTROL, _seal);
  vTaskDelay(pdMS_TO_TICKS(1));

  auto _controlStatus = bq27742_control_status_read();
  NRF_LOG_INFO("Control Status(sealed): 0x%X\n", _controlStatus);
  return (_controlStatus & ((1<<BQ_SEALED_MODE)|(1<<BQ_FULL_SEALED_MODE))) ? true : false;
}

/**
 * @fn  bq27742_full_access_set()
 * @brief put device into full access state
 *  param[out] return true if device enter full access state
*/
static bool bq27742_full_access_set(void)
{
  uint32_t _fullAccess = 0xFFFFFFFF;

  set_bq_register(BQ27742_CONTROL, reinterpret_cast<uint8_t *>(&_fullAccess), 2);
  vTaskDelay(pdMS_TO_TICKS(1));
  set_bq_register(BQ27742_CONTROL, &reinterpret_cast<uint8_t *>(&_fullAccess)[2], 2);
  vTaskDelay(pdMS_TO_TICKS(1));

  auto _controlStatus = bq27742_control_status_read();
  return (!(_controlStatus & (1<<BQ_FULL_SEALED_MODE))) ? true : false;
}


void bq27742_program_flash_subclass_if_cfg1(void){
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
  bq27742_program_flash(bqSubclassData::relax_smooth_time,           BQ27742_IT_CFG_CLASS_ID, 0x03, 0x12);
}

#include "ic_service_ltc.h"


static uint16_t bq27742_device_type_read(){
  uint16_t devType = bq27742_read_control_data(BQ27742_DEVICE_TYPE);
  NRF_LOG_INFO("Output: 0x%X vs static: 0x%X\n", devType,
      0xFFFF&BQ27742_DEVICE_TYPE_VALUE);
  return 0x00;
}

#define BQ_ADVANCED

void ic_bq_flash_image(){
  NRF_LOG_INFO("Control Status: 0x%X\n", bq27742_control_status_read());
  auto _unsealed = bq27742_unsealed_set();
  if(!_unsealed){
    NRF_LOG_INFO("could not unseal!\n");
    return;
  }
  auto _fullAccess = bq27742_full_access_set();
  if(!_fullAccess){
    NRF_LOG_INFO("no full access\n");
    return;
  }
  lightGREEN();
  NRF_LOG_INFO("Control Status: 0x%X\n", bq27742_control_status_read());
  NRF_LOG_INFO("Unsealed: %s, FullAccess: %s\n",
      (uint32_t)(_unsealed?"true":"false"),
      (uint32_t)(_fullAccess?"true":"false"));
  vTaskDelay(pdMS_TO_TICKS(10));

  bq27742_device_type_read();

  NRF_LOG_RAW_INFO("---=== SAFETY ===---\n");
  bq27742_program_flash(bqSubclassData::safety, BQ27742_SAFETY_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== CHARGE ===---\n");
  bq27742_program_flash(bqSubclassData::charge, BQ27742_CHARGE_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== CHARGE TERMINATION ===---\n");
  bq27742_program_flash(bqSubclassData::charge_termination, BQ27742_CHARGE_TERMINATION_CLASS_ID);

  #ifdef BQ_ADVANCED
  NRF_LOG_RAW_INFO("---=== JEITA ===---\n");
  bq27742_program_flash(bqSubclassData::jeita, BQ27742_JEITA_CLASS_ID);
  #endif

  NRF_LOG_RAW_INFO("---=== DESIGN VOLTAGE ===---\n");
  bq27742_program_flash(bqSubclassData::design_voltage, BQ27742_DATA_CLASS_ID);
  uint8_t zz = 0x00;
  bq27742_program_flash(zz, BQ27742_DATA_CLASS_ID,0x00,0x02);
  NRF_LOG_RAW_INFO("---=== DATA ===---\n");
  bq27742_program_flash(bqSubclassData::data, BQ27742_DATA_CLASS_ID, 0x00, 0x08);
  NRF_LOG_RAW_INFO("---=== DISCHARGE ===---\n");
  bq27742_program_flash(bqSubclassData::discharge, BQ27742_DISCHARGE_CLASS_ID);

  #ifdef BQ_ADVANCED
  NRF_LOG_RAW_INFO("---=== MANU DATA ===---\n");
  bq27742_program_flash(bqSubclassData::manu_data, BQ27742_MANUFACTURER_DATA_CLASS_ID);
  #endif

  NRF_LOG_RAW_INFO("---=== INTEGRITY DATA ===---\n");
  bq27742_program_flash(bqSubclassData::integrity_data, BQ27742_INTEGRITY_CLASS_ID, 0x00, 0x06);
  NRF_LOG_RAW_INFO("---=== LIFETIME DATA ===---\n");
  bq27742_program_flash(bqSubclassData::lifetime_data, BQ27742_LIFETIME_DATA_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== LIFETIME TEMP SAMPLES ===---\n");
  bq27742_program_flash(bqSubclassData::lifetime_temp_samples, BQ27742_LIFETIME_TEMP_SAMPLES_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== REGISTERS ===---\n");
  bq27742_program_flash(bqSubclassData::registers, BQ27742_REGISTERS_CLASS_ID);



  #ifdef BQ_ADVANCED
  NRF_LOG_RAW_INFO("---=== LIFETIME RES ===---\n");
  bq27742_program_flash(bqSubclassData::lifetime_res, BQ27742_LIFETIME_RES_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== POWER ===---\n");
  bq27742_program_flash(bqSubclassData::power, BQ27742_POWER_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== SHUTDOWN V ===---\n");
  bq27742_program_flash(bqSubclassData::shutdown_v, BQ27742_POWER_CLASS_ID, 0x00, 0x0A);
  NRF_LOG_RAW_INFO("---=== FS WAIT ===---\n");
  bq27742_program_flash(bqSubclassData::fs_wait, BQ27742_POWER_CLASS_ID, 0x00, 0x0C);
  NRF_LOG_RAW_INFO("---=== MANUFACTURER INFO1 ===---\n");
  bq27742_program_flash(bqSubclassData::manufacturer_info1, BQ27742_MANUFACTURER_INFO_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== MANUFACTURER INFO2 ===---\n");
  bq27742_program_flash(bqSubclassData::manufacturer_info2, BQ27742_MANUFACTURER_INFO_CLASS_ID, 0x01, 0x00);
  NRF_LOG_RAW_INFO("---=== IF CFG1 ===---\n");
  bq27742_program_flash_subclass_if_cfg1();
  NRF_LOG_RAW_INFO("---=== CURRENT TRESHOLD ===---\n");
  bq27742_program_flash(bqSubclassData::current_thresholds, BQ27742_CURRENT_TRESHOLD_CLASS_ID);
  #endif

  NRF_LOG_RAW_INFO("---=== STATE ===---\n");
  bq27742_program_flash(bqSubclassData::state, BQ27742_STATE_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== CHEM ID ===---\n");
  bq27742_program_flash(bqSubclassData::chem_id, BQ27742_CHEM_ID_CLASS_ID);
  zz = 0x10;
  bq27742_program_flash(zz, BQ27742_CHEM_ID_CLASS_ID,0x00,0x02);
  NRF_LOG_RAW_INFO("---=== R A0 ===---\n");
  bq27742_program_flash(bqSubclassData::r_a0, BQ27742_R_A0_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== R A0X ===---\n");
  bq27742_program_flash(bqSubclassData::r_a0x, BQ27742_R_A0X_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== CALIBRATION ===---\n");
  bq27742_program_flash(bqSubclassData::calibration, BQ27742_CALIBRATION_CLASS_ID);

#if 0
  #ifdef BQ_ADVANCED
  NRF_LOG_RAW_INFO("---=== CURRENT ===---\n");
  bq27742_program_flash(bqSubclassData::current, BQ27742_CURRENT_CLASS_ID);
  NRF_LOG_RAW_INFO("---=== CODES ===---\n");
  bq27742_program_flash(bqSubclassData::codes, BQ27742_CODES_CLASS_ID);
  #endif

#endif

  auto _sealed = bq27742_sealed_set();
  lightOFF();
  NRF_LOG_INFO("Sealed: %s\n",
      (uint32_t)(_sealed?"true":"false"));

  vTaskDelay(5);
}

/**
 * @fn bq27742_read_reg_data  (uint8_t command)
 * @brief read data from  general register
 *  param[in] command: data command code
 *  param[out] read register data
 */
static uint16_t bq27742_read_reg_data (uint8_t command)
{
  uint16_t return_value =0;

  get_bq_register (command, return_value);

  return return_value;
}

static ic_return_val_e bq27742_read_reg_data (uint8_t command, void (*cb)(uint16_t)){
  return get_bq_register (command, cb);
}


/**
 * @fn bq27742_flags_read ()
 * @brief read data from flags register
 *  param[out] register value
 */
static uint16_t bq27742_flags_read ()
{
  uint16_t flag_value =0;

  flag_value = bq27742_read_reg_data(BQ27742_FLAGS);

  return flag_value;
}

static ic_return_val_e bq27742_flags_read (void (*cb)(uint16_t)){
  return bq27742_read_reg_data(BQ27742_FLAGS, cb);
}



void ic_bq_reset(){
  for (uint16_t i = 0; i<0x09; ++i){
    auto _rc = bq27742_read_control_data(i);
    NRF_LOG_INFO("0x%04X: 0x%X\n",i, _rc);
  }
  auto _unsealed = bq27742_unsealed_set();
  NRF_LOG_INFO("Unsealed: %s\n",
      (uint32_t)(_unsealed?"true":"false"));
  uint16_t _chem_id = swapBytes(0x185);
  bq27742_program_flash(_chem_id, BQ27742_CHEM_ID_CLASS_ID);
  auto _sealed = bq27742_sealed_set();
  NRF_LOG_INFO("Sealed: %s\n",
      (uint32_t)(_sealed?"true":"false"));
}

uint16_t ic_bq_getSafetyStatus(void) {

  uint16_t safety = bq27742_read_control_data(BQ27742_SAFETY_STATUS);
//  NRF_LOG_INFO("** Safety status: 0x%02X invalid protector: %d\n", safety , safety&0x80);

  return safety;
}


static bool ic_bq_battery_full_charged (void)
{
  uint16_t flag_value = 0;

  flag_value = bq27742_flags_read ();

  if(flag_value & (1<<FC))
    return true;
  else
    return false;
}

static ic_return_val_e ic_bq_battery_full_charged (void (*cb)(bool))
{
  static uint16_t _flag_value;

  auto _lamda = [&_flag_value](uint16_t flag_value){
    cb(flag_value & (1<<FC);
  };

  return bq27742_flags_read (_lamda);
}


///**
// * Check is State of charge is below 1%
// * @return true/false
// */
//bool bq27742_battery_soc1_threshold (void)
//{
//  uint16_t flag_value = 0;
//
//  flag_value = bq27742_flags_read ();
//  if(flag_value & (1<<SOC1))
//    return true;
//  else
//    return false;
//}
//
///**
// * Check is State of charge is 100%
// * @return true/false
// */
//bool bq27742_battery_socf_threshold (void)
//{
//  uint16_t flag_value = 0;
//
//  flag_value = bq27742_flags_read ();
//  if(flag_value & (1<<SOCF))
//    return true;
//  else
//    return false;
//}

///**
// * One of discharging errors detected
// * @return true/false
// */
//bool bq27742_discharging_error_detected (void)
//{
//  uint16_t safety_status = 0;
//  uint8_t charging_error_mask = 1 << UVP_S | 1 << OTD_S | 1 << TDD_S | 1 << ISD_S;
//  safety_status = ic_bq_getSafetyStatus();
//  if(safety_status & charging_error_mask)
//    return true;
//  else
//    return false;
//}

/**
 * One of charging errors detected
 * @return true/false
 */
static bool bq27742_charging_error_detected (void)
{
  uint16_t safety_status = 0;
  uint8_t charging_error_mask = 1 << OVP_S | 1 << OTC_S | 1 << TDD_S | 1 << ISD_S;
  safety_status = ic_bq_getSafetyStatus();
  if(safety_status & charging_error_mask)
    return true;
  else
    return false;
}

static bool bq27742_charging_error_detected (void (*cb)(bool)){
  uint16_t safety_status = 0;
  uint8_t charging_error_mask = 1 << OVP_S | 1 << OTC_S | 1 << TDD_S | 1 << ISD_S;
  safety_status = ic_bq_getSafetyStatus();
  if(safety_status & charging_error_mask)
    return true;
  else
    return false;
}



void ic_bq_read_measurement_data (void)
{
  uint16_t temp;
  get_bq_register(BQ27742_TEMPERATURE, temp);
  vTaskDelay(100);
  uint16_t voltage;
  get_bq_register(BQ27742_VOLTAGE, voltage);
  vTaskDelay(100);
  int16_t avgCurr;
  get_bq_register(BQ27742_AVERAGE_CURRENT, avgCurr);
  vTaskDelay(100);
  uint16_t soc;
  get_bq_register(BQ27742_STATE_OF_CHARGE, soc);
  vTaskDelay(100);

  NRF_LOG_RAW_INFO("temperature: %d\n", temp );
  NRF_LOG_RAW_INFO("voltage: %dmV\n", voltage);
  NRF_LOG_RAW_INFO("avg current: %dmA\n", avgCurr);
  NRF_LOG_RAW_INFO("state of charge: %d%%\n", soc);
}

ic_return_val_e ic_bq_init(void){
  if (m_module_initialized == false)
    return IC_SUCCESS;
  TWI_INIT(BQ);
  m_module_initialized = true;

  return IC_SUCCESS;
}

ic_return_val_e ic_bq_deinit(void){
  if(m_module_initialized == false)
    return IC_NOT_INIALIZED;
  TWI_DEINIT(BQ);
  m_module_initialized = false;

  return IC_SUCCESS;
}

uint16_t ic_bq_getChargeLevel(void (*cb)(uint16_t))
{
  return get_bq_register(BQ27742_STATE_OF_CHARGE, cb);
//  NRF_LOG_RAW_INFO("state of charge: %d\n", soc);
}


/**
 * Return State of charge
 * @return percent value
 */
bool ic_bq_getDischarging(void) {
  uint16_t flag_value = 0;

  flag_value = bq27742_flags_read();
  if (flag_value & (1 << DSG))
    return true;
  else
    return false;
}

static void (*m_user_charger_state_cb)(en_chargerState state);

static void avg_current_cb(int16_t avg_curr){
  if(avg_curr < 0)
    m_user_charger_state_cb(BATT_NOTCHARGING);
  else
    m_user_charger_state_cb(BATT_CHARGING);
}

static void battery_charged_cb(bool charged){
  if(charged)
    m_user_charger_state_cb(BATT_CHARGED);
  else
    get_bq_register(BQ27742_AVERAGE_CURRENT, avgCurr);
}

ic_return_val_e ic_bq_getChargerState(void (*cb)(en_chargerState)){
  if(cb == NULL)
    return IC_ERROR;

  m_user_charger_state_cb = cb;

  return ic_bq_battery_full_charged(battery_charged_cb)
}


/**
 * Read data from data flash
 * @param subclass_id - data block to read
 * @param offset - data offset to read
 * @param value - return buffer
 * @param len - length of data to read
 */
void bq27742_data_flash_read(
    uint8_t subclass_id,
    uint8_t offset,
    uint8_t *value,
    size_t len,
    uint8_t block = 0)
{
  NRF_LOG_RAW_INFO("Read of class id(%d): 0x%02x (block=%d)\n",subclass_id, subclass_id,block);
  for(int i = 0;i<32;i++)
    value[i] = 0;
  set_bq_register(BQ27742_BLOCK_DATA_CONTROL, (uint8_t)0x00);
  vTaskDelay(100);
  set_bq_register(BQ27742_DATA_FLASH_CLASS, (uint8_t)subclass_id);
  vTaskDelay(100);
  set_bq_register(BQ27742_DATA_FLASH_BLOCK, (uint8_t)block);
  vTaskDelay(100);

  TWI_READ_DATA(BQ, BQ27742_BLOCK_DATA, (uint8_t*)value, len, NULL, NULL);
  vTaskDelay(pdMS_TO_TICKS(1));

  uint8_t memChck[1] = {0};
  get_bq_register(BQ27742_BLOCK_DATA_CS,memChck);
  vTaskDelay(100);

  uint32_t _flash_data_sum = 0;
  for (uint8_t i = 0; i < len; i++){
    NRF_LOG_RAW_INFO("0x%02x ",value[i]);
    _flash_data_sum += value[i];
  }
  uint8_t _checksum = 255 - (uint8_t)(_flash_data_sum);
  NRF_LOG_RAW_INFO("chckCalc0x%02X memChck0x%02X",_checksum, memChck[0]);
  NRF_LOG_RAW_INFO("\n")
}

/**
 * Read and print on console all data from data flash
 */
void ic_bq_readFlashTest(void){
//  NRF_LOG_INFO("Control Status: 0x%X\n", bq27742_control_status_read());
  auto _unsealed = bq27742_unsealed_set();
  if(!_unsealed){
    NRF_LOG_INFO("could not unseal!\n");
    return;
  }
  auto _fullAccess = bq27742_full_access_set();
  if(!_fullAccess){
    NRF_LOG_INFO("no full access\n");
    return;
  }
  uint8_t data[32];
  for(int i = 0;i<32;i++)
    data[i] = 0;
  NRF_LOG_RAW_INFO("BQ27742_SAFETY_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_SAFETY_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_CHARGE_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_CHARGE_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_CHARGE_TERMINATION_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_CHARGE_TERMINATION_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_DISCHARGE_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_DISCHARGE_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_CALIBRATION_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_CALIBRATION_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_JEITA_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_JEITA_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_DATA_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_DATA_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_MANUFACTURER_DATA_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_MANUFACTURER_DATA_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_INTEGRITY_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_INTEGRITY_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_LIFETIME_DATA_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_LIFETIME_DATA_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_LIFETIME_TEMP_SAMPLES_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_LIFETIME_TEMP_SAMPLES_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_REGISTERS_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_REGISTERS_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_LIFETIME_RES_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_LIFETIME_RES_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_POWER_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_POWER_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_MANUFACTURER_INFO_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_MANUFACTURER_INFO_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_CURRENT_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_CURRENT_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_STATE_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_STATE_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_CHEM_ID_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_CHEM_ID_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_R_A0_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_R_A0_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_R_A0X_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_R_A0X_CLASS_ID,0,data,32);
  NRF_LOG_RAW_INFO("BQ27742_CURRENT_TRESHOLD_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_CURRENT_TRESHOLD_CLASS_ID,0,data,32);

  NRF_LOG_RAW_INFO("BQ27742_IT_CFG_CLASS_ID\n");
  bq27742_data_flash_read(BQ27742_IT_CFG_CLASS_ID,0,data,32,0);
  bq27742_data_flash_read(BQ27742_IT_CFG_CLASS_ID,0,data,32,1);
  bq27742_data_flash_read(BQ27742_IT_CFG_CLASS_ID,0,data,32,2);
  bq27742_data_flash_read(BQ27742_IT_CFG_CLASS_ID,0,data,32,3);


  bq27742_sealed_set();
}

