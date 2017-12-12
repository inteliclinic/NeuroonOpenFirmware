/**
 * @file    ic_driver_lis3dh.c
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    October, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_lis3dh.h"
#include "ic_driver_twi.h"

#define NRF_LOG_MODULE_NAME "LIS3DH"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"
#include "ic_driver_button.h"


TWI_REGISTER(LIS3DH, LIS3DH_SLAVE_ADDR);

static uint8_t lis3dh_bufer[10];

  /*  function pointer for holding acc data and reseting wdt function  */
static volatile void (*m_fp)(acc_data_s) = NULL;
static volatile void (*m_fp_force)(acc_data_s) = NULL;

/**
 * @brief
 *
 * @param e
 * @param p_context
 */
static void acc_twi_callback(ic_return_val_e e, void *p_context){
  if(e != IC_SUCCESS){
    NRF_LOG_ERROR("{%s}\n",(uint32_t)__func__);
    return;
  }

  if (m_fp != NULL)
  {
    m_fp(*(acc_data_s *)&lis3dh_bufer[1]);
  }
  else
    NRF_LOG_INFO("ERROR with wdt timer\r\n");

  if(p_context != NULL)
    p_context = (acc_data_s *)&lis3dh_bufer[1];

  if((lis3dh_bufer[0] & 0b00001000)==0){
     __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, NULL);
     if(_ret_val == IC_SOFTWARE_BUSY)
       TWI_READ_DATA_FORCED(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, NULL);
    /*NRF_LOG_INFO("Read: %s\n",(uint32_t)g_return_val_string[_ret_val]);*/
  }
//  NRF_LOG_INFO("Going next...\r\n");
  if(m_fp_force != NULL){
    NRF_LOG_INFO("Status: 0x%X\n",lis3dh_bufer[0]);
    m_fp(*(acc_data_s *)&lis3dh_bufer[1]);
    m_fp_force = NULL;
  }

//  NRF_LOG_INFO("x: %d, y: %d, z: %d\n",
//			((acc_data_s *)&lis3dh_bufer[1])->x,
//			((acc_data_s *)&lis3dh_bufer[1])->y,
//			((acc_data_s *)&lis3dh_bufer[1])->z);

  NRF_LOG_FLUSH();
}

/**
 * @brief
 *
 * @param e
 * @param p_context
 */
static void acc_twi_read_callback(ic_return_val_e e, void *p_context){
  UNUSED_PARAMETER(e);
  if(m_fp_force != NULL){
//    NRF_LOG_INFO("Status: 0x%X\n",lis3dh_bufer[0]);
    m_fp_force(*(acc_data_s *)&lis3dh_bufer[1]);
    m_fp_force = NULL;
  }
}

static volatile bool m_lock = false;

#define LOCK    m_lock = true
#define UNLOCK  m_lock = false
#define LOCKED  (m_lock == true)

/**
 * @brief
 *
 * @param edge
 */
static void acc_int_callback(enum exti_edge_dir edge){
  ic_return_val_e _ret_val;
  switch(edge){
    case EXTI_EDGE_DOWN:
//      NRF_LOG_INFO("Edge down\n");
      break;
    case EXTI_EDGE_UP:
//      NRF_LOG_INFO("Edge up\n");
      _ret_val =
        TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, NULL);
      if(_ret_val != IC_SUCCESS){
        NRF_LOG_ERROR("TWI problem: %s\n", (uint32_t)g_return_val_string[_ret_val]);
        if(_ret_val == IC_SOFTWARE_BUSY)
          _ret_val = TWI_READ_DATA_FORCED(
              LIS3DH,
              LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG,
              lis3dh_bufer,
              7,
              acc_twi_callback,
  	      NULL);
      }
      break;
    default:
      break;
  }
}

ic_return_val_e ic_lis3dh_read_data(void(*fp)(acc_data_s data)){
  if (m_fp_force != NULL)
    return IC_BUSY;
  m_fp_force = fp;
  __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_read_callback, NULL);
  if (_ret_val != IC_SUCCESS)
  {
    ic_twi_refresh_bus();
    m_fp_force = NULL;
    return _ret_val;
  }
  return IC_SUCCESS;
}

static void m_config_reg(uint8_t reg, uint8_t reg_val){
  uint8_t _d[] = {reg, reg_val};
  __auto_type _ret_val = TWI_SEND_DATA(LIS3DH, _d, sizeof(_d), NULL, NULL);
  if(_ret_val == IC_SUCCESS){
    NRF_LOG_INFO("CTRL_REG 0x%X: 0x%X\n",_d[0], _d[1]);
  }
  else{
    NRF_LOG_INFO("CTRL_REG; some error\n");
  }
}

void m_reconfigure_reg_bit(uint8_t reg, uint8_t bits, bool set){
  uint8_t _reg_val_current;
  uint8_t _reg_val_target;

  __auto_type _ret_val = TWI_READ_DATA(LIS3DH, reg, &_reg_val_current, sizeof(_reg_val_current), NULL, NULL);

  if(_ret_val != IC_SUCCESS){
    NRF_LOG_ERROR("could not read data from reg: 0x%X\n", reg);
    return;
  }

  _reg_val_target = set ? _reg_val_current|bits:_reg_val_current&(~bits);

  if(_reg_val_target == _reg_val_current){
    NRF_LOG_INFO("no change in register 0x%X\n", reg);
    return;
  }
  NRF_LOG_INFO("Register 0x%X value: 0x%X. Changing to 0x%X\n",
      reg, _reg_val_current, _reg_val_target);

  m_config_reg(reg, _reg_val_target);
}

#define SET_REG_BIT(reg, bits) m_reconfigure_reg_bit(reg, bits, true)
#define CLR_REG_BIT(reg, bits) m_reconfigure_reg_bit(reg, bits, false)

#define DISABLE_DRDY_INT  CLR_REG_BIT(LIS3DH_REG_CTRL_REG3, LIS3DH_CTRL_REG3_I1_DRDY)
  /*  enable interrupt by setting I1_ZXDYA bit in LIS3DH_REG_CTRL_REG3 register  */
#define ENABLE_DRDY_INT   SET_REG_BIT(LIS3DH_REG_CTRL_REG3, LIS3DH_CTRL_REG3_I1_DRDY)

ic_return_val_e ic_lis3dh_init (void(*fp)(acc_data_s)){
  if (fp == NULL)
  {
    uint8_t _val = 0x11;

    TWI_INIT(LIS3DH);
    __auto_type ret_val  = TWI_READ_DATA(LIS3DH, LIS3DH_REG_WHO_AM_I, &_val, sizeof(_val), NULL, NULL);
    NRF_LOG_INFO("WHO AM I: 0x%X\t ret_val: %d\n", _val==LIS3DH_WHO_AM_I, ret_val);

    DISABLE_DRDY_INT;

    uint8_t _reg_val_combo[][2] =
    {
      {LIS3DH_REG_CTRL_REG1, LIS3DH_CTRL_REG1_25HZ_RATE|
       LIS3DH_CTRL_REG1_XEN|LIS3DH_CTRL_REG1_YEN|LIS3DH_CTRL_REG1_ZEN},
      {LIS3DH_REG_CTRL_REG2, 0x00},
      {LIS3DH_REG_CTRL_REG3, 0x00},
      {LIS3DH_REG_CTRL_REG4, 0x00},
      {LIS3DH_REG_CTRL_REG5, 0x00},
      {LIS3DH_REG_CTRL_REG6, 0x00}

    /*{LIS3DH_REG_CTRL_REG4, LIS3DH_CTRL_REG4_FS1|LIS3DH_CTRL_REG4_FS0|LIS3DH_CTRL_REG4_HR}*/
    };

    for(int i=0; i<sizeof(_reg_val_combo)/sizeof(_reg_val_combo[0]); ++i){
      m_config_reg(_reg_val_combo[i][0], _reg_val_combo[i][1]);
    }

  }
  else
  {
    ic_acc_exti_handle_init(acc_int_callback);
    TWI_INIT(LIS3DH);
    uint8_t _val = 0x11;

    m_fp = fp;
    NRF_LOG_INFO("Pointer address: %p\r\n", (uint32_t)m_fp);

    __auto_type ret_val  = TWI_READ_DATA(LIS3DH, LIS3DH_REG_WHO_AM_I, &_val, sizeof(_val), NULL, NULL);
    NRF_LOG_INFO("WHO AM I: 0x%X\t ret_val: %d\n", _val==LIS3DH_WHO_AM_I, ret_val);

    DISABLE_DRDY_INT;

    uint8_t _reg_val_combo[][2] =
    {
      {LIS3DH_REG_CTRL_REG1, LIS3DH_CTRL_REG1_25HZ_RATE|
       LIS3DH_CTRL_REG1_XEN|LIS3DH_CTRL_REG1_YEN|LIS3DH_CTRL_REG1_ZEN},
      {LIS3DH_REG_CTRL_REG2, 0x00},
      {LIS3DH_REG_CTRL_REG3, 0x00},
      {LIS3DH_REG_CTRL_REG4, 0x00},
      {LIS3DH_REG_CTRL_REG5, 0x00},
      {LIS3DH_REG_CTRL_REG6, 0x00}

      /*{LIS3DH_REG_CTRL_REG4, LIS3DH_CTRL_REG4_FS1|LIS3DH_CTRL_REG4_FS0|LIS3DH_CTRL_REG4_HR}*/
    };

    for(int i=0; i<sizeof(_reg_val_combo)/sizeof(_reg_val_combo[0]); ++i){
      m_config_reg(_reg_val_combo[i][0], _reg_val_combo[i][1]);
    }

    ENABLE_DRDY_INT;

    NRF_LOG_FLUSH();
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_lis3dh_uninit(void)
{
  DISABLE_DRDY_INT;
  TWI_DEINIT(LIS3DH);
    /*  set AccInt pin to high impedance mode  */
  nrf_gpio_cfg_default(IC_ACC_EXTI_PIN);

  return IC_SUCCESS;
}

ic_return_val_e ic_lis3dh_set_power_mode(acc_power_mode_e power_mode)
{
  uint8_t _val = 0x00;
  __auto_type ret_val  = TWI_READ_DATA(LIS3DH, LIS3DH_REG_CTRL_REG1, &_val, sizeof(_val), NULL, NULL);
  if(ret_val != IC_SUCCESS)
    NRF_LOG_ERROR("TWI problem: %s\n", (uint32_t)g_return_val_string[ret_val]);
  _val &= ~(0x0F);
  _val |= power_mode << 4;

  m_config_reg(LIS3DH_REG_CTRL_REG1, _val);

  return IC_SUCCESS;
}

ic_return_val_e ic_lis3dh_set_g_range(acc_g_range_e g_range)
{
  uint8_t _val = 0x00;
  __auto_type ret_val  = TWI_READ_DATA(LIS3DH, LIS3DH_REG_CTRL_REG4, &_val, sizeof(_val), NULL, NULL);
  if(ret_val != IC_SUCCESS)
    NRF_LOG_ERROR("TWI problem: %s\n", (uint32_t)g_return_val_string[ret_val]);
  _val &= ~(0x30);
  _val |= g_range << 4;

  m_config_reg(LIS3DH_REG_CTRL_REG4, _val);

  return IC_SUCCESS;
}

ic_return_val_e ic_lis3dh_get_g_range(uint8_t *range)
{
  uint8_t _val = 0x00;
  __auto_type ret_val  = TWI_READ_DATA(LIS3DH, LIS3DH_REG_CTRL_REG4, &_val, 1, NULL, NULL);
  if(ret_val != IC_SUCCESS)
    NRF_LOG_ERROR("TWI problem: %s\n", (uint32_t)g_return_val_string[ret_val]);

  _val &= (0x30);
  *range = _val >> 4;

  switch(*range)
  {
    case LIS3DH_RATE_G_RANGE_2g:
      NRF_LOG_INFO("G RANGE 2G\r\n");
      break;
    case LIS3DH_RATE_G_RANGE_4g:
      NRF_LOG_INFO("G RANGE 4G\r\n");
      break;
    case LIS3DH_RATE_G_RANGE_8g:
      NRF_LOG_INFO("G RANGE 8G\r\n");
      break;
    case LIS3DH_RATE_G_RANGE_16g:
      NRF_LOG_INFO("G RANGE 16G\r\n");
      break;
    default:
      NRF_LOG_INFO("ERROR: %d\r\n", *range);
      break;
  }

  return IC_SUCCESS;
}

#ifdef _DO_SELF_TEST

static acc_data_s m_ave_nost_acc_data = {0};
static acc_data_s m_ave_st_acc_data = {0};

ic_return_val_e ic_lis3dh_self_test1()
{
    /*  high-pass filter mode normal  */
  m_config_reg(LIS3DH_REG_CTRL_REG2, 0x00);
    /*  disable interrupt pin configuration */
  m_config_reg(LIS3DH_REG_CTRL_REG3, 0x00);
  m_config_reg(LIS3DH_REG_CTRL_REG4, 0x80);
  m_config_reg(LIS3DH_REG_CTRL_REG1, 0x47);

  return IC_SUCCESS;
}

ic_return_val_e ic_lis3dh_self_test2()
{
  __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, NULL, NULL);
  if(_ret_val == IC_SOFTWARE_BUSY)
    TWI_READ_DATA_FORCED(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, NULL, NULL);

  acc_data_s _nost_acc_data[5] = {0};

  for (int i = 0; i < 5;i++)
  {
    __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, &_nost_acc_data[i]);
    if(_ret_val == IC_SOFTWARE_BUSY)
      TWI_READ_DATA_FORCED(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, &_nost_acc_data[i]);
  }

    /*  get the average */
  for (int i = 0; i < 5; i++)
  {
    m_ave_nost_acc_data.x += _nost_acc_data[i].x;
    m_ave_nost_acc_data.y += _nost_acc_data[i].y;
    m_ave_nost_acc_data.z += _nost_acc_data[i].z;
  }

  m_ave_nost_acc_data.x /= 5;
  m_ave_nost_acc_data.y /= 5;
  m_ave_nost_acc_data.z /= 5;

    /*  enable self-test  */
  m_config_reg(LIS3DH_REG_CTRL_REG4, 0x82);

  return IC_SUCCESS;
}

ic_return_val_e ic_lis3dh_self_test3()
{
  __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, NULL, NULL);
  if(_ret_val == IC_SOFTWARE_BUSY)
    TWI_READ_DATA_FORCED(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, NULL, NULL);

  acc_data_s _st_acc_data[5] = {0};

  for (int i = 0; i < 5; i++)
  {
    __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, &_st_acc_data[i]);
    if(_ret_val == IC_SOFTWARE_BUSY)
      TWI_READ_DATA_FORCED(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback, &_st_acc_data[i]);
  }

    /*  get the average */
  for (int i = 0; i < 5; i++)
  {
    m_ave_st_acc_data.x += _st_acc_data[i].x;
    m_ave_st_acc_data.y += _st_acc_data[i].y;
    m_ave_st_acc_data.z += _st_acc_data[i].z;
  }

  m_ave_st_acc_data.x /= 5;
  m_ave_st_acc_data.y /= 5;
  m_ave_st_acc_data.z /= 5;

#define MIN_VAL 10
  if ((abs(abs(m_ave_st_acc_data.x) - abs(m_ave_nost_acc_data.x)) < MIN_VAL) &&
      (abs(abs(m_ave_st_acc_data.y) - abs(m_ave_nost_acc_data.y)) < MIN_VAL) &&
      (abs(abs(m_ave_st_acc_data.z) - abs(m_ave_nost_acc_data.z)) < MIN_VAL))
  {
      /*  disable sensor  */
    m_config_reg(LIS3DH_REG_CTRL_REG1, 0x00);
      /*  disable selftest  */
    m_config_reg(LIS3DH_REG_CTRL_REG4, 0x00);

    return IC_SUCCESS;
  }
  else
  {
    /*  disable sensor  */
    m_config_reg(LIS3DH_REG_CTRL_REG1, 0x00);
      /*  disable selftest  */
    m_config_reg(LIS3DH_REG_CTRL_REG4, 0x00);
  }

  return IC_ERROR;
}
#endif
