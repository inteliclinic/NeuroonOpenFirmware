/**
 * @file    ic_driver_lis3dh.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
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
#include "ic_driver_button.h"


TWI_REGISTER(LIS3DH, LIS3DH_SLAVE_ADDR);

static uint8_t lis3dh_bufer[10];

static void acc_twi_callback(ic_return_val_e e){
  if(e != IC_SUCCESS){
    NRF_LOG_ERROR("{%s}\n",(uint32_t)__func__);
    return;
  }
  /*NRF_LOG_INFO("x: %d, y: %d, z: %d\n",*/
      /*((acc_data_s *)&lis3dh_bufer[1])->x,*/
      /*((acc_data_s *)&lis3dh_bufer[1])->y,*/
      /*((acc_data_s *)&lis3dh_bufer[1])->z);*/
  if((lis3dh_bufer[0] & 0b00001000)==0){
     __auto_type _ret_val = TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback);
     if(_ret_val == IC_SOFTWARE_BUSY)
       TWI_READ_DATA_FORCED(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback);
    /*NRF_LOG_INFO("Read: %s\n",(uint32_t)g_return_val_string[_ret_val]);*/
  }
}

static void (*m_fp)(acc_data_s) = NULL;

static void acc_twi_read_callback(ic_return_val_e e){
  UNUSED_PARAMETER(e);
  if(m_fp != NULL){
    NRF_LOG_INFO("Status: 0x%X\n",lis3dh_bufer[0]);
    m_fp(*(acc_data_s *)&lis3dh_bufer[1]);
    m_fp = NULL;
  }
}

static volatile bool m_lock = false;

#define LOCK    m_lock = true
#define UNLOCK  m_lock = false
#define LOCKED  (m_lock == true)

static void acc_int_callback(enum exti_edge_dir edge){
  ic_return_val_e _ret_val;
  switch(edge){
    case EXTI_EDGE_DOWN:
      NRF_LOG_INFO("Edge down\n");
      break;
    case EXTI_EDGE_UP:
      NRF_LOG_INFO("Edge up\n");
      _ret_val =
        TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_callback);
      if(_ret_val != IC_SUCCESS){
        NRF_LOG_ERROR("TWI problem: %s\n", (uint32_t)g_return_val_string[_ret_val]);
        if(_ret_val == IC_SOFTWARE_BUSY)
          _ret_val = TWI_READ_DATA_FORCED(
              LIS3DH,
              LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG,
              lis3dh_bufer,
              7,
              acc_twi_callback);
      } else {
        NRF_LOG_INFO("x:%d y:%d z:%d\n",
            ((acc_data_s *)&lis3dh_bufer[1])->x,
            ((acc_data_s *)&lis3dh_bufer[1])->y,
            ((acc_data_s *)&lis3dh_bufer[1])->z
            );
      }
      break;
    default:
      break;
  }
}

ic_return_val_e ic_lis3dh_read_data(void(*fp)(acc_data_s data)){
  if (m_fp != NULL)
    return IC_BUSY;
  m_fp = fp;
  TWI_READ_DATA(LIS3DH, LIS3DH_REG_STATUS_REG|LIS3DH_INC_REG, lis3dh_bufer, 7, acc_twi_read_callback);
  return IC_SUCCESS;
}

static void m_config_reg(uint8_t reg, uint8_t reg_val){
  uint8_t _d[] = {reg, reg_val};
  __auto_type _ret_val = TWI_SEND_DATA(LIS3DH, _d, sizeof(_d), NULL);
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

  __auto_type _ret_val = TWI_READ_DATA(LIS3DH, reg, &_reg_val_current, sizeof(_reg_val_current), NULL);

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
#define ENABLE_DRDY_INT   SET_REG_BIT(LIS3DH_REG_CTRL_REG3, LIS3DH_CTRL_REG3_I1_DRDY)

ic_return_val_e ic_lis3dh_init (void){
  ic_acc_exti_handle_init(acc_int_callback);
  TWI_INIT(LIS3DH);
  uint8_t _val = 0x11;

  __auto_type ret_val  = TWI_READ_DATA(LIS3DH, LIS3DH_REG_WHO_AM_I, &_val, sizeof(_val), NULL);
  NRF_LOG_INFO("WHO AM I: 0x%X\t ret_val: %d\n", _val==LIS3DH_WHO_AM_I, ret_val);

  DISABLE_DRDY_INT;

  uint8_t _reg_val_combo[][2] =
  {
    {LIS3DH_REG_CTRL_REG2, 0x00},
    {LIS3DH_REG_CTRL_REG5, 0x00},
    {LIS3DH_REG_CTRL_REG4, 0x00},
    {LIS3DH_REG_CTRL_REG1, LIS3DH_CTRL_REG1_25HZ_RATE|
      LIS3DH_CTRL_REG1_XEN|LIS3DH_CTRL_REG1_YEN|LIS3DH_CTRL_REG1_ZEN}
    /*{LIS3DH_REG_CTRL_REG4, LIS3DH_CTRL_REG4_FS1|LIS3DH_CTRL_REG4_FS0|LIS3DH_CTRL_REG4_HR}*/
  };

  for(int i=0; i<sizeof(_reg_val_combo)/sizeof(_reg_val_combo[0]); ++i){
    m_config_reg(_reg_val_combo[i][0], _reg_val_combo[i][1]);
  }

  ENABLE_DRDY_INT;

  NRF_LOG_FLUSH();

  return IC_SUCCESS;
}
