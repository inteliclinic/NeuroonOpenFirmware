/**
 * @file    ic_driver_spi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    czerwiec, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_driver_spi.h"
#include "ic_config.h"

#include "app_error.h"

#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

#define NRF_LOG_MODULE_NAME "SPI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define MAX_INSTANCES 5

static struct{
  const nrf_drv_spi_t nrf_drv_instance;
  uint8_t spi_instance_cnt;
  bool line_busy;
  ic_spi_instance_s *instance;
  ic_spi_event_cb callback;
}m_current_state = {
                    .nrf_drv_instance = NRF_DRV_SPI_INSTANCE(IC_SPI_INSTANCE),
                    .spi_instance_cnt = 0,
                    .line_busy        = false,
                    .instance          = NULL,
                    .callback         = NULL
                  };

static struct{
  uint8_t tail;
  uint8_t head;
  struct{
    bool occupied;
    ic_spi_instance_s instance;
  }data[MAX_INSTANCES];
}m_instance_queue;

static bool add_instance_to_queue(ic_spi_instance_s *instance){

  if(m_instance_queue.data[m_instance_queue.head].occupied == true)
    return false;

  memcpy(
      &m_instance_queue.data[m_instance_queue.head++].instance,
      instance,
      sizeof(ic_spi_instance_s));

  m_instance_queue.data[m_instance_queue.head-1].occupied = true;

  if(m_instance_queue.head == MAX_INSTANCES) m_instance_queue.head = 0;

  return true;
}

static ic_spi_instance_s *get_instance(){

  if(m_instance_queue.data[m_instance_queue.tail].occupied == false)
    return NULL;

  m_instance_queue.tail = m_instance_queue.tail == MAX_INSTANCES ? 0 : m_instance_queue.tail;
  m_instance_queue.data[m_instance_queue.tail].occupied = false;

  return &m_instance_queue.data[m_instance_queue.tail].instance;
}

static ic_spi_instance_s *show_next(){
  return m_instance_queue.data[m_instance_queue.tail].occupied == true ?
    &m_instance_queue.data[m_instance_queue.tail].instance :
    NULL;
}

static void clear_queuue(){
  memset(&m_instance_queue, 0, sizeof(m_instance_queue));
}

static void spi_event_handler(nrf_drv_spi_evt_t const *p_event){
  UNUSED_VARIABLE(p_event);

  m_current_state.line_busy = false;

  __auto_type _instance = get_instance();
  _instance->active = false;

  if (_instance->callback != NULL){
    _instance->callback(NULL);
  }

  _instance = show_next();

  if (_instance != NULL){
    m_current_state.line_busy = true;
    nrf_drv_spi_transfer(
        &m_current_state.nrf_drv_instance,
        _instance->transaction_desc.in_buffer,
        _instance->transaction_desc.in_len,
        _instance->transaction_desc.out_buffer,
        _instance->transaction_desc.out_len);
  }
}

ic_return_val_e ic_spi_init(ic_spi_instance_s *instance, uint8_t pin){
  ASSERT(instance!=NULL);

  /*instance->nrf_spi_instance = (void *)&m_current_state.nrf_drv_instance;*/

  instance->pin = pin;
  instance->active = false;
  instance->callback = NULL;
  memset(&instance->transaction_desc, 0, sizeof(instance->transaction_desc));

  nrf_gpio_cfg_output(instance->pin);

  if(m_current_state.spi_instance_cnt++ == 0){
    clear_queuue();
    nrf_drv_spi_config_t _spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    _spi_config.ss_pin    = NRF_DRV_SPI_PIN_NOT_USED;
    _spi_config.miso_pin  = IC_SPI_MISO_PIN;
    _spi_config.mosi_pin  = IC_SPI_MOSI_PIN;
    _spi_config.sck_pin   = IC_SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_current_state.nrf_drv_instance, &_spi_config, spi_event_handler));
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_spi_send(
    ic_spi_instance_s *instance,
    uint8_t *in_buffer,
    size_t in_len,
    uint8_t *out_buffer,
    size_t out_len,
    ic_spi_event_cb callback)
{
  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(out_buffer!=NULL);

  instance->callback = callback;

  instance->transaction_desc.in_len = in_len;
  instance->transaction_desc.in_buffer = in_buffer;
  instance->transaction_desc.out_len = out_len;
  instance->transaction_desc.out_buffer = out_buffer;

  if(!add_instance_to_queue(instance))
    return IC_BUSY;
  else{
    instance->active = true;
    if(m_current_state.line_busy){
      return IC_SUCCESS;
    }
    else{
      m_current_state.instance = instance;
      m_current_state.line_busy = true;
      __auto_type _ret_val =
        nrf_drv_spi_transfer(
            &m_current_state.nrf_drv_instance,
            in_buffer,
            in_len,
            out_buffer,
            out_len);

      switch(_ret_val){
        case NRF_SUCCESS:
          return IC_SUCCESS;
        case NRF_ERROR_BUSY:
          return IC_BUSY;
        default:
          return IC_ERROR;
      }
    }
  }
}

void ic_spi_cs_high(ic_spi_instance_s *instance){
  nrf_gpio_pin_set(instance->pin);
}

void ic_spi_cs_low(ic_spi_instance_s *instance){
  nrf_gpio_pin_clear(instance->pin);
}

bool ic_spi_instance_busy(ic_spi_instance_s *instance){
  return instance->active;
}
