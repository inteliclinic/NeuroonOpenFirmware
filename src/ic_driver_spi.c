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

static struct transaction_queue_s{
  volatile uint8_t tail;
  volatile uint8_t head;
  struct{
    volatile bool occupied;
    ic_spi_instance_s instance;
  }data[IC_SPI_PENDIG_TRANSACTIONS];
}m_instance_queue;

static inline void pop_element(struct transaction_queue_s *queue){
  if(queue->head != queue->tail){
    if(++queue->tail==IC_SPI_PENDIG_TRANSACTIONS)
      queue->tail = 0;
  }
}

static inline bool put_queue_top(
    struct transaction_queue_s *queue,
    ic_spi_instance_s *instance)
{
  uint8_t _head = queue->head;
  uint8_t _tail = queue->tail;

  if(++_head == IC_SPI_PENDIG_TRANSACTIONS)
    _head = 0;

  if(_head == _tail)
    return false;

  memcpy(
      &queue->data[queue->head].instance,
      instance,
      sizeof(ic_spi_instance_s)
      );

  if(++queue->head == IC_SPI_PENDIG_TRANSACTIONS)
    queue->head = 0;

  return true;
}

#define get_queue_last(v) &(v).data[(v).head==0?IC_SPI_PENDIG_TRANSACTIONS-1:(v).head-1].instance

#define get_queue_first(v) &(v).data[(v).tail].instance

#define is_queue_empty(v) ((v).head == (v).tail)

static void clean_queue(struct transaction_queue_s *queue){
  while(!is_queue_empty(*queue)){
    pop_element(queue);
  }
}

static void spi_event_handler(nrf_drv_spi_evt_t const *p_event){
  UNUSED_VARIABLE(p_event);

  m_current_state.line_busy = false;

  if(!is_queue_empty(m_instance_queue)){
    __auto_type _instance = get_queue_first(m_instance_queue);
    if(!_instance->transaction_desc.open)
      ic_spi_cs_high(_instance);

    _instance->active = false;

    if (_instance->callback != NULL){
      _instance->callback(_instance->context);
    }

    pop_element(&m_instance_queue);

    if (!is_queue_empty(m_instance_queue)){
      m_current_state.line_busy = true;
      _instance = get_queue_first(m_instance_queue);
      ic_spi_cs_low(_instance);
      nrf_drv_spi_transfer(
          &m_current_state.nrf_drv_instance,
          _instance->transaction_desc.in_buffer,
          _instance->transaction_desc.in_len,
          _instance->transaction_desc.out_buffer,
          _instance->transaction_desc.out_len);
    }
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
    clean_queue(&m_instance_queue);
    nrf_drv_spi_config_t _spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    _spi_config.ss_pin    = NRF_DRV_SPI_PIN_NOT_USED;
    _spi_config.miso_pin  = IC_SPI_MISO_PIN;
    _spi_config.mosi_pin  = IC_SPI_MOSI_PIN;
    _spi_config.sck_pin   = IC_SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_current_state.nrf_drv_instance, &_spi_config, spi_event_handler));
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_spi_deinit(ic_spi_instance_s *instance){
  ASSERT(instance!=NULL);

  nrf_gpio_cfg_default(instance->pin);

  NRF_LOG_INFO("{%s}\n", (uint32_t)__func__);

  if(--m_current_state.spi_instance_cnt == 0){
    NRF_LOG_INFO("SPI killed\n");
    nrf_drv_spi_uninit(&m_current_state.nrf_drv_instance);
    nrf_gpio_cfg_default(IC_SPI_SCK_PIN);
    nrf_gpio_cfg_default(IC_SPI_MOSI_PIN);
    nrf_gpio_cfg_default(IC_SPI_MISO_PIN);
  }

  return IC_SUCCESS;
}

ic_return_val_e ic_spi_send(
    ic_spi_instance_s *instance,
    uint8_t *in_buffer,
    size_t in_len,
    uint8_t *out_buffer,
    size_t out_len,
    ic_spi_event_cb callback,
    void *context,
    bool open)
{
  ASSERT(instance!=NULL);
  ASSERT(in_buffer!=NULL);
  ASSERT(out_buffer!=NULL);


  instance->callback = callback;
  instance->context = context;

  instance->transaction_desc.in_len = in_len;
  instance->transaction_desc.in_buffer = in_buffer;
  instance->transaction_desc.out_len = out_len;
  instance->transaction_desc.out_buffer = out_buffer;
  instance->transaction_desc.open = open;

  __auto_type _ret_val = false;

  CRITICAL_REGION_ENTER();
  _ret_val = put_queue_top(&m_instance_queue, instance);
  CRITICAL_REGION_EXIT();

  if(!_ret_val)
    return IC_BUSY;
  else{
    instance->active = true;
    if(m_current_state.line_busy){
      return IC_SUCCESS;
    }
    else{
      m_current_state.line_busy = true;
      ic_spi_cs_low(instance);
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
