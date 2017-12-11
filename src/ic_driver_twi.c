/**
 * @file    ic_driver_twi.c
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */
#include "nrf_drv_twi.h"
#include "app_twi.h"

#include "ic_driver_twi.h"
#include "ic_config.h"

#include "app_error.h"
#include "sdk_errors.h"
#include "nrf_assert.h"

#define NRF_LOG_MODULE_NAME "TWI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_gpio.h"

#define TWI_READ_OP(addr) (addr|0x01)
#define TWI_WRITE_OP(addr) (addr&0xFE)

// Helper macro. Not intended to be used directly
#define IC_TWI_TRANSFER(_name, _operation, _p_data, _length, _flags)                              \
    _name.p_data    = (uint8_t *)(_p_data);                                                       \
    _name.length    = _length;                                                                    \
    _name.operation = _operation;                                                                 \
    _name.flags     = _flags

// Macro covering Nordics APP_TWI_WRITE
#define IC_TWI_WRITE(name, address, p_data, length, flags)                                        \
    IC_TWI_TRANSFER(name, TWI_WRITE_OP(address), p_data, length, flags)

// Macro covering Nordics APP_TWI_READ
#define IC_TWI_READ(name, address, p_data, length, flags)                                         \
    IC_TWI_TRANSFER(name, TWI_READ_OP(address), p_data, length, flags)

/**
 * @brief 
 */
struct transaction_queue_field_s{
  ic_twi_event_cb callback;           /** TWI IRQ callback */
  void *context;                      /** TWI IRQ callbacks context */
  app_twi_transaction_t transaction;  /** RESERVED */
  app_twi_transfer_t transfers[2];    /** RESERVED */
};

/**
 * @brief
 */
struct transaction_queue_s{
  struct transaction_queue_field_s callback_array[IC_TWI_PENDIG_TRANSACTIONS];
  uint8_t volatile head;
  uint8_t volatile tail;
}m_transaction_queue;

/**
 * @brief
 */
static struct{
  app_twi_t nrf_drv_instance;
  uint8_t twi_instance_cnt;
}m_curren_state =
{
  .nrf_drv_instance = APP_TWI_INSTANCE(IC_TWI_INSTANCE),
  .twi_instance_cnt = 0,
};

static inline void pop_element(struct transaction_queue_s *queue){
  if(queue->head != queue->tail){
    if(++queue->tail==IC_TWI_PENDIG_TRANSACTIONS)
      queue->tail = 0;
  }
}

static inline bool put_queue_top(
    struct transaction_queue_s *queue,
    ic_twi_event_cb callback,
    void *context,
    app_twi_transaction_t transaction,
    app_twi_transfer_t *transfers) // Assumed there are 2 * sizeof(app_twi_transfer_t) bytes
{
  uint8_t _head = queue->head;
  uint8_t _tail = queue->tail;

  if(++_head == IC_TWI_PENDIG_TRANSACTIONS)
    _head = 0;

  if(_head == _tail)
    return false;

  queue->callback_array[queue->head].callback = callback;
  queue->callback_array[queue->head].context = context;
  queue->callback_array[queue->head].transaction = transaction;
  queue->callback_array[queue->head].transaction.p_user_data =
    &(queue->callback_array[queue->head]);
  queue->callback_array[queue->head].transaction.p_transfers =
    queue->callback_array[queue->head].transfers;
  memcpy(
      queue->callback_array[queue->head].transfers,
      transfers,
      sizeof(queue->callback_array[queue->head].transfers)
      );

  if(++queue->head == IC_TWI_PENDIG_TRANSACTIONS)
    queue->head = 0;

  return true;
}

#define get_queue_last(v) (v).callback_array[(v).head==0?IC_TWI_PENDIG_TRANSACTIONS-1:(v).head-1]

#define get_queue_first(v) (v).callback_array[(v).tail]

#define is_queue_empty(v) ((v).head == (v).tail)

void clean_queue(struct transaction_queue_s *queue){
  while(!is_queue_empty(*queue)){
    pop_element(queue);
  }
}

/**
 * @brief TWI IRQ handler
 *
 * @param result    message from driver @ref NRF_ERRORS_BASE
 * @param p_context user data passed by driver
 */
static void m_twi_event_handler(uint32_t result, void *p_context){
  struct transaction_queue_field_s * _transaction = p_context;

  if(_transaction != NULL){
    if(!is_queue_empty(m_transaction_queue)){
      __auto_type _callback = get_queue_first(m_transaction_queue).callback;
      __auto_type _context = get_queue_first(m_transaction_queue).context;

      if(_callback != NULL)
        _callback(result == NRF_SUCCESS ? IC_SUCCESS : IC_ERROR, _context);

      pop_element(&m_transaction_queue);
    }
  }else{
    NRF_LOG_INFO("no instance data!\n");
  }
}

/**
 * @brief Transaction function
 *
 * @param instance  Externally allocated device instance.
 * @param address   Devices TWI address.
 * @param reg_addr  Target register for read purpose.
 * @param buffer    Preallocated transfer buffer.
 * @param len       Length buffer.
 * @param callback  IRQ handler code.
 * @param read      if true - read transaction. Otherwise - write.
 *
 * @return  IC_SUCCESS, when everything went ok. IC_BUSY when previously started device transaction
 * wasnt handled yet
 */
static ic_return_val_e m_ic_twi_transaction(
    ic_twi_instance_s *const instance,
    uint8_t address,
    uint8_t reg_addr,
    uint8_t *buffer,
    size_t len,
    ic_twi_event_cb callback,
    void *context,
    bool read,
    bool force)
{
  UNUSED_PARAMETER(force);
  ASSERT(buffer!=NULL);
  ASSERT(instance!=NULL);
  ASSERT(len<=255);

  app_twi_transaction_t transaction = {.callback = m_twi_event_handler};
  app_twi_transfer_t transfers[2];

  if(read){
    IC_TWI_WRITE(transfers[0], address, &reg_addr, 1, APP_TWI_NO_STOP);
    IC_TWI_READ(transfers[1], address, buffer, len, 0x00);
    transaction.number_of_transfers = 2;
  }
  else{
    IC_TWI_WRITE(transfers[0], address, buffer, len, 0x00);
    transaction.number_of_transfers = 1;
  }

  if(callback != NULL){
    __auto_type _ret_val = false;
    CRITICAL_REGION_ENTER();
    _ret_val = put_queue_top(
        &m_transaction_queue,
        callback,
        context,
        transaction,
        transfers);
    CRITICAL_REGION_EXIT();
    if((!_ret_val) == true) {
      return IC_SOFTWARE_BUSY;
    }
  }

  __auto_type _ret_val = callback == NULL ?
    app_twi_perform(
        &m_curren_state.nrf_drv_instance,
        transfers,
        transaction.number_of_transfers,
        NULL) :
    app_twi_schedule(
        &m_curren_state.nrf_drv_instance,
        &get_queue_last(m_transaction_queue).transaction);

  switch (_ret_val){
    case NRF_SUCCESS:
      return IC_SUCCESS;
    case NRF_ERROR_BUSY:
    default:
      return _ret_val == NRF_ERROR_BUSY? IC_DRIVER_BUSY : IC_ERROR;
  }
}

static app_twi_transaction_t const * m_nordic_twi_queue[IC_TWI_PENDIG_TRANSACTIONS+1];
static nrf_drv_twi_config_t m_twi_config = {
    .frequency          = (nrf_twi_frequency_t)IC_TWI_FREQUENCY,
    .scl                = IC_TWI_SCL_PIN,
    .sda                = IC_TWI_SDA_PIN,
    .interrupt_priority = IC_TWI_IRQ_PRIORITY,
    .clear_bus_init     = true,
    .hold_bus_uninit    = true};


////////////////////////////////////////////////////////////////////////////////////////////////////
ic_return_val_e ic_twi_init(ic_twi_instance_s * instance){

  ASSERT(instance!=NULL);

  if(m_curren_state.twi_instance_cnt++ == 0){
    __auto_type err_code = app_twi_init(
        &m_curren_state.nrf_drv_instance,
        &m_twi_config,
        IC_TWI_PENDIG_TRANSACTIONS,
        m_nordic_twi_queue);

    APP_ERROR_CHECK(err_code);
  }

  return IC_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ic_return_val_e ic_twi_deinit(ic_twi_instance_s *instance){

  if (--m_curren_state.twi_instance_cnt == 0){
    app_twi_uninit(&m_curren_state.nrf_drv_instance);
    nrf_gpio_cfg_default(IC_TWI_SCL_PIN);
    nrf_gpio_cfg_default(IC_TWI_SDA_PIN);
  }


  return IC_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ic_return_val_e ic_twi_send(
    ic_twi_instance_s * const instance,
    uint8_t *buffer,
    size_t len,
    ic_twi_event_cb callback,
    void *context,
    bool force)

{

  return m_ic_twi_transaction(
      instance,                 // ic_twi_instance_s *const instance,
      instance->device_address, // uint8_t address,
      0x00,                     // uint8_t reg_addr,
      buffer,                   // uint8_t *buffer,
      len,                      // size_t len,
      callback,                 // ic_twi_event_cb callback,
      context,
      false,
      force);                   // bool read

}

////////////////////////////////////////////////////////////////////////////////////////////////////
ic_return_val_e ic_twi_read(
    ic_twi_instance_s *const instance,
    uint8_t reg_addr,
    uint8_t *buffer,
    size_t len,
    ic_twi_event_cb callback,
    void *context,
    bool force)
{

  return m_ic_twi_transaction(
      instance,                 // ic_twi_instance_s *const instance,
      instance->device_address, // uint8_t address,
      reg_addr,                 // uint8_t reg_addr,
      buffer,                   // uint8_t *buffer,
      len,                      // size_t len,
      callback,                 // ic_twi_event_cb callback,
      context,
      true,
      force);                    // bool read

}

void ic_twi_refresh_bus(){
  NRF_LOG_INFO("{%s}\n", (uint32_t)__func__);
  /*NRF_LOG_FLUSH();*/

  app_twi_uninit(&m_curren_state.nrf_drv_instance);
  clean_queue(&m_transaction_queue);
  app_twi_init(
      &m_curren_state.nrf_drv_instance,
      &m_twi_config,
      IC_TWI_PENDIG_TRANSACTIONS,
      m_nordic_twi_queue);
}
