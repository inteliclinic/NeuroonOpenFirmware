/**
 * @file    ic_driver_twi.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_TWI_H
#define IC_DRIVER_TWI_H
#include <stdint.h>
#include <stddef.h>

#include "ic_common_types.h"
#include "app_twi.h"

/** @defgroup IC_TWI
 *  @{
 */

/**
 * @brief Callback prototype.
 *
 * @param[in] context provided by user preallocated data.
 *
 * @return void
 */
typedef void (*ic_twi_event_cb)(ic_return_val_e);

/**
 * @brief Struct holding TWI instance information.
 *
 * It describes device connected to TWI line and its handling.
 * DO NOT FILL IT MANUALLY!!
 */
typedef struct{
  void *nrf_twi_instance;             /** Nordic driver instance. */
  ic_twi_event_cb callback;           /** TWI IRQ callback */
  bool active;                        /** Is line ocupated by device */
  uint8_t device_address;             /** Devices TWI address */
  app_twi_transaction_t transaction;  /** RESERVED */
  app_twi_transfer_t transfers[2];    /** RESERVED */
}ic_twi_instance_s;

/**
 * @brief Macro allocates statically memory for TWI instance.
 *
 * This macro should be called once at the begining of module. It supposed to represent ONE device
 * connected to TWI line.
 *
 * @param name Name of instance.
 *
 */
#define TWI_REGISTER(name, address)                                                               \
  static ic_twi_instance_s name##_twi_instance = {.device_address = address};

/**
 * @brief   Macro simplifying usage of @ref ic_twi_init.
 *
 * This macro needs to be run once per driver instance.
 *
 * @param   name Name of instance.
 *
 * @return  @ref ic_twi_init.
 */
#define TWI_INIT(name)                                                                            \
  ic_twi_init(&name##_twi_instance);

/**
 * @brief Macro simplifying usage of @ref ic_twi_uninit.
 *
 * @param name Name of instance.
 *
 * @return  @ref ic_twi_uninit.
 */
#define TWI_UNINIT(name)                                                                          \
  ic_twi_uninit(&name##_twi_instance);

/**
 * @brief   Macro simplifying @ref ic_twi_send
 *
 * @param name      Name of instace.
 * @param in_buffer Prealocated buffer provided by user.
 * @param len       Length of buffer.
 * @param callback  Code with IRQ handler. Can be NULL.
 *
 * @return  @ref ic_twi_send.
 */
#define TWI_SEND_DATA(                                                                            \
    name,                                                                                         \
    in_buffer,                                                                                    \
    len,                                                                                          \
    callback)                                                                                     \
  ic_twi_send(&name##_twi_instance, in_buffer, len, callback, false)

/**
 * @brief   Macro simplifying @ref ic_twi_send
 *
 * Forced version of @ref TWI_SEND_DATA. Lets user write data even when bus is busy
 *
 * @param name      Name of instace.
 * @param in_buffer Prealocated buffer provided by user.
 * @param len       Length of buffer.
 * @param callback  Code with IRQ handler. Can be NULL.
 *
 * @return  @ref ic_twi_send.
 */
#define TWI_SEND_DATA_FORCED(                                                                            \
    name,                                                                                         \
    in_buffer,                                                                                    \
    len,                                                                                          \
    callback)                                                                                     \
  ic_twi_send(&name##_twi_instance, in_buffer, len, callback, true)

/**
 * @brief   Macro simplifying @ref ic_twi_read
 *
 * @param name      Name of instance.
 * @param reg_addr  Target register address.
 * @param in_buffer Prealocated buffer provided by user.
 * @param len       Length of buffer.
 * @param callback  code with IRQ handler. Can be NULL.
 *
 * @return  @ref ic_twi_read
 */
#define TWI_READ_DATA(                                                                            \
    name,                                                                                         \
    reg_addr,                                                                                     \
    in_buffer,                                                                                    \
    len,                                                                                          \
    callback)                                                                                     \
  ic_twi_read(&name##_twi_instance, reg_addr, in_buffer, len, callback, false)

/**
 * @brief   Macro simplifying @ref ic_twi_read
 *
 * Forced version of @ref TWI_READ_DATA. Lets user read data even when bus is busy
 *
 * @param name      Name of instance.
 * @param reg_addr  Target register address.
 * @param in_buffer Prealocated buffer provided by user.
 * @param len       Length of buffer.
 * @param callback  code with IRQ handler. Can be NULL.
 *
 * @return  @ref ic_twi_read
 */
#define TWI_READ_DATA_FORCED(                                                                            \
    name,                                                                                         \
    reg_addr,                                                                                     \
    in_buffer,                                                                                    \
    len,                                                                                          \
    callback)                                                                                     \
  ic_twi_read(&name##_twi_instance, reg_addr, in_buffer, len, callback, true)

/**
 * @brief TWI instance initialization function.
 *
 * Function should be run onse per initialization (instance should be deinitialized before
 * reinitialization. It has to be called BEFORE any transfer request(read or write).
 *
 * @param instance  Pointer to allocated instance memory.
 *
 * @return  IC_SUCCESS when everything went ok (@ref ic_return_val_e).
 */
ic_return_val_e ic_twi_init(ic_twi_instance_s * instance);

/**
 * @brief TWI instance deinitialization funcion.
 *
 * Function should be called upon previously initialized instance.
 *
 * @param instance  Pointer to allocated instance memory.
 *
 * @return  IC_SUCCESS when everything went ok (@ref ic_return_val_e).
 */
ic_return_val_e ic_twi_uninit(ic_twi_instance_s *instance);

/**
 * @brief Start sendig data transaction.
 *
 * When transnaction is finished, it calls callback(if provided).
 *
 * @param instance  Pointer to allocated instance memory.
 * @param in_buffer Prealocated buffer provided by user.
 * @param len       Lenght of buffer.
 * @param callback  code with IRQ handler. Can be NULL.
 *
 * @return  Function will return IC_BUSY (@ref ic_return_val_e) if previous transaction did not
 * fininish.
 */
ic_return_val_e ic_twi_send(
    ic_twi_instance_s * const instance,
    uint8_t *in_buffer,
    size_t len,
    ic_twi_event_cb callback,
    bool force);

/**
 * @brief Start reading data transaction.
 *
 * When transnaction is finished, it calls callback(if provided)
 *
 * @param instance  Pointer to allocated instance memory.
 * @param reg_addr  target register address.
 * @param in_buffer Prealocated buffer provided by user.
 * @param len       Lenght of buffer.
 * @param callback  code with IRQ handler.
 *
 * @return  Function will return IC_BUSY (@ref ic_return_val_e) if previous transaction did not
 * fininish.
 */
ic_return_val_e ic_twi_read(
    ic_twi_instance_s *const instance,
    uint8_t reg_addr,
    uint8_t *in_buffer,
    size_t len,
    ic_twi_event_cb callback,
    bool force);


#endif /* !IC_DRIVER_TWI_H */
