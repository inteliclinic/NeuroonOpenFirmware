/**
 * @file    ic_driver_afe4400.h
 * @author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_DRIVER_AFE4400_H
#define _IC_DRIVER_AFE4400_H

#include <stdint.h>
#include <stddef.h>
#include "ic_common_types.h"

#define AFE_NRF_DEBUG
#undef AFE_NRF_DEBUG

/** @defgroup AFE4400_FUNCTIONS_DECLARATION
 *  @ingroup LOW_LEVEL_DRIVER_AFE4400
*  @{
 */

/**
 * @brief Structure for holding led values
 *
 */
typedef struct __attribute__((packed))
{
  uint32_t ir_val;
  uint32_t air_val;
  uint32_t red_val;
  uint32_t ared_val;
  uint32_t ir_diff;
  uint32_t red_diff;
}ic_afe_val_s;

typedef void (*ic_afe_event_cb_done)(ic_afe_val_s);

/**
 * @brief AFE4400 initialization function
 *
 * Initialize SPI interface for AFE4400 and set module in reading mode (read from registers)
 */
ic_return_val_e ic_afe_init(void);

/**
 * @brief Deinitialization of AFE4400
 *
 */
ic_return_val_e ic_afe_deinit(void);

/**
 * @brief Read all of the leds registers
 *
 * @param led_val - pointer to led values structure
 * @param cb      - callback for reading data when available
 *
 * @return IC_SUCCESS if everything goes okay
 *
 * Example:
 * @code
 *
 * led_val_s led_value;
 *
 * ic_afe_get_values(&led_value, my_callback_fun);
 *
 * @endcode
 */
ic_return_val_e ic_afe_get_values(ic_afe_event_cb_done cb, bool force);

/**
 *  @}
 */
#endif /* !_IC_DRIVER_AFE4400_H */
