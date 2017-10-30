/**
 * @file    ic_common_types.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_COMMON_TYPES_H
#define IC_COMMON_TYPES_H

typedef enum{
  IC_SUCCESS = 0x00,
  IC_BLE_NOT_CONNECTED,
  IC_BUSY,
  IC_DRIVER_BUSY,
  IC_SOFTWARE_BUSY,
  IC_WARNING,
  IC_ERROR,
  IC_UNKNOWN_ERROR
}ic_return_val_e;

extern const char *g_return_val_string[];

#endif /* !IC_COMMON_TYPES_H */
