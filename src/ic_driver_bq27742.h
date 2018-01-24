/**
 * @file    ic_driver_bq27742.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    January, 2018
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_BQ27742_H
#define IC_DRIVER_BQ27742_H

#ifdef __cplusplus
extern "C" {
#endif

/** Battery charging state */
typedef enum {
  BATT_NOTCHARGING = 0,   //!< Charger disconnected
  BATT_CHARGING = 1,    //!< Charger connected and battery is charging
  BATT_CHARGED = 2,    //!< Charger connected and battery is full charged
  BATT_CHARGER_FAULT =3 //!< Charger connected and battery is on fault state
}en_chargerState;

void ic_bq_flash_image();
void ic_bq_reset();

uint16_t ic_bq_getChargeLevel(void);
en_chargerState ic_bq_getChargerState(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !IC_DRIVER_BQ27742_H */
