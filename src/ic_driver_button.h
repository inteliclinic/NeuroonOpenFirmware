/**
 * @file    ic_driver_button.h
 * @Author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @date    June, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_DRIVER_BUTTON_H
#define IC_DRIVER_BUTTON_H

typedef void(*p_btnCode)();

void ic_btn_pwr_press_handle_init(p_btnCode code);
void ic_btn_pwr_release_handle_init(p_btnCode code);
void ic_btn_pwr_long_press_handle_init(p_btnCode code);
void ic_btn_usb_connect_handle_init(p_btnCode code);
void ic_btn_usb_disconnect_handle_init(p_btnCode code);
void ic_btn_acc_rise_handle_init(p_btnCode code);
void ic_btn_acc_fall_handle_init(p_btnCode code);

void neuroon_exti_init(void);

#endif /* !IC_DRIVER_BUTTON_H */
