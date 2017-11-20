/**
 * @file    ic_command_task.h
 * @author  Wojtek Węclewski <w.weclewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef IC_COMMAND_TASK_H
#define IC_COMMAND_TASK_H

#include "ic_frame_handle.h"
#include "ic_config.h"

void cmd_module_init(void);
bool cmd_queue_reset(void);
void cmd_module_destroy(void);
ic_return_val_e cmd_task_connect_to_rgb_led_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_power_led_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_vibrator_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_pulseoximeter_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_e_alarm_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_device_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_status_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_dfu_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_feed_cmd(void (*p_func)(u_BLECmdPayload));
ic_return_val_e cmd_task_connect_to_unlock_cmd(void (*p_func)(u_BLECmdPayload));

#endif /* !IC_COMMAND_TASK_H */
