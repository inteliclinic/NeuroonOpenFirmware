/**
 * @file    ic_command_task.c
 * @author  Wojtek Węclewski <w.weclewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#include <string.h>

#include "ic_ble_service.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ic_command_task.h"

#define NRF_LOG_MODULE_NAME "CMD"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

enum e_command_desc{
  RGB_LED_DESC = 0,
  POWER_LED_DESC,
  VIBRATOR_DESC,
  PULSEOXIMETER_DESC,
  E_ALARM_DESC,
  DEVICE_DESC,
  STATUS_DESC,
  DFU_DESC,
  FEED_DESC,
  UNLOCK_DESC,

  NUM_OF_COMMANDS
};

typedef struct{
  e_cmd cmd;
  union{
    void *p_func;
    void (*cmd_handle)(u_BLECmdPayload);
  }cmd_callback;
}command_desc_t;

static TaskHandle_t m_cmd_main_task_handle;
static void m_cmd_handle(uint8_t *, size_t);

static void default_cmd_handle(u_BLECmdPayload);
static void wrong_cmd_handle(e_cmd);

xQueueHandle cmd_queue;
static command_desc_t m_cmd_list[NUM_OF_COMMANDS] = {
  {
    .cmd = RGB_LED_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = POWER_LED_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = VIBRATOR_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = PULSEOXIMETER_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = E_ALARM_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = DEVICE_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = STATUS_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = DFU_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = FEED_CMD,
    .cmd_callback.cmd_handle = default_cmd_handle
  },
  {
    .cmd = UNLOCK_MASK,
    .cmd_callback.cmd_handle = default_cmd_handle
  }
};

static ic_return_val_e m_cmd_task_connect_to_cmd(void *p_func, command_desc_t *cmd_desc){
  cmd_desc->cmd_callback.p_func = p_func;
  return p_func != NULL ? IC_SUCCESS : IC_ERROR;
}

ic_return_val_e cmd_task_connect_to_rgb_led_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[RGB_LED_DESC]);
}

ic_return_val_e cmd_task_connect_to_power_led_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[POWER_LED_DESC]);
}

ic_return_val_e cmd_task_connect_to_vibrator_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[VIBRATOR_DESC]);
}

ic_return_val_e cmd_task_connect_to_pulseoximeter_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[PULSEOXIMETER_DESC]);
}

ic_return_val_e cmd_task_connect_to_e_alarm_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[E_ALARM_DESC]);
}

ic_return_val_e cmd_task_connect_to_device_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[DEVICE_DESC]);
}

ic_return_val_e cmd_task_connect_to_status_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[STATUS_DESC]);
}

ic_return_val_e cmd_task_connect_to_dfu_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[DFU_DESC]);
}

ic_return_val_e cmd_task_connect_to_feed_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[FEED_DESC]);
}

ic_return_val_e cmd_task_connect_to_unlock_cmd(void (*p_func)(u_BLECmdPayload)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[UNLOCK_DESC]);
}

void cmd_main_task(void *args){
  UNUSED_PARAMETER(args);
  cmd_queue = xQueueCreate(5, sizeof(u_cmdFrameContainer));
  if(NULL == cmd_queue){
    NRF_LOG_ERROR("CMD queue init error\n");
    APP_ERROR_HANDLER(NRF_ERROR_INTERNAL);
  ble_iccs_connect_to_cmd(m_cmd_handle);
  }

  uint8_t i = 0;
  u_cmdFrameContainer tmp_frame;
  memset(&tmp_frame, 0, sizeof(u_cmdFrameContainer));

  for(;;){
    xQueueReceive(cmd_queue, &(tmp_frame), portMAX_DELAY);
    if(tmp_frame.frame.sync != 0xEE || !neuroon_cmd_frame_validate(tmp_frame.data, 20)){
      NRF_LOG_ERROR("Wrong SYNC/CRC\n");
      continue;
    }
    for(i = 0; i < NUM_OF_COMMANDS; i++){
      if(tmp_frame.frame.cmd == m_cmd_list[i].cmd){
        m_cmd_list[i].cmd_callback.cmd_handle(tmp_frame.frame.payload);
        break;
      }
    }
    if(i == NUM_OF_COMMANDS){
      wrong_cmd_handle(tmp_frame.frame.cmd);
    }
  }
  cmd_module_destroy();
}

void cmd_module_init(void){
  if(pdPASS != xTaskCreate(cmd_main_task, "CMD", 256, NULL, IC_FREERTOS_TASK_PRIORITY_LOW, &m_cmd_main_task_handle)){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

bool cmd_queue_reset(void){
  if(NULL == cmd_queue)
    return false;
  xQueueReset(cmd_queue);
  return true;
}

void cmd_module_destroy(void){
  vQueueDelete(cmd_queue);
  vTaskDelete(m_cmd_main_task_handle);
  taskYIELD();
}

static void m_cmd_handle(uint8_t *data, size_t len){
  if(data == NULL || data == (void *)0x01) return; //TODO error is lower(bluetooth driver - here is only a dirty fix
  if(len < sizeof(s_frame)){
    NRF_LOG_ERROR("Uncomplete frame. Number of bytes: %d\n", len);
    return;
  }
  xQueueSendFromISR(cmd_queue, (u_cmdFrameContainer*)data, NULL);
}

static void default_cmd_handle(u_BLECmdPayload cmd_payload){
  NRF_LOG_INFO("Payload:\n");
  for(uint8_t i = 0; i < sizeof(u_BLECmdPayload); i++)
    NRF_LOG_INFO(" %d\n", cmd_payload.data[i]);
}

static void wrong_cmd_handle(e_cmd cmd){
  NRF_LOG_ERROR("Error: Wrong command: %d\n", cmd);
}
