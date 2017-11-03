/**
 * @file    ic_command_task.c
 * @author  Wojtek Węclewski <w.weclewski@inteliclinic.com>
 * @date    November, 2017
 * @brief   Brief description
 *
 * Description
 */

#include <string.h>

#include "ic_command_task.h"
#include "ic_ble_service.h"

#include "FreeRTOS.h"
#include "task.h"

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
  s_frame cmd_frame;
  union{
    void *p_func;
    /*void (*unknown_cmd_handle)(e_cmd);*/
    void (*cmd_handle)(u_BLECmdPayload*);
  }cmd_callback;
  bool is_valid;
}command_desc_t;

static TaskHandle_t m_cmd_main_task_handle;
static void m_cmd_handle(uint8_t *, size_t);

static void default_cmd_handle(u_BLECmdPayload*);
static void wrong_cmd_handle(e_cmd);

/*static void cmd_task_init_cmd(e_cmd, void*, s_frame);*/
static command_desc_t m_cmd_list[NUM_OF_COMMANDS] = {
  {
    .cmd = RGB_LED_CMD,
    /*.cmd_frame = m_cmd_frame_list[RGB_LED_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = POWER_LED_CMD,
    /*.cmd_frame = m_cmd_frame_list[POWER_LED_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = VIBRATOR_CMD,
    /*.cmd_frame = m_cmd_frame_list[VIBRATOR_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = PULSEOXIMETER_CMD,
    /*.cmd_frame = m_cmd_frame_list[PULSEOXIMETER_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = E_ALARM_CMD,
    /*.cmd_frame = m_cmd_frame_list[E_ALARM_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = DEVICE_CMD,
    /*.cmd_frame = m_cmd_frame_list[DEVICE_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = STATUS_CMD,
    /*.cmd_frame = m_cmd_frame_list[STATUS_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = DFU_CMD,
    /*.cmd_frame = m_cmd_frame_list[DFU_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = FEED_CMD,
    /*.cmd_frame = m_cmd_frame_list[FEED_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  },
  {
    .cmd = UNLOCK_MASK,
    /*.cmd_frame = m_cmd_frame_list[UNLOCK_DESC],*/
    .cmd_callback.cmd_handle = default_cmd_handle,
    .is_valid = false
  }
};

static ic_return_val_e m_cmd_task_connect_to_cmd(void *p_func, command_desc_t *cmd_desc){
  cmd_desc->cmd_callback.p_func = p_func;
  return p_func != NULL ? IC_SUCCESS : IC_ERROR;
}

ic_return_val_e cmd_task_connect_to_rgb_led_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[RGB_LED_DESC]);
}

ic_return_val_e cmd_task_connect_to_power_led_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[POWER_LED_DESC]);
}

ic_return_val_e cmd_task_connect_to_vibrator_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[VIBRATOR_DESC]);
}

ic_return_val_e cmd_task_connect_to_pulseoximeter_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[PULSEOXIMETER_DESC]);
}

ic_return_val_e cmd_task_connect_to_e_alarm_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[E_ALARM_DESC]);
}

ic_return_val_e cmd_task_connect_to_device_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[DEVICE_DESC]);
}

ic_return_val_e cmd_task_connect_to_status_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[STATUS_DESC]);
}

ic_return_val_e cmd_task_connect_to_dfu_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[DFU_DESC]);
}

ic_return_val_e cmd_task_connect_to_feed_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[FEED_DESC]);
}

ic_return_val_e cmd_task_connect_to_unlock_cmd(void (*p_func)(u_BLECmdPayload*)){
  return m_cmd_task_connect_to_cmd(p_func, &m_cmd_list[UNLOCK_DESC]);
}

void cmd_main_task(void *args){
  UNUSED_PARAMETER(args);
  /*static u_cmdFrameContainer m_rsp_frame;*/
  /*memset(m_rsp_frame, 0, sizeof(m_rsp_frame));*/
  ble_iccs_connect_to_cmd(m_cmd_handle);

  for(;;){
    vTaskSuspend(NULL);
    /*memcpy(m_rsp_frame.frame, m_cmd_frame, sizeof(m_rsp_frame));*/
    for(uint8_t i = 0; i < NUM_OF_COMMANDS; i++){
      if(m_cmd_list[i].is_valid){
        NRF_LOG_INFO("Command: %d\n", m_cmd_list[i].cmd);
        m_cmd_list[i].cmd_callback.cmd_handle(&(m_cmd_list[i].cmd_frame.payload));
        m_cmd_list[i].is_valid = false;
      }
    }
  }
  vTaskDelete(NULL);
  taskYIELD();
}

void cmd_module_init(void){
  for(uint8_t i = 0; i < NUM_OF_COMMANDS; i++){
    memset(&(m_cmd_list[i].cmd_frame), 0, sizeof(s_frame));
    m_cmd_list[i].is_valid = false;
  }
  if(pdPASS != xTaskCreate(cmd_main_task, "CMD", 256, NULL, IC_FREERTOS_TASK_PRIORITY_LOW, &m_cmd_main_task_handle)){
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

void cmd_module_deinit(void){
  vTaskDelete(m_cmd_main_task_handle);
  taskYIELD();
}

static void m_cmd_handle(uint8_t *data, size_t len){
  if(len < sizeof(s_frame)){
    NRF_LOG_INFO("Uncomplete frame. Number of bytes: %d\n", len);
    return;
  }
  uint8_t i = 0;
  u_cmdFrameContainer *tmp_frame = (u_cmdFrameContainer*)data;
  /*memcpy(m_cmd_list[i].cmd_frame, data, sizeof(s_frame));*/
  for(i = 0; i < NUM_OF_COMMANDS; i++){
    if(tmp_frame->frame.cmd == m_cmd_list[i].cmd){
      memcpy(&(m_cmd_list[i].cmd_frame), data, sizeof(s_frame));
      m_cmd_list[i].is_valid = true;
      break;
    }
  }
  if(i == NUM_OF_COMMANDS){
    wrong_cmd_handle(tmp_frame->frame.cmd);
  }
  /*switch(((u_cmdFrameContainer)data).frame.cmd){*/
    /*case RGB_LED_CMD:*/
      /*rsp_code = rgb_cmd_handle(m_cmd_frame.payload.rgb_cmd);*/
      /*break;*/
    /*case POWER_LED_CMD:*/
      /*rsp_code = pwr_led_cmd_handle(m_cmd_frame.payload.power_cmd);*/
      /*break;*/
    /*case VIBRATOR_CMD:*/
      /*rsp_code = vib_cmd_handle(m_cmd_frame.payload.vibra_cmd);*/
      /*break;*/
    /*case PULSEOXIMETER_CMD:*/
      /*rsp_code = pox_cmd_handle(m_cmd_frame.payload.pox_cmd);*/
      /*break;*/
    /*case E_ALARM_CMD:*/
      /*rsp_code = alarm_cmd_handle(m_cmd_frame.payload.alarm_cmd);*/
      /*break;*/
    /*case DEVICE_CMD:*/
      /*rsp_code = dev_cmd_handle(m_cmd_frame.payload.device_cmd);*/
      /*break;*/
    /*case STATUS_CMD:*/
      /*rsp_code = status_cmd_handle(m_cmd_frame.payload.status_cmd);*/
      /*break;*/
    /*case DFU_CMD:*/
      /*rsp_code = dfu_cmd_handle(m_cmd_frame.payload.enable_dfu);*/
      /*break;*/
    /*case FEED_CMD:*/
      /*rsp_code = feed_cmd_handle(m_cmd_frame.payload.enable_feed);*/
      /*break;*/
    /*case UNLOCK_MASK:*/
      /*rsp_code = unlock_cmd_handle(m_cmd_frame.payload.unlock_mask);*/
      /*break;*/
    /*default:*/
      /*wrong_cmd_handle(m_cmd_frame.cmd);*/
      /*break;*/
  /*}*/
  xTaskResumeFromISR(m_cmd_main_task_handle);
  /*NRF_LOG_INFO("data:\n");*/
  /*for(uint8_t i = 0; i < len; i++)*/
    /*NRF_LOG_INFO(" %d\n", data[i]);*/
  /*NRF_LOG_INFO("size:%d\n", len);*/
}

static void default_cmd_handle(u_BLECmdPayload *cmd_payload){
  NRF_LOG_INFO("Payload:\n");
  for(uint8_t i = 0; i < sizeof(u_BLECmdPayload); i++)
    NRF_LOG_INFO(" %d\n", cmd_payload->data[i]);
}

static void wrong_cmd_handle(e_cmd cmd){
  NRF_LOG_INFO("Error: Wrong command: %d\n", cmd);
}
