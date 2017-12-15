/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup bootloader_secure main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_dfu.h"
#include "nrf_dfu_types.h"
#include "nrf_dfu_settings.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"

/*#include "ic_pwr_management.h"*/

extern nrf_dfu_settings_t s_dfu_settings;

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("received a fault! id: 0x%08x, pc: 0x&08x\r\n", id, pc);
    NVIC_SystemReset();
}

void app_error_handler_bare(uint32_t error_code)
{
    (void)error_code;
    NRF_LOG_ERROR("received an error: 0x%08x!\r\n", error_code);
    NVIC_SystemReset();
}

bool nrf_dfu_enter_check(void){
  if (s_dfu_settings.enter_buttonless_dfu == 1){
    s_dfu_settings.enter_buttonless_dfu = 0;
    APP_ERROR_CHECK(nrf_dfu_settings_write(NULL));
    return true;
  }
  return false;
}

/**@brief Function for application main entry.
 */
int main(void){
  /*sd_softdevice_vector_table_base_set(0x37C00);*/

  uint32_t ret_val;

  (void) NRF_LOG_INIT(NULL);

  NRF_LOG_INFO("Inside main\r\n");

  /*ic_power_up_all_systems();*/
  /*ic_boot_led_init();*/
  ret_val = nrf_bootloader_init();
  APP_ERROR_CHECK(ret_val);

  // Either there was no DFU functionality enabled in this project or the DFU module detected
  // no ongoing DFU operation and found a valid main application.
  // Boot the main application.
  /*ic_power_down_all_systems();*/
  nrf_bootloader_app_start(MAIN_APPLICATION_START_ADDR);

  // Should never be reached.
  NRF_LOG_INFO("After main\r\n");
}

/**
 * @}
 */
