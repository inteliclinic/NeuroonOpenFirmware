/**
 * @file    ic_driver_flash_eon.c
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */


#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ic_config.h"
#include <stdio.h>
#include <stdint.h>

#define NRF_LOG_MODULE_NAME "FLASH_EON"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_driver_spi.h"
#include "ic_driver_flash.h"
#include "ic_driver_flash_eon.h"
#include "ic_driver_flash_general.h"

#define FLASH_TIMEOUT	3

  /*  variables for receiving and sending data  */
//static char m_output_buffer [sizeof(s_spiSendToFlash)] = {0};
//static char m_input_buffer  [sizeof(s_spiSendToFlash)] = {0};
static char m_output_buffer [256] = {0};
static char m_input_buffer  [256] = {0};
  /*	create pointer for filling struct to send data	*/
static s_spiSendToFlash *m_data_to_send = (s_spiSendToFlash *)m_input_buffer;
  /*	convert given address to vector for filling struct	*/
void convert_vector_addr(uint32_t udAddr, uint8_t* pIns_Addr, size_t num_address_byte);
  /*	configure SPI REGISTER to handle the SPI interrupt	*/
SPI_REGISTER(flash_write);

static bool m_flash_semaphore = true;
static void flash_drv_callback()
{
    /*  release semaphore when callback from spi is returned  */
  m_flash_semaphore = true;
}

                 /**********************************************		EN25QH256 LOW LEVEL FUNCTIONS		**************************************************************/
FlashReturnType EON_FlashReadDeviceIdentification(uint32_t *deviceID)
{
  SPI_INIT(flash_write, IC_SPI_FLASH_SS_PIN);

  m_data_to_send->inst = EN25QH256_R_IDENTIFICATION;
  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_4BYTES, flash_drv_callback, NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }
  else
    return Flash_OperationOngoing;
//	NRF_LOG_INFO("%d\r\n", ret_val);
  while(m_flash_semaphore == false);

  *deviceID = m_output_buffer[1];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[2];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[3];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashUninit(void)
{
  SPI_UNINIT(flash_write);

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashReadStatusRegister(uint8_t *status_reg)
{
  m_data_to_send->inst = EN25QH256_R_STATREG_ADDR;

  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_2BYTES, flash_drv_callback, NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  #ifdef NRF_DEBUG
    NRF_LOG_INFO("%d\r\n", ret_val);
  #endif
  }
  else
    return Flash_OperationOngoing;

  while(m_flash_semaphore == false);

  *status_reg = m_output_buffer[1];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType  EON_FlashWriteStatusRegister(uint8_t data_to_write, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _stat_reg = 0;
  fdo->GenOp.FlashReadStatusRegister(&_stat_reg);

  if (_stat_reg & EON_SR_SRP_BIT)
  {
#ifdef FLASH_DEBUG
    printf("Stat reg blocked\r\n");
#endif
    return Flash_StatRegBlocked;
 }

  fdo->GenOp.FlashWriteEnable(fdo);
  m_data_to_send->inst = EN25QH256_W_STATREG_ADDR;
  *m_data_to_send->address = data_to_write;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_2BYTES, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashWriteEnable(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _status_reg;
  size_t _timeout = 0;

  m_data_to_send->inst = EN25QH256_W_EN_ADDR;

  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, flash_drv_callback, NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }
  else
    return Flash_OperationOngoing;

  while(m_flash_semaphore == false);
  do
  {
    fdo->GenOp.FlashReadStatusRegister(&_status_reg);
    _timeout++;
  } while ((!(_status_reg & EON_SR_WEL_BIT) && _timeout < FLASH_TIMEOUT));

  if (_timeout == FLASH_TIMEOUT)
    return Flash_OperationTimeOut;

  return Flash_Write_Enabled;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashWriteDisable()
{
  m_data_to_send->inst = EN25QH256_W_DIS_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;
#ifdef NRF_DEBUG
  NRF_LOG_INFO("%d\r\n", ret_val);
#endif
  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashPageProgram(uint32_t address, uint8_t *data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /* Validate address input  */
  if(!(address < fdo->Desc.FlashSize))
    return Flash_AddressInvalid;

  if (len > FLASH_BUFFER_SIZE_MAX)
    return Flash_WrongDataSize;

  if (fdo->GenOp.FlashWriteEnable(fdo) != Flash_Write_Enabled)
  {
    return Flash_ProgramFailed;
  }

  m_data_to_send->inst = EN25QH256_PAGE_PROGRAM_ADDR;
  convert_vector_addr(address, m_data_to_send->address, fdo->Desc.NumAddrByte);

  __auto_type _ret_val = SPI_SEND_DATA_OPEN(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE + fdo->Desc.NumAddrByte, NULL, NULL);
  if (len == FLASH_BUFFER_SIZE_MAX)
  {
    _ret_val = SPI_SEND_DATA_OPEN(flash_write, data, NULL, FLASH_BUFFER_SIZE_MAX - 1, NULL, NULL);
    _ret_val = SPI_SEND_DATA(flash_write, &data[FLASH_BUFFER_SIZE_MAX - 1], NULL, FLASH_SEND_1BYTE, flash_drv_callback, NULL);
  }
  else
    _ret_val = SPI_SEND_DATA(flash_write, data, NULL, len, flash_drv_callback, NULL);

  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashPageProgramSafety(uint32_t address, uint8_t *data, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Check whether any previous Write, Program or Erase cycle is on-going (2-step safety checking)  */
  if(IsFlashBusy())
    return Flash_OperationOngoing;
  else
  {
    EON_FlashPageProgram(address, data, len, fdo);
  }

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashGenWrite(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr,
														uint8_t *data_input, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Validate address input  */
  if(!((block_addr < fdo->Desc.FlashBlockCount ) && (sector_addr < fdo->Desc.FlashSectorSize_bit ) && (page_addr < fdo->Desc.FlashPageSize_bit )))
  {
#ifdef NRF_DEBUG
    NRF_LOG_INFO("Invalid address\r\n");
#endif
    return Flash_AddressInvalid;
  }

  uint32_t _all_addr = 0;
    /*  when chosen 3-byte addressing mode -> 256 blocks (1byte) + [16 sectors + 16 pages] (1byte)  */
  if (fdo->Desc.NumAddrByte == FLASH_3_BYTE_ADDR_MODE)
  {
    _all_addr  = block_addr   << 16;
    _all_addr |= sector_addr  << 12;
    _all_addr |= page_addr    << 8;
  }
  if (fdo->Desc.NumAddrByte == FLASH_4_BYTE_ADDR_MODE)
  {
    _all_addr  = block_addr   << 24;
    _all_addr |= block_addr   << 16;
    _all_addr |= sector_addr  << 12;
    _all_addr |= page_addr    << 8;
  }

  if (EON_FlashPageProgram(_all_addr, data_input, len, fdo) != Flash_Success)
    return Flash_ProgramFailed;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashGenWriteSafety(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr,
																	uint8_t *data_input, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Check whether any previous Write, Program or Erase cycle is on-going  */
  if(IsFlashBusy())
    return Flash_OperationOngoing;
  else
    EON_FlashGenWrite(block_addr, sector_addr, page_addr, data_input, len, fdo);

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashDataRead(uint32_t address, uint8_t *data_output, size_t len, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  m_data_to_send->inst = EN25QH256_R_DATA_ADDR;
  convert_vector_addr(address, m_data_to_send->address, fdo->Desc.NumAddrByte);

  __auto_type _ret_val = SPI_SEND_DATA_OPEN(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE + fdo->Desc.NumAddrByte, NULL, NULL);
  if (len == FLASH_BUFFER_SIZE_MAX)
  {
    _ret_val = SPI_SEND_DATA_OPEN(flash_write, NULL, data_output, FLASH_BUFFER_SIZE_MAX - 1, NULL, NULL);
    _ret_val = SPI_SEND_DATA(flash_write, NULL, &data_output[FLASH_BUFFER_SIZE_MAX - 1], FLASH_SEND_1BYTE, flash_drv_callback, NULL);
  }
  else
    _ret_val = SPI_SEND_DATA(flash_write, NULL, data_output, len, flash_drv_callback, NULL);

  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

#ifdef FLASH_NRF_DEBUG
  NRF_LOG_INFO("output data: %s\r\n", nrf_log_push((char*)data_output));
  NRF_LOG_INFO("output data:\r\n %s\r\n", ((char*)data_output));
#endif

	return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashSectorErase(uint32_t sector_address, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Validate the sector number input  */
  if(!(sector_address < fdo->Desc.FlashSubSectorCount))
  {
#ifdef FLASH_NRF_DEBUG
      NRF_LOG_INFO("Wrong sector number\r\n");
#endif
    return Flash_SectorNrInvalid;
  }

    /*  Disable Write protection  */
  if (fdo->GenOp.FlashWriteEnable(fdo) != Flash_Write_Enabled)
  {
    return Flash_ProgramFailed;
  }
#ifdef FLASH_DEBUG
  printf("Write enabled!!!\r\n");
#endif

  m_data_to_send->inst = EN25QH256_SECTOR_ERASE_ADDR;
  sector_address <<= 12;
  convert_vector_addr(sector_address, m_data_to_send->address, fdo->Desc.NumAddrByte);

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE + fdo->Desc.NumAddrByte, flash_drv_callback, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashBlockErase(uint16_t block_address, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint32_t _bl_addr = 0;
#ifdef FLASH_DEBUG
  printf("Block erase\r\n");
#endif

  if (block_address >= fdo->Desc.FlashBlockCount)
    return Flash_WrongBlockNum;

  fdo->GenOp.FlashWriteEnable(fdo);
  m_data_to_send->inst = EN25QH256_BLOCK_ERASE_ADDR;
  if (fdo->Desc.NumAddrByte == FLASH_4_BYTE_ADDR_MODE)
  {
    _bl_addr  = block_address << 24;
    _bl_addr |= block_address << 16;
  }
	if (fdo->Desc.NumAddrByte == FLASH_3_BYTE_ADDR_MODE)
	  _bl_addr = block_address << 16;

	convert_vector_addr(_bl_addr, m_data_to_send->address, fdo->Desc.NumAddrByte);

	__auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, fdo->Desc.NumAddrByte, flash_drv_callback, NULL);
	if (_ret_val != IC_SUCCESS)
	  return Flash_Error;

	return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashChipErase(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Enable flash write  */
  fdo->GenOp.FlashWriteEnable(fdo);

  m_data_to_send->inst = EN25QH256_CHIP_ERASE_ADDR;
  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashReadInformationReg(uint8_t *info_reg)
{
  m_data_to_send->inst = EN25QH256_R_INFOREG_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_2BYTES, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  *info_reg = m_output_buffer[1];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashReadDeviceID(uint32_t *deviceID)
{
  m_data_to_send->inst =EN25QH256_R_DEVICE_ID_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_4BYTES, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  *deviceID = m_output_buffer[1];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[2];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[3];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashEnterDeepPwrDown(void)
{
  m_data_to_send->inst = EN25QH256_DEPP_PWR_DOWN_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  /*
   * You should wait here for about 3 us
   */

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashExitDeepPwrDown(void)
{
  m_data_to_send->inst = EN25QH256_RELEASE_DEEP_PWR_DOWN_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  /*
   * You should wait here for about 3 us
   */

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashEnter4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _info_reg = 0;

  m_data_to_send->inst = EN25QH256_ENTER_4BYTE_MODE_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  do
  {
    fdo->GenOp.FlashReadInformationReg(&_info_reg);
    NRF_LOG_INFO("Info reg: 0x%02X\r\n", _info_reg);
  }	while (!(_info_reg & EON_IR_4BYTE_BIT));

  fdo->Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType EON_FlashExit4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _info_reg = 0;

  m_data_to_send->inst = EN25QH256_EXIT_4BYTE_MODE_ADDR;

  __auto_type _ret_val = SPI_SEND_DATA(flash_write, m_input_buffer, m_output_buffer, FLASH_SEND_1BYTE, NULL, NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  fdo->GenOp.FlashReadInformationReg(&_info_reg);
  if (_info_reg & EON_IR_4BYTE_BIT)
    return Flash_WrongType;

  fdo->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
