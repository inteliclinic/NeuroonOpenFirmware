/**
 * @file    ic_driver_micron.c
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

#define NRF_LOG_MODULE_NAME "FLASH_MICRON"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_driver_spi.h"
#include "ic_driver_flash.h"
#include "ic_driver_flash_micron.h"
#include "ic_driver_flash_general.h"

#include "ic_config.h"

#define FLASH_TIMEOUT   3

  /* variables for receiving and sending data */
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
FlashReturnType MICRON_FlashReadDeviceIdentification(uint32_t *deviceID)
{
  SPI_INIT(flash_write, IC_SPI_FLASH_SS_PIN);

  m_data_to_send->inst = N25Q256A_READ_ID;
  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            m_output_buffer,
            FLASH_SEND_4BYTES,
            flash_drv_callback,
            NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }
    /*  block reading identification operation  */
  while(m_flash_semaphore == false);

  *deviceID = m_output_buffer[1];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[2];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[3];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashUninit()
{
  SPI_UNINIT(flash_write);

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashReadStatusRegister(uint8_t *status_reg)
{
  m_data_to_send->inst = N25Q256A_READ_STAT_REG;
  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            m_output_buffer,
            FLASH_SEND_2BYTES,
            flash_drv_callback,
            NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }
    /*  block reading identification operation  */
  while(m_flash_semaphore == false);

  *status_reg = m_output_buffer[1];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashWriteStatusRegister(uint8_t data_to_write, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _stat_reg = 0;
  fdo->GenOp.FlashReadStatusRegister(&_stat_reg);

  if (_stat_reg & MICRON_SR_SRP_BIT)
    return Flash_StatRegBlocked;

  fdo->GenOp.FlashWriteEnable(fdo);
  m_data_to_send->inst = N25Q256A_WRITE_STAT_REG;
  *m_data_to_send->address = data_to_write;

  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_2BYTES,
          NULL,
          NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashWriteEnable(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _status_reg = 0;
  size_t _timeout = 0;

  m_data_to_send->inst = N25Q256A_WRITE_EN;
  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            m_output_buffer,
            FLASH_SEND_1BYTE,
            flash_drv_callback,
            NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }

  while (m_flash_semaphore == false);

  do
  {
    fdo->GenOp.FlashReadStatusRegister(&_status_reg);
    _timeout++;
  } while ((!
      (_status_reg & MICRON_SR_WEL_BIT)) &&
      (_timeout < FLASH_TIMEOUT));

  if (_timeout == FLASH_TIMEOUT)
    return Flash_OperationTimeOut;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashWriteDisable()
{
  m_data_to_send->inst = N25Q256A_WRITE_DIS;

  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE,
          NULL,
          NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashPageProgram(
    uint32_t address,
    uint8_t *data,
    size_t len,
    ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Validate address input  */
  if(!(address < fdo->Desc.FlashSize))
    return Flash_AddressInvalid;

  if (len > FLASH_BUFFER_SIZE_MAX)
    return Flash_WrongDataSize;

  if (fdo->GenOp.FlashWriteEnable(fdo) != Flash_Success)
    return Flash_ProgramFailed;

  m_data_to_send->inst = N25Q256A_PAGE_PROG;
  convert_vector_addr(address, m_data_to_send->address, fdo->Desc.NumAddrByte);

    /*  It's necessary to divide spi transaction if
     *  user want to send more than 256 bytes of data
     *
     *  Please remember that first you have to send 4 bytes (instruction + address)
     *  and then you can push via spi up to 255 + 1 bytes of data due to nrf_spi_driver
     * */
  __auto_type _ret_val =
      SPI_SEND_DATA_OPEN(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE + fdo->Desc.NumAddrByte,
          NULL,
          NULL);
  if (len == FLASH_BUFFER_SIZE_MAX)
  {
    _ret_val =
        SPI_SEND_DATA_OPEN(
            flash_write,
            data,
            m_output_buffer,
            FLASH_BUFFER_SIZE_MAX - 1,
            NULL,
            NULL);
    _ret_val =
        SPI_SEND_DATA(
            flash_write,
            &data[FLASH_BUFFER_SIZE_MAX - 1],
            m_output_buffer,
            FLASH_SEND_1BYTE,
            flash_drv_callback,
            NULL);
  }
  else
    _ret_val =
        SPI_SEND_DATA(
            flash_write,
            data,
            m_output_buffer,
            len,
            flash_drv_callback,
            NULL);

  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashPageProgramSafety(
    uint32_t address,
    uint8_t *data,
    size_t len,
    ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  /*  Check whether any previous Write, Program or Erase cycle is on-going (2-step safety checking)  */
  if(IsFlashBusy())
    return Flash_OperationOngoing;
  else
  {
    MICRON_FlashPageProgram(address, data, len, fdo);
  }

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashGenWrite(
    uint16_t block_addr,
    uint8_t sector_addr,
    uint8_t page_addr,
		uint8_t *data_input,
		size_t len,
		ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /* Validate address input  */
  if(!(
      (block_addr < fdo->Desc.FlashBlockCount) &&
      (sector_addr < fdo->Desc.FlashSectorSize_bit) &&
      (page_addr < fdo->Desc.FlashPageSize_bit)
      ))
  {
    NRF_LOG_INFO("Invalid address\r\n");
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

  /*NRF_LOG_INFO("Writing all adrr: 0x%08X\r\n", all_addr);*/

  MICRON_FlashPageProgram(_all_addr, data_input, len, fdo);

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashGenWriteSafety(
    uint16_t block_addr,
    uint8_t sector_addr,
    uint8_t page_addr,
		uint8_t *data_input,
		size_t len,
		ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
     /* Check whether any previous Write, Program or Erase cycle is on-going  */
  if(IsFlashBusy())
    return Flash_OperationOngoing;
  else
    MICRON_FlashGenWrite(
        block_addr,
        sector_addr,
        page_addr,
        data_input,
        len,
        fdo);

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashDataRead(
    uint32_t address,
    uint8_t *data_output,
    size_t len,
    ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  m_data_to_send->inst = N25Q256A_READ;
  convert_vector_addr(address, m_data_to_send->address, fdo->Desc.NumAddrByte);

  __auto_type _ret_val =
      SPI_SEND_DATA_OPEN(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE + fdo->Desc.NumAddrByte,
          NULL,
          NULL);

  if (len == FLASH_BUFFER_SIZE_MAX)
  {
    _ret_val =
        SPI_SEND_DATA_OPEN(
            flash_write,
            m_input_buffer,
            data_output,
            FLASH_BUFFER_SIZE_MAX - 1,
            NULL,
            NULL);
    _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            &data_output[FLASH_BUFFER_SIZE_MAX - 1],
            FLASH_SEND_1BYTE,
            flash_drv_callback,
            NULL);
  }
  else
    _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            data_output,
            len,
            flash_drv_callback,
            NULL);

  if (_ret_val != IC_SUCCESS)
    return Flash_Error;


  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashSectorErase(uint32_t sector_address, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Validate the sector number input  */
  if(!(sector_address < fdo->Desc.FlashSubSectorCount))
    return Flash_SectorNrInvalid;

    /*  Disable Write protection  */
  if (fdo->GenOp.FlashWriteEnable(fdo) != Flash_Success)
    return Flash_ProgramFailed;

  m_data_to_send->inst = N25Q256A_SUBSECTOR_ERASE;
  sector_address <<= 12;
  convert_vector_addr(sector_address, m_data_to_send->address, fdo->Desc.NumAddrByte);
  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE + fdo->Desc.NumAddrByte,
          flash_drv_callback,
          NULL);

  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashBlockErase(uint16_t block_address, ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint32_t _bl_addr = 0;

  if (block_address >= fdo->Desc.FlashBlockCount)
    return Flash_WrongBlockNum;

  fdo->GenOp.FlashWriteEnable(fdo);

  m_data_to_send->inst = N25Q256A_SECTOR_ERASE;
  if (fdo->Desc.NumAddrByte == FLASH_4_BYTE_ADDR_MODE)
  {
    _bl_addr  = block_address << 24;
    _bl_addr |= block_address << 16;
  }
  if (fdo->Desc.NumAddrByte == FLASH_3_BYTE_ADDR_MODE)
  {
    _bl_addr = block_address << 16;
  }
  convert_vector_addr(_bl_addr, m_data_to_send->address, fdo->Desc.NumAddrByte);

  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          fdo->Desc.NumAddrByte,
          flash_drv_callback,
          NULL);

  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashChipErase(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
    /*  Enable WEL flag  */
  fdo->GenOp.FlashWriteEnable(fdo);
  NRF_LOG_INFO("Write latch enabled\r\n");

  m_data_to_send->inst = N25Q256A_BULK_ERASE;

  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE,
          NULL,
          NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashReadInformationReg(uint8_t *info_reg)
{
  m_data_to_send->inst = N25Q256A_READ_FLAG_STATUS_REG;

  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            m_output_buffer,
            FLASH_SEND_2BYTES,
            flash_drv_callback,
            NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }

  while(m_flash_semaphore == false);

  *info_reg = m_output_buffer[1];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashReadDeviceID(uint32_t *deviceID)
{
  m_data_to_send->inst =N25Q256A_READ_ID2;

  if (m_flash_semaphore != false)
  {
    m_flash_semaphore = false;
    __auto_type _ret_val =
        SPI_SEND_DATA(
            flash_write,
            m_input_buffer,
            m_output_buffer,
            FLASH_SEND_4BYTES,
            flash_drv_callback,
            NULL);
    if (_ret_val != IC_SUCCESS)
      return Flash_Error;
  }

  while(m_flash_semaphore == false);

  *deviceID = m_output_buffer[1];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[2];
  *deviceID <<= 8;
  *deviceID |= m_output_buffer[3];

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
//FlashReturnType MICRON_FlashEnterDeepPwrDown()
//{
//  __auto_type ret_val = SPI_SEND_DATA(flash_write, data_to_send, output_buffer, FLASH_SEND_1BYTE, callback_spi);
//  if (ret_val != IC_SUCCESS)
//    return Flash_Error;
////	NRF_LOG_INFO("%d\r\n", ret_val);
//
//  return Flash_Success;
//}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashEnter4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _info_reg = 0;

  m_data_to_send->inst = N25Q256A_ENTER_4BYTE_ADDR_MODE;

  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE,
          NULL,
          NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  do
  {
    fdo->GenOp.FlashReadInformationReg(&_info_reg);
  }	while (!(_info_reg & MICRON_IR_4BYTE_BIT));

  fdo->Desc.NumAddrByte = FLASH_4_BYTE_ADDR_MODE;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType MICRON_FlashExit4ByteMode(ic_flash_FLASH_DEVICE_OBJECT *fdo)
{
  uint8_t _info_reg = 0;

  m_data_to_send->inst = N25Q256A_EXIT_4BYTE_ADDR_MODE;

  __auto_type _ret_val =
      SPI_SEND_DATA(
          flash_write,
          m_input_buffer,
          m_output_buffer,
          FLASH_SEND_1BYTE,
          NULL,
          NULL);
  if (_ret_val != IC_SUCCESS)
    return Flash_Error;

  fdo->GenOp.FlashReadInformationReg(&_info_reg);

  if (_info_reg & MICRON_IR_4BYTE_BIT)
    return Flash_WrongType;

  fdo->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
