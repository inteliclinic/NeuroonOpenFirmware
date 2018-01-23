/**
 * @file    ic_driver_flash.c
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

#define NRF_LOG_MODULE_NAME "FLASH_DRIVER"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ic_driver_spi.h"
#include "ic_driver_flash.h"
#include "ic_driver_flash_eon.h"
#include "ic_driver_flash_micron.h"
#include "ic_driver_flash_general.h"
#include "ic_flash_filesystem.h"


  /* static pointer to flash device object */
static ic_flash_FLASH_DEVICE_OBJECT *m_flash_object;

/**********************************************************************************************************************************************************/
FlashReturnType init_flash_driver(ic_flash_FLASH_DEVICE_OBJECT *flash_device_object)
{
  uint32_t _device_ID = 0;
  m_flash_object = flash_device_object;

  __auto_type _ret = MICRON_FlashReadDeviceIdentification(&_device_ID);
              /*_ret = EON_FlashReadDeviceIdentification(&_device_ID);*/

  if(_ret == Flash_Success)
    m_flash_object->Desc.FlashID = _device_ID;

  	/* EN25QH256 */
  if (_device_ID == MEM_TYPE_EN25Q256)
  {
      /* device shape */
    m_flash_object->Desc.FlashSize               = 0x2000000;
    m_flash_object->Desc.FlashBlockCount         = 0x200;
    m_flash_object->Desc.FlashSectorSize         = 0x10000;
    m_flash_object->Desc.FlashSectorSize_bit     = 16;
    m_flash_object->Desc.FlashSubSectorCount     = 0x2000;
    m_flash_object->Desc.FlashSubSectorSize      = 0x1000;
    m_flash_object->Desc.FlashSubSectorSize_bit  = 12;
    m_flash_object->Desc.FlashPageCount          = 0x20000;
    m_flash_object->Desc.FlashPageSize           = 0x100;
    m_flash_object->Desc.FlashPageSize_bit       = 16;
    m_flash_object->Desc.FlashOTPSize            = 0x40;
    m_flash_object->Desc.FlashAddressMask        = 0x00FF;
    	/*	3-addr-byte is default startup address mode	*/
    m_flash_object->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
      /* device operation for EN25QH256 flash*/

    m_flash_object->GenOp.FlashReadDeviceIdentification  = EON_FlashReadDeviceIdentification;
    m_flash_object->GenOp.FlashReadDeviceID              = EON_FlashReadDeviceID;
    m_flash_object->GenOp.FlashReadStatusRegister        = EON_FlashReadStatusRegister;
    m_flash_object->GenOp.FlashDataRead                  = EON_FlashDataRead;
    m_flash_object->GenOp.FlashWriteEnable               = EON_FlashWriteEnable;
    m_flash_object->GenOp.FlashWriteDisable              = EON_FlashWriteDisable;
    m_flash_object->GenOp.FlashPageProgram               = EON_FlashPageProgram;
    m_flash_object->GenOp.FlashGenWrite                  = EON_FlashGenWrite;
    m_flash_object->GenOp.FlashSectorErase               = EON_FlashSectorErase;
    m_flash_object->GenOp.FlashBlockErase                = EON_FlashBlockErase;
    m_flash_object->GenOp.FlashChipErase                 = EON_FlashChipErase;
    m_flash_object->GenOp.FlashReadInformationReg        = EON_FlashReadInformationReg;
    m_flash_object->GenOp.FlashEnter4ByteMode            = EON_FlashEnter4ByteMode;
    m_flash_object->GenOp.FlashExit4ByteMode             = EON_FlashExit4ByteMode;
    m_flash_object->GenOp.FlashEnterDeepPwrDown          = EON_FlashEnterDeepPwrDown;
    m_flash_object->GenOp.FlashExitDeepPwrDown           = EON_FlashExitDeepPwrDown;

    /**
     * if more functions were implemented, assigns them to GenOp pointer (here and in ic_flash_general header file)
     **/

    return Flash_Init_Success;

  } else if (_device_ID == MEM_TYPE_N25Q256A)
  						/* N25Q256A	MICRON */
	{
		  /* device shape */
		m_flash_object->Desc.FlashSize               = 0x2000000;
		m_flash_object->Desc.FlashBlockCount         = 0x200;
		m_flash_object->Desc.FlashSectorSize         = 0x10000;
		m_flash_object->Desc.FlashSectorSize_bit     = 16;
		m_flash_object->Desc.FlashSubSectorCount     = 0x2000;
		m_flash_object->Desc.FlashSubSectorSize      = 0x1000;
		m_flash_object->Desc.FlashSubSectorSize_bit  = 12;
		m_flash_object->Desc.FlashPageCount          = 0x20000;
		m_flash_object->Desc.FlashPageSize           = 0x100;
		m_flash_object->Desc.FlashPageSize_bit       = 16;
		m_flash_object->Desc.FlashOTPSize            = 0x40;
		m_flash_object->Desc.FlashAddressMask        = 0x00FF;
      /* 3-addr-byte is default startup address mode, except if you use
       * NVConfig addr mode setting (please see datasheet for more details)
       */
		m_flash_object->Desc.NumAddrByte = FLASH_3_BYTE_ADDR_MODE;
		  /* device operation */
		m_flash_object->GenOp.FlashReadDeviceIdentification  = MICRON_FlashReadDeviceIdentification;
		m_flash_object->GenOp.FlashReadDeviceID              = MICRON_FlashReadDeviceID;
		m_flash_object->GenOp.FlashReadStatusRegister        = MICRON_FlashReadStatusRegister;
		m_flash_object->GenOp.FlashDataRead                  = MICRON_FlashDataRead;
		m_flash_object->GenOp.FlashWriteEnable               = MICRON_FlashWriteEnable;
		m_flash_object->GenOp.FlashWriteDisable              = MICRON_FlashWriteDisable;
		m_flash_object->GenOp.FlashPageProgram               = MICRON_FlashPageProgram;
		m_flash_object->GenOp.FlashGenWrite                  = MICRON_FlashGenWrite;
		m_flash_object->GenOp.FlashSectorErase               = MICRON_FlashSectorErase;
		m_flash_object->GenOp.FlashBlockErase                = MICRON_FlashBlockErase;
		m_flash_object->GenOp.FlashChipErase                 = MICRON_FlashChipErase;
		m_flash_object->GenOp.FlashReadInformationReg        = MICRON_FlashReadInformationReg;
		m_flash_object->GenOp.FlashEnter4ByteMode            = MICRON_FlashEnter4ByteMode;
		m_flash_object->GenOp.FlashExit4ByteMode             = MICRON_FlashExit4ByteMode;
      /**
       * if more functions were implemented, assigns them to GenOp pointer (here and in ic_flash_general header file)
       **/
		return Flash_Init_Success;
	}
  else if (_device_ID == 0)
  	return Flash_WrongType;
  else
    return Flash_Init_Failed;
}
/**********************************************************************************************************************************************************/
FlashReturnType write_to_flash(uint32_t addr, uint8_t *data_to_write, size_t len,
		ic_flash_state *flash_state, func_finished cb)
{
  if (flash_state->state != nop)
    return Flash_OperationOngoing;
  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashPageProgram(addr, data_to_write, len, m_flash_object);
  else
    return Flash_Error;
  flash_state->state = write_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType write_to_flash_specific(uint16_t block_addr,uint8_t sector_addr, uint8_t page_addr, uint8_t *data_to_write, size_t len,
		ic_flash_state *flash_state, func_finished cb)
{
  if (flash_state->state != nop)
    return Flash_OperationOngoing;
  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashGenWrite(block_addr, sector_addr, page_addr, data_to_write, len, m_flash_object);
  else
    return Flash_Error;
  flash_state->state = write_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType read_from_flash(uint32_t address, uint8_t *data_output, size_t len,
		ic_flash_state *flash_state, func_finished cb)
{
  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashDataRead(address, data_output, len, m_flash_object);
  else
    return Flash_Error;
  flash_state->state = read_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType read_from_flash_specific(uint16_t block_addr, uint8_t sector_addr, uint8_t page_addr, uint8_t *data_output, size_t len,
		ic_flash_state *flash_state, func_finished cb)
{
  uint32_t _all_addr = 0;

  if (m_flash_object->Desc.NumAddrByte == FLASH_3_BYTE_ADDR_MODE)
  {
    _all_addr  = block_addr  << 16;
    _all_addr |= sector_addr << 12;
    _all_addr |= page_addr	 << 8;
  }
  if (m_flash_object->Desc.NumAddrByte == FLASH_4_BYTE_ADDR_MODE)
  {
    _all_addr  = block_addr  << 24;
    _all_addr |= block_addr  << 16;
    _all_addr |= sector_addr << 12;
    _all_addr |= page_addr   << 8;
  }

  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashDataRead(_all_addr, data_output, len, m_flash_object);
  else
    return Flash_Error;
  flash_state->state = read_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType erase_chip (ic_flash_state *flash_state, func_finished cb)
{
  if (flash_state->state != nop)
    return Flash_OperationOngoing;
  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashChipErase(m_flash_object);
  else
    return Flash_Error;
  flash_state->state = erase_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType erase_sector_from_flash(uint32_t address, ic_flash_state *flash_state, func_finished cb)
{
  if (flash_state->state != nop)
    return Flash_OperationOngoing;
  if (m_flash_object != NULL)
    if (m_flash_object->GenOp.FlashSectorErase(address, m_flash_object) != Flash_Success)
      return Flash_PageEraseFailed;

  flash_state->state = erase_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType erase_block_from_flash(uint16_t block_address, ic_flash_state *flash_state, func_finished cb)
{
  if (flash_state->state != nop)
    return Flash_OperationOngoing;

  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashBlockErase(block_address, m_flash_object);
  else
    return Flash_Error;
  flash_state->state = erase_state;
  flash_state->callback = cb;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
FlashReturnType deinit_flash()
{
  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashUninit();
  else
    return Flash_Error;

  return Flash_Success;
}
/**********************************************************************************************************************************************************/
bool IsFlashBusy(void)
{
  uint8_t _status_reg = 0;
  if (m_flash_object != NULL)
    m_flash_object->GenOp.FlashReadStatusRegister(&_status_reg);
  if(_status_reg & EON_SR_WIP_BIT)
    return true;

  return false;
}
/**********************************************************************************************************************************************************/
void convert_vector_addr(uint32_t data, uint8_t *vec_data, size_t num_address_byte)
{
    /* 3-addr byte mode */
  if(num_address_byte == FLASH_3_BYTE_ADDR_MODE)
  {
    vec_data[0]  = data >> 16;
    vec_data[1]  = data >> 12;
    vec_data[1]  = data >> 8;
    vec_data[2]  = data;
  }

    /* 4-addr byte mode */
  if(num_address_byte == FLASH_4_BYTE_ADDR_MODE)
  {
    vec_data[0]  = data >> 24;
    vec_data[1]  = data >> 16;
    vec_data[2]  = data >> 12;
    vec_data[2]  = data >> 8;
    vec_data[3]  = data;
  }
}
/**********************************************************************************************************************************************************/
