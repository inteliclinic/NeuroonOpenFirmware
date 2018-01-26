/**
 * @file    ic_flash_filesystem.c
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#include "ic_flash_filesystem.h"
#include "ic_flash_address_map.h"
#include "ic_service_flash.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nrf_log.h"

#define MAX_FILES_NUM		      6
#define FILE_WRITE_PACK_SIZE  256
#define OFFSET_BETWEEN_FILES	0x100

static uint32_t temp_file_addr = FLASH_STARTING_ADDRESS;
static volatile uint16_t temp_mbr_addr = 0;
uint8_t output_buffer[256];

/*
void callback_filesys(ic_flash_state *m_flash_state)
{
  NRF_LOG_INFO("{ %s }\r\n", (uint32_t)__func__);
  m_flash_state->state = nop;
}
*/
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_open_source(s_mbr_info *get_source)
{
  /*NRF_LOG_INFO("MBR temp1: %d\r\n", temp_mbr_addr);*/
	ic_mbr_read(output_buffer);
	memcpy(&get_source->neuroon_sig, &output_buffer[FLASH_MBR_SECTOR_START], sizeof(uint32_t));
	memcpy(&get_source->erasal_num, &output_buffer[FLASH_ERASAL_NUM_OFFSET], sizeof(uint32_t));
	memcpy(&get_source->num_of_files, &output_buffer[FLASH_SOURCE_NUM_OFFSET], sizeof(uint32_t));
	if (get_source->num_of_files != 0 && get_source->num_of_files < MAX_FILES_NUM)
	{
		for (int i=0; i < get_source->num_of_files; i++)
		{
			memcpy(&get_source->source_info[i].start_addr, &output_buffer[FLASH_SOURCE_START_ADDR_OFFSET + ( i * FLASH_SOURCE_STRUCT_LEN )], sizeof(uint32_t));
			memcpy(&get_source->source_info[i].end_addr, &output_buffer[FLASH_SOURCE_END_ADDR_OFFSET + ( i * FLASH_SOURCE_STRUCT_LEN )], sizeof(uint32_t));
			get_source->source_info[i].source_flag = output_buffer[FLASH_SOURCE_NAME_OFFSET + ( i * FLASH_SOURCE_STRUCT_LEN )];
			memcpy(&get_source->source_info[i].source_name, &output_buffer[FLASH_SOURCE_ATTRIBUTE_OFFSET + ( i * FLASH_SOURCE_STRUCT_LEN )], FLASH_SOURCE_NAME_LENGTH);
		}
	}
	else
	{
		NRF_LOG_INFO("No files!\r\n");

		return IC_FILE_EMPTY;
	}

	return IC_FILE_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_mbr_write(s_mbr_info *source_p)
{
  if (temp_mbr_addr == 0 || temp_mbr_addr == 4096)
  {
    temp_mbr_addr = 0;
    if (ic_flash_sector_erase(FLASH_MBR_SECTOR, flash_service_cb) != IC_SUCCESS)
      return IC_FILE_ERROR;
  }
//  temp_mbr_addr += 256;
  /*NRF_LOG_INFO("MBR temp2: %d\r\n", temp_mbr_addr);*/
  if (ic_flash_write(temp_mbr_addr, (uint8_t*)source_p, sizeof(s_mbr_info), flash_service_cb) != IC_SUCCESS)
    return IC_FILE_ERROR;

	return IC_FILE_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_mbr_read(uint8_t *mbr_buff)
{
  /*NRF_LOG_INFO("MBR temp2: %d\r\n", temp_mbr_addr);*/
  if (ic_flash_read(temp_mbr_addr, mbr_buff, FILE_WRITE_PACK_SIZE, flash_service_cb) != IC_SUCCESS)
    return IC_FILE_ERROR;

  return IC_FILE_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_mbr_format(s_mbr_info *source_p, bool first_use)
{
  if (first_use == false)
  {
    ic_flash_read(0x00, output_buffer, FILE_WRITE_PACK_SIZE, flash_service_cb);
//	memcpy(&source_p->neuroon_sig, &output_buffer[0], sizeof(uint32_t));
    memcpy(&source_p->erasal_num, &output_buffer[FLASH_ERASAL_NUM_OFFSET], sizeof(uint32_t));
  }
  else
  {
    if (ic_flash_sector_erase(0, flash_service_cb) == IC_SUCCESS)
    {
      source_p->erasal_num = 0;
      source_p->num_of_files = 0;
      source_p->neuroon_sig = 0x1234;
    }
    else
      return IC_FILE_ERROR;
  }
  if (ic_mbr_write(source_p) != IC_FILE_SUCCESS)
    return IC_FILE_ERROR;

  return IC_FILE_SUCCESS;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_create_file(s_mbr_info *file, char *filename, size_t length)
{
	if ((file->num_of_files > 0) && (file->num_of_files < MAX_FILES_NUM - 1))
	{
		size_t i = 0;
		for (i = 0; i < file->num_of_files; i++)
		{
			if (strcmp(file->source_info[i].source_name, filename) == 0)
			{
				NRF_LOG_INFO("File already exists\r\n");

				return IC_FILE_ALREADY_EXISTS;
			}
		}
		if (file->source_info[i-1].source_flag == IC_FILE_RO)
		{
			memcpy(file->source_info[i].source_name, filename, length);
			file->source_info[i].start_addr = OFFSET_BETWEEN_FILES + file->source_info[i-1].end_addr;
			temp_file_addr = file->source_info[i].start_addr;
			++(file->num_of_files);

			NRF_LOG_INFO("File created\r\n");
			return IC_FILE_CREATED;
		}
		else
		{
		  NRF_LOG_INFO("Previous file is not closed!!!\r\n");
			return IC_FILE_NOT_CLOSED;
		}
	}
	else
	{
		memcpy(file->source_info[0].source_name, filename, length);
		file->source_info[0].start_addr = FLASH_STARTING_ADDRESS;
		temp_file_addr = file->source_info[0].start_addr;
		++(file->num_of_files);
//		ic_mbr_write(file);

		NRF_LOG_INFO("File created\r\n");
		return IC_FILE_CREATED;
	}
	NRF_LOG_INFO("File not created\r\n");
	return IC_FILE_ERROR;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_read_file(s_mbr_info *file, char *filename, uint8_t *data)
{
	for (int i = 0; i < file->num_of_files; i++)
	{
		if (strcmp(file->source_info[i].source_name, filename) == 0)
		{
			if (file->source_info[i].source_flag == IC_FILE_RO)
			{
				for (int j = 0; j < (file->source_info[i].end_addr - file->source_info[i].start_addr) / FILE_WRITE_PACK_SIZE; j++)
				{
				  if (data != NULL)
				  {
				    ic_flash_read((file->source_info[j].start_addr * ( j + 1 )), data, FILE_WRITE_PACK_SIZE, flash_service_cb);
				    data += FILE_WRITE_PACK_SIZE;
				  }
				  else
				    return IC_FILE_ERROR;

				}
        return IC_FILE_SUCCESS;
			}
		}
	}
	NRF_LOG_INFO("File not found\r\n");

	return IC_FILE_ERROR;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_write_file(s_mbr_info *file, char *filename, uint8_t *data)
{
	for (int i = 0; i < file->num_of_files; i++)
	{
		if (strcmp(file->source_info[i].source_name, filename) == 0)
		{
			if (file->source_info[i].source_flag == IC_FILE_W)
			{
			  uint8_t *_temp_data = 0;
			  memcpy(_temp_data, data, 256);
        ic_flash_write(temp_file_addr, _temp_data, FILE_WRITE_PACK_SIZE, flash_service_cb);
				temp_file_addr += FILE_WRITE_PACK_SIZE;

				return IC_FILE_SUCCESS;
			}
			else
			  NRF_LOG_INFO("File is not in IC_FILE_W mode\r\n");
		}
	}
	return IC_FILE_ERROR;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_open_file(s_mbr_info *file, char *filename, e_source_flag flag)
{
	for (int i = 0; i < file->num_of_files; i++)
  {
  	if ((strcmp(file->source_info[i].source_name, filename) == 0))
  	{
  		file->source_info[i].source_flag = flag;
//  		ic_mbr_write(file);

  		return IC_FILE_SUCCESS;
  	}
  }
	return IC_FILE_ERROR;
}
/******************************************************************************************************************************************************************/
ic_filesys_return_type ic_close_file(s_mbr_info *file, char *filename)
{
	for (int i = 0; i < file->num_of_files; i++)
  {
  	if (strcmp(file->source_info[i].source_name, filename) == 0 && (file->source_info[i].source_flag != IC_FILE_RO))
  	{
  		file->source_info[i].source_flag = IC_FILE_RO;
  		file->source_info[i].end_addr = temp_file_addr;
  		temp_file_addr = 0;
  		NRF_LOG_INFO("File closed\r\n");
  		ic_mbr_write(file);
  		return IC_FILE_SUCCESS;
  	}
  }
	NRF_LOG_INFO("File does not exist\r\n");
	ic_mbr_write(file);
	return IC_FILE_NOT_CLOSED;
}
