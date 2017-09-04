/**
 * @file    ic_flash_filesystem.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_FLASH_FILESYSTEM
#define _IC_FLASH_FILESYSTEM

#include <stdint.h>
#include <stdio.h>

#include "ic_driver_flash.h"

typedef void (*func_cb)(ic_flash_state *flash_state);

typedef enum __packed
{
	IC_FILE_EMPTY = 0x00,
	IC_FILE_W,
	IC_FILE_RO
} e_source_flag;

/**
 *
 */
typedef struct __attribute__((packed))
{
  uint32_t start_addr;        // 4 bytes
  uint32_t end_addr;          // 4 bytes
	e_source_flag source_flag;  // 1 byte
	char source_name[20];       // 20 bytes
}s_source_info;

/**
 *
 */
typedef struct __attribute__((packed))
{
	uint32_t neuroon_sig;           // 4 bytes
	uint32_t erasal_num;            // 4 bytes
	s_source_info source_info[5];   // 29 * 5 = 145 bytes
	uint8_t num_of_files;           // 1 bytes
	uint8_t not_used[102];          // 102 bytes
}s_mbr_info;

typedef enum  __attribute__((packed))
{
	IC_FILE_SUCCESS = 0x00,
	IC_FILE_CREATED,
	IC_FILE_ALREADY_EXISTS,
	IC_FILE_NOT_CLOSED,
	IC_NOFILES,
	IC_FILE_ERROR
} ic_filesys_return_type;


ic_filesys_return_type ic_open_source(s_mbr_info *get_source);
ic_filesys_return_type ic_mbr_write(s_mbr_info *source_p);
ic_filesys_return_type ic_mbr_read(uint8_t *mbr_buff);
ic_filesys_return_type ic_mbr_format(s_mbr_info *source_p, bool flag);
ic_filesys_return_type ic_create_file(s_mbr_info *file, char *filename, size_t length);
ic_filesys_return_type ic_read_file(s_mbr_info *file, char *filename, uint8_t *data);
ic_filesys_return_type ic_write_file(s_mbr_info *file, char *filename, uint8_t *data);
ic_filesys_return_type ic_open_file(s_mbr_info *file, char *filename, e_source_flag flag);
ic_filesys_return_type ic_close_file(s_mbr_info *file, char *filename);



#endif // !_IC_FLASH_FILESYSTEM
