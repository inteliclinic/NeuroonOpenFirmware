/**
 * @file    ic_flash_address_map.h
 * @Author  Kacper Gracki <k.gracki@inteliclinic.com>
 * @date    July, 2017
 * @brief   Brief description
 *
 * Description
 */

#ifndef _IC_FLASH_ADDRESS_MAP
#define _IC_FLASH_ADDRESS_MAP

#define TEST_ADDR_SIGNATURE             0xFACE00
#define FLASH_MBR_SECTOR_START          0x00
#define FLASH_MBR_SECTOR                0x00
#define FLASH_NEUROON_SIG_OFFSET        0x000		/*  signature max 4bytes (uint32_t)  */
#define FLASH_ERASAL_NUM_OFFSET         0x004		/*  max number of erase operation is 4bytes (uint32_t)  */
#define FLASH_SOURCE_START_ADDR_OFFSET  0x008		/*  max source name is limited to 20 bytes (+ 8bytes of start and end address) and max number of source names is 5 (28 * 5 = 140 bytes)  */
#define FLASH_SOURCE_END_ADDR_OFFSET    0x00C
#define FLASH_SOURCE_NAME_OFFSET        0x010
#define FLASH_SOURCE_ATTRIBUTE_OFFSET   0x011
#define FLASH_SOURCE_NAME_LENGTH        20
#define FLASH_SOURCE_STRUCT_LEN         29
#define FLASH_SOURCE_NUM_OFFSET         0x099		/*  1 byte holding number of source files  */
#define FLASH_SOMETHING_ELSE            0x08D
#define FLASH_MBR_SECTOR_END            0xFFF

#define FLASH_STARTING_ADDRESS          0x1000



#endif // !_IC_FLASH_ADDRESS_MAP
