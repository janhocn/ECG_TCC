/**
 ******************************************************************************
 * @file    MCU_Flash_Map.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 January 2018
 * @brief   MCU Flash Module Address Map
 ******************************************************************************
 */

#ifndef APPLICATION_MICROCONTROLLER_FLASH_MCU_FLASH_MAP_H_
#define APPLICATION_MICROCONTROLLER_FLASH_MCU_FLASH_MAP_H_

#include "fsl_common.h"
#include "Messages.h"
#include "pin_mux.h"

/*************************************************************************************************/
/**
 * @attention flash start address will be located at the last sector of any MCU
 * So, the first address of data should be 0x000000
 */
#define FLASH_TOTAL_SIZE            0x00100000
#define FLASH_SECTOR_SIZE           0x1000
#define FLASH_USER_BASE_ADDR        (FLASH_TOTAL_SIZE-FLASH_SECTOR_SIZE)

#define FLASH_LANGUAGE_START_ADDR   0x000000
#define FLASH_LANGUAGE_SIZE         FSL_FEATURE_FLASH_PFLASH_RESOURCE_CMD_ADDRESS_ALIGMENT
#define FLASH_LANGUAGE_END_ADDR     (FLASH_LANGUAGE_START_ADDR + FLASH_LANGUAGE_SIZE)

#define FLASH_TOUCH_CAL_START_ADDR   FLASH_LANGUAGE_END_ADDR
#define FLASH_TOUCH_CAL_SIZE         FSL_FEATURE_FLASH_PFLASH_RESOURCE_CMD_ADDRESS_ALIGMENT
#define FLASH_TOUCH_CAL_END_ADDR     (FLASH_TOUCH_CAL_START_ADDR + FLASH_LANGUAGE_SIZE)

#endif /* APPLICATION_MICROCONTROLLER_FLASH_MCU_FLASH_MAP_H_ */
