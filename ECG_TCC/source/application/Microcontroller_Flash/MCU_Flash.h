/**
 ******************************************************************************
 * @file    MCU_Flash.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 January 2018
 * @brief   MCU Flash Module
 ******************************************************************************
 */

#ifndef APPLICATION_MICROCONTROLLER_FLASH_MCU_FLASH_H_
#define APPLICATION_MICROCONTROLLER_FLASH_MCU_FLASH_H_

#include "MCU_Flash_Map.h"

/*************************************************************************************************/
/**
 * @enum Parameters to use user data
 */
typedef enum _eflashUserData {
	_FLASH_USER_LANGUAGE, /** @< Language data (See the proper data type) */
	_FLASH_USER_TOUCH_CAL, /** @< Touch calibration data (See the proper data type) */
} eflashUserData;

/*************************************************************************************************/
/**
 * @brief Initializes the HAL functions to work with internal flash
 * @return kStatus_Success on success
 */
status_t stMCU_Flash_Init(void);

/*************************************************************************************************/
/**
 * @brief Get flash properties
 * @param pflashBlockBase Pointer to Base Address
 * @param pflashTotalSize Pointer to Total Size
 * @param pflashSectorSize Pointer to Sector Size
 * @return void
 */
void vMCU_Flash_getProperties(uint32_t* pflashBlockBase, uint32_t* pflashTotalSize, uint32_t* pflashSectorSize);

/*************************************************************************************************/
/**
 * @brief Read a uint32_t value from a flash valid address
 * @param destAdrss Address to read from
 * @return stored value
 */
uint32_t stMCU_Flash_ReadU32(uint32_t destAdrss);

/*************************************************************************************************/
/**
 * @brief Program data in the flash
 * @param destAdrss Destination Address
 * @param buffer buffer to be written
 * @param lengthInBytes number of bytes to be written
 * @return kStatus_Success on success
 */
status_t stMCU_Flash_Program(uint32_t destAdrss, uint32_t *buffer, uint32_t lengthInBytes);

/*************************************************************************************************/
/**
 * @brief Program a user defined data in the flash
 * @param data data type to be programmed
 * @param value pointer to the data to be saved
 * @return kStatus_Success on success
 * @attention This function is preferred to be used and must be well defined!!!
 */
status_t stMCU_Flash_saveKnownData(eflashUserData data, void* value);

/*************************************************************************************************/
/**
 * @brief Read a user defined data from the flash
 * @param data data type to be read
 * @param value pointer to the data read
 * @return kStatus_Success on success
 * @attention This function is preferred to be used and must be well defined!!!
 */
status_t stMCU_Flash_retrieveKnownData(eflashUserData data,const void * dataRead);
/*************************************************************************************************/
#endif /* APPLICATION_MICROCONTROLLER_FLASH_MCU_FLASH_H_ */
