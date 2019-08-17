/**
 ******************************************************************************
 * @file    user.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 June 2017
 * @brief   General Module
 ******************************************************************************
 */
#ifndef USER_USER_H_
#define USER_USER_H_

#include "fsl_gpio.h"
#include "MK64F12.h"
#include "fsl_port.h"
/*************************************************************************************************/
/**
 * @brief Converts K64 GPIO Base to PORT
 * @brief base GPIO
 * @return PORT
 */

#define FRDM_LED_PIN 1U << 23U
#define EXT_LED1_PIN 1U << 22U
#define EXT_LED2_PIN 1U << 16U

#define vUser_ToggleFRDM_LED() GPIO_TogglePinsOutput(GPIOB, FRDM_LED_PIN);
#define vUser_ToggleEXT_LED1() GPIO_TogglePinsOutput(GPIOB, EXT_LED1_PIN);
#define vUser_ToggleEXT_LED2() GPIO_TogglePinsOutput(GPIOC, EXT_LED2_PIN);
#define vUser_SetEXT_LED2() GPIO_SetPinsOutput(GPIOC, EXT_LED2_PIN);
#define vUser_ClearEXT_LED2() GPIO_ClearPinsOutput(GPIOC, EXT_LED2_PIN);

/*************************************************************************************************/
/**
 * @brief Converts K64 GPIO Base to PORT
 * @brief base GPIO
 * @return PORT
 */
PORT_Type* xUser_GetPortByGPIO(uint32_t base);

/*************************************************************************************************/
/**
 * @brief Configures the DMA peripheral
 * @attention It should be called before all DMA based user modules.
 * @return void
 */
void vUser_InitDMA(void);

#endif /* USER_USER_H_ */
