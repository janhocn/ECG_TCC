/*
 * PWM.h
 *
 *  Created on: 2 de dez de 2017
 *      Author: Diego
 */

#ifndef APPLICATION_MICROCONTROLLER_PWM_PWM_H_
#define APPLICATION_MICROCONTROLLER_PWM_PWM_H_

#include "fsl_gpio.h"
#include "fsl_ftm.h"

/** @def The Flextimer instance used for board */
#define TFT_BKL_FTM_BASEADDR FTM0
/** @def The Flextimer channel used for board */
#define TFT_BKL_FTM_CHANNEL kFTM_Chnl_0
/** @def Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*************************************************************************************************/
/**
 @enum Parameters to distinguish the PWM control.
 */
typedef enum {
   _E_PWM_TFT_BACKLIGHT, /** @< TFT Backlight PWM*/
}ePWMDevices;

/*************************************************************************************************/
/**
 @brief Initializes PWM module.
 @return void
 */
void vPWM_Init(void);

/*************************************************************************************************/
/**
 @brief Update the duty cycle.
 @param device which PWM device
 @param _dc Duty Cycle (0-100)
 @return void
 */
void vPWM_UpdateDutyCycle(ePWMDevices device, uint8_t _dc);

/*************************************************************************************************/
/**
 @brief Get the current duty cycle.
 @param device which PWM device
 @return void
 */
uint8_t uiPWM_GetDutyCycle(ePWMDevices device);
#endif /* APPLICATION_MICROCONTROLLER_PWM_PWM_H_ */
