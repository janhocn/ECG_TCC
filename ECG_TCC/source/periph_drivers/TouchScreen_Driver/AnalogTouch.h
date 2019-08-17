/**
 ******************************************************************************
 * @file    AnalogTouch.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    8 July 2017
 * @brief   Analog Touch Driver
 ******************************************************************************
 */
#ifndef FSL_RESISTIVETOUCH_H_
#define FSL_RESISTIVETOUCH_H_

#include "stdbool.h"
#include "stdint.h"
#include "arm_math.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_adc16.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task_manager.h"

/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
typedef struct _analogTouch_handle
{
    uint16_t xSize; /** @< X Size in pixels of the display*/
    uint16_t ySize; /** @< Y Size in pixels of the display*/

    uint16_t pressureThreshold; /** @< Pressure Threshold to detect a touch*/
    uint16_t pressureThresholdMax; /** @< Maximum Pressure Threshold to detect a touch*/
    uint16_t _rxplate; /** @< Resistance of the touch plate*/

    struct
    {
        uint32_t channelNumberXm; /** @< K64 X- ADC channel*/
        uint32_t channelNumberYp; /** @< K64 Y+ ADC channel*/
        uint32_t adcChannelGroup; /** @< K64 ADC channel Group*/
        ADC_Type *ADCBase; /** @< K64 ADC Instance*/
        adc16_config_t adc16ConfigStruct; /** @< Structure for ADC configuration*/
        adc16_channel_config_t adc16ChannelConfigStruct; /** @< Structure for ADC Channel configuration*/
        uint8_t resolution; /** @< ADC Resolution in bits*/
        uint32_t MAXADCValue; /** @< Maximum value based on adc resolution*/
        volatile bool isADCConversionCompleted; /** @< Flag to signal conversion completed*/
        uint32_t conversionValue; /** @< ADC Conversion value*/
        SemaphoreHandle_t xSemaSignal; /** @< Semaphore to control the singals ADC completion IRQ*/
    } Analog;

    struct
    {
        GPIO_Type *GPIOYp; /** @< K64 Y+ GPIO*/
        GPIO_Type *GPIOXp; /** @< K64 X+ GPIO*/
        PORT_Type *PORTYp; /** @< K64 Y+ Port*/
        PORT_Type *PORTXp; /** @< K64 X+ Port*/
        uint16_t pinXp; /** @< K64 Y+ pin*/
        uint16_t pinYp; /** @< K64 X+ pin*/
        GPIO_Type *GPIOYm; /** @< K64 Y- GPIO*/
        GPIO_Type *GPIOXm; /** @< K64 X- GPIO*/
        PORT_Type *PORTYm; /** @< K64 Y- Port*/
        PORT_Type *PORTXm; /** @< K64 X- Port*/
        uint16_t pinXm; /** @< K64 Y- pin*/
        uint16_t pinYm; /** @< K64 X- pin*/
    } Digital;
} analogTouch_handle_t;

/*************************************************************************************************/
/**
 * @brief Configures the module (initialize it)
 * @param touchHandle Touch Screen Handler for configuration
 * @return void
 */
void vAnalogTouch_Configure(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
/**
 * @brief Check if the touchscreen is pressed
 * @param touchHandle Touch Screen Handler for configuration
 * @return True or False
 */
bool bAnalogTouch_isPressed(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
/**
 * @brief Measure the ADC value on X
 * @param touchHandle Touch Screen Handler for configuration
 * @return ADC Value
 */
uint16_t ui16AnalogTouch_MeasureX(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
/**
 * @brief Measure the ADC value on Y
 * @param touchHandle Touch Screen Handler for configuration
 * @return ADC Value
 */
uint16_t ui16AnalogTouch_MeasureY(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
/**
 * @brief Prepare the X line to be read
 * @param touchHandle Touch Screen Handler for configuration
 * @return void
 */
void vAnalogTouch_ActivateX(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
/**
 * @brief Prepare the Y line to be read
 * @param touchHandle Touch Screen Handler for configuration
 * @return void
 */
void vAnalogTouch_ActivateY(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
/**
 * @brief ADC Interrupt Handler
 * @param touchHandle Touch Screen Handler for configuration
 * @return void
 */
void vAnalogTouch_ADCIRQHandler(analogTouch_handle_t* touchHandle);
/*************************************************************************************************/

#endif /* FSL_RESISTIVETOUCH_H_ */
