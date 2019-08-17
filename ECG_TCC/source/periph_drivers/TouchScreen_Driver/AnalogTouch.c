/**
 ******************************************************************************
 * @file    AnalogTouch.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    8 July 2017
 * @brief   Analog Touch Driver
 ******************************************************************************
 */

#include <stdlib.h>
#include "AnalogTouch.h"
#include "user.h"

/*************************************************************************************************/
/**
 * @enum Type of reading for the lines (x,y)
 */
typedef enum _eAnalogTouchReadType
{
    _E_READ_XM_ANA, /** @< Analog read on X-*/
    _E_READ_YP_ANA, /** @< Analog read on Y+*/
    _E_READ_XM_DIG, /** @< Digital read on X-*/
    _E_READ_XP_DIG, /** @< Digital read on X+*/
    _E_READ_YM_DIG, /** @< Digital read on Y-*/
    _E_READ_YP_DIG, /** @< Digital read on Y+*/
} eAnalogTouchReadType_t;


/*************************************************************************************************/
/**
 * @brief Read on line (pin) either digital or analog
 * @param touchHandle Touch Screen Handler
 * @param typeRead Reading type (x,y, analog or digital) -> see eAnalogTouchReadType_t
 * @return Read Value.
 */
static uint16_t _ui16AnalogTouch_ReadLine(analogTouch_handle_t* touchHandle, eAnalogTouchReadType_t typeRead);

/*************************************************************************************************/
/**
 * @brief Write on line (pin) digital only
 * @param touchHandle Touch Screen Handler
 * @param typeRead Reading type (x,y,digital) -> see eAnalogTouchReadType_t
 * @param value Digital value to be written
 * @return void.
 */
static void _vAnalogTouch_WriteLine(analogTouch_handle_t* touchHandle, eAnalogTouchReadType_t typeWrite, bool value);

/*************************************************************************************************/
/**
 * @brief Configures the analog peripheral
 * @param touchHandle Touch Screen Handler
 * @return void.
 */
static void _vAnalogTouch_InitADC(analogTouch_handle_t* touchHandle);

/*************************************************************************************************/
static uint16_t _ui16AnalogTouch_pressure(analogTouch_handle_t* touchHandle)
{
    volatile uint16_t z1;
    volatile uint16_t z2;

    // Set X+ to ground
    _vAnalogTouch_WriteLine(touchHandle, _E_READ_XP_DIG, 0);

    // Set Y- to VCC
    _vAnalogTouch_WriteLine(touchHandle, _E_READ_YM_DIG, 1);

    // Hi-Z X- and Y+
    _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_XM_DIG);
    _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_YP_DIG);

    z1 = _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_XM_ANA);
    z2 = _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_YP_ANA);

    if (touchHandle->_rxplate != 0)
    {
        // now read the x
        float32_t rtouch;
        rtouch = z2;
        rtouch /= z1;
        rtouch -= 1;
        vAnalogTouch_ActivateX(touchHandle);

        rtouch *= (touchHandle->Analog.MAXADCValue - ui16AnalogTouch_MeasureY(touchHandle));
        rtouch *= touchHandle->_rxplate;
        rtouch /= (touchHandle->Analog.MAXADCValue + 1);

        return (uint16_t)rtouch;
    }
    else
    {
        return (touchHandle->Analog.MAXADCValue - (z2 - z1));
    }
}

/*************************************************************************************************/
static void _vAnalogTouch_InitADC(analogTouch_handle_t* touchHandle)
{
    ADC16_Init(touchHandle->Analog.ADCBase, &touchHandle->Analog.adc16ConfigStruct);
    ADC16_SetHardwareAverage(touchHandle->Analog.ADCBase, kADC16_HardwareAverageCount32);
    ADC16_EnableHardwareTrigger(touchHandle->Analog.ADCBase, false);
    ADC16_DoAutoCalibration(touchHandle->Analog.ADCBase);
}

/*************************************************************************************************/
static void _vAnalogTouch_WriteLine(analogTouch_handle_t* touchHandle, eAnalogTouchReadType_t typeWrite, bool value)
{
    switch (typeWrite)
    {
        case _E_READ_XM_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTXm, touchHandle->Digital.pinXm, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOXm, touchHandle->Digital.pinXm, &(gpio_pin_config_t ) { kGPIO_DigitalOutput, (value) });
            GPIO_WritePinOutput(touchHandle->Digital.GPIOXm, touchHandle->Digital.pinXm, value);
            break;
        case _E_READ_XP_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTXp, touchHandle->Digital.pinXp, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOXp, touchHandle->Digital.pinXp, &(gpio_pin_config_t ) { kGPIO_DigitalOutput, (value) });
            GPIO_WritePinOutput(touchHandle->Digital.GPIOXp, touchHandle->Digital.pinXp, value);
            break;
        case _E_READ_YM_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTYm, touchHandle->Digital.pinYm, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOYm, touchHandle->Digital.pinYm, &(gpio_pin_config_t ) { kGPIO_DigitalOutput, (value) });
            GPIO_WritePinOutput(touchHandle->Digital.GPIOYm, touchHandle->Digital.pinYm, value);
            break;
        default:
            case _E_READ_YP_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTYp, touchHandle->Digital.pinYp, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOYp, touchHandle->Digital.pinYp, &(gpio_pin_config_t ) { kGPIO_DigitalOutput, (value) });
            GPIO_WritePinOutput(touchHandle->Digital.GPIOYp, touchHandle->Digital.pinYp, value);
            break;
    }
}

/*************************************************************************************************/
static uint16_t _ui16AnalogTouch_ReadLine(analogTouch_handle_t* touchHandle, eAnalogTouchReadType_t typeRead)
{
    uint16_t readValue = 0;
    switch (typeRead)
    {
        case _E_READ_XM_ANA:
            PORT_SetPinMux(touchHandle->Digital.PORTXm, touchHandle->Digital.pinXm, kPORT_PinDisabledOrAnalog);
            _vAnalogTouch_InitADC(touchHandle);
            touchHandle->Analog.adc16ChannelConfigStruct.channelNumber = touchHandle->Analog.channelNumberXm;
            touchHandle->Analog.isADCConversionCompleted = false;
            touchHandle->Analog.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
            ADC16_SetChannelConfig(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup, &touchHandle->Analog.adc16ChannelConfigStruct);
//            while ((touchHandle->Analog.isADCConversionCompleted) == false)
//            {
//            }
            while (0U
                    == (kADC16_ChannelConversionDoneFlag
                            & ADC16_GetChannelStatusFlags(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup)))
            {
                ;
                ;
            }
            touchHandle->Analog.conversionValue = ADC16_GetChannelConversionValue(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup);
//            xTaskManager_SemaphoreTake(touchHandle->Analog.xSemaSignal, portMAX_DELAY);
            touchHandle->Analog.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
            touchHandle->Analog.isADCConversionCompleted = false;
            readValue = touchHandle->Analog.conversionValue;
            break;
        case _E_READ_YP_ANA:
            PORT_SetPinMux(touchHandle->Digital.PORTYp, touchHandle->Digital.pinYp, kPORT_PinDisabledOrAnalog);
            _vAnalogTouch_InitADC(touchHandle);
            touchHandle->Analog.adc16ChannelConfigStruct.channelNumber = touchHandle->Analog.channelNumberYp;
            touchHandle->Analog.isADCConversionCompleted = false;
            touchHandle->Analog.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
            ADC16_SetChannelConfig(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup, &touchHandle->Analog.adc16ChannelConfigStruct);
//            while ((touchHandle->Analog.isADCConversionCompleted) == false)
//            {
//            }
            while (0U
                    == (kADC16_ChannelConversionDoneFlag
                            & ADC16_GetChannelStatusFlags(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup)))
            {
                ;
                ;
            }
            touchHandle->Analog.conversionValue = ADC16_GetChannelConversionValue(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup);
//            xTaskManager_SemaphoreTake(touchHandle->Analog.xSemaSignal, portMAX_DELAY);
            touchHandle->Analog.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
            touchHandle->Analog.isADCConversionCompleted = false;
            readValue = touchHandle->Analog.conversionValue;
            break;
        case _E_READ_XM_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTXm, touchHandle->Digital.pinXm, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOXm, touchHandle->Digital.pinXm, &(gpio_pin_config_t ) { kGPIO_DigitalInput, (0) });
            readValue = GPIO_ReadPinInput(touchHandle->Digital.GPIOXm, touchHandle->Digital.pinXm);
            break;
        case _E_READ_XP_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTXp, touchHandle->Digital.pinXp, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOXp, touchHandle->Digital.pinXp, &(gpio_pin_config_t ) { kGPIO_DigitalInput, (0) });
            readValue = GPIO_ReadPinInput(touchHandle->Digital.GPIOXp, touchHandle->Digital.pinXp);
            break;
        case _E_READ_YM_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTYm, touchHandle->Digital.pinYm, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOYm, touchHandle->Digital.pinYm, &(gpio_pin_config_t ) { kGPIO_DigitalInput, (0) });
            readValue = GPIO_ReadPinInput(touchHandle->Digital.GPIOYm, touchHandle->Digital.pinYm);
            break;
        default:
            case _E_READ_YP_DIG:
            //Pin now is GPIO
            PORT_SetPinMux(touchHandle->Digital.PORTYp, touchHandle->Digital.pinYp, kPORT_MuxAsGpio);
            //Pin is digital input
            GPIO_PinInit(touchHandle->Digital.GPIOYp, touchHandle->Digital.pinYp, &(gpio_pin_config_t ) { kGPIO_DigitalInput, (0) });
            readValue = GPIO_ReadPinInput(touchHandle->Digital.GPIOYp, touchHandle->Digital.pinYp);
            break;
    }
    return readValue;
}

/*************************************************************************************************/
void vAnalogTouch_ActivateY(analogTouch_handle_t* touchHandle)
{
    _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_XP_DIG);
    _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_XM_DIG);

    _vAnalogTouch_WriteLine(touchHandle, _E_READ_YP_DIG, 1);
    _vAnalogTouch_WriteLine(touchHandle, _E_READ_YM_DIG, 0);
}

/*************************************************************************************************/
void vAnalogTouch_ActivateX(analogTouch_handle_t* touchHandle)
{
    _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_YP_DIG);
    _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_YM_DIG);

    _vAnalogTouch_WriteLine(touchHandle, _E_READ_XP_DIG, 1);
    _vAnalogTouch_WriteLine(touchHandle, _E_READ_XM_DIG, 0);
}

/*************************************************************************************************/
bool bAnalogTouch_isPressed(analogTouch_handle_t* touchHandle)
{
    volatile uint16_t z = _ui16AnalogTouch_pressure(touchHandle);
    if (z > touchHandle->pressureThreshold && z < touchHandle->pressureThresholdMax)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*************************************************************************************************/
uint16_t ui16AnalogTouch_MeasureY(analogTouch_handle_t* touchHandle)
{
    return _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_YP_ANA);
}

/*************************************************************************************************/
uint16_t ui16AnalogTouch_MeasureX(analogTouch_handle_t* touchHandle)
{
    return _ui16AnalogTouch_ReadLine(touchHandle, _E_READ_XM_ANA);
}

/*************************************************************************************************/
void vAnalogTouch_ADCIRQHandler(analogTouch_handle_t* touchHandle)
{
    touchHandle->Analog.isADCConversionCompleted = true;
    touchHandle->Analog.conversionValue = ADC16_GetChannelConversionValue(touchHandle->Analog.ADCBase, touchHandle->Analog.adcChannelGroup);
//    xTaskManager_SemaphoreGive(touchHandle->Analog.xSemaSignal);
}

/*************************************************************************************************/
void vAnalogTouch_Configure(analogTouch_handle_t* touchHandle)
{
    ADC16_GetDefaultConfig(&touchHandle->Analog.adc16ConfigStruct);
    switch (touchHandle->Analog.resolution)
    {
        case 8:
            touchHandle->Analog.adc16ConfigStruct.resolution = kADC16_ResolutionSE8Bit;
            touchHandle->Analog.MAXADCValue = 255;
            break;
        default:
            case 10:
            touchHandle->Analog.adc16ConfigStruct.resolution = kADC16_ResolutionSE10Bit;
            touchHandle->Analog.MAXADCValue = 1023;
            break;
        case 12:
            touchHandle->Analog.adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
            touchHandle->Analog.MAXADCValue = 4095;
            break;
        case 16:
            touchHandle->Analog.adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;
            touchHandle->Analog.MAXADCValue = 65535;
            break;
    }

    touchHandle->Analog.isADCConversionCompleted = false;

    touchHandle->Digital.PORTYp = xUser_GetPortByGPIO((uint32_t)touchHandle->Digital.GPIOYp);
    touchHandle->Digital.PORTYm = xUser_GetPortByGPIO((uint32_t)touchHandle->Digital.GPIOYm);
    touchHandle->Digital.PORTXp = xUser_GetPortByGPIO((uint32_t)touchHandle->Digital.GPIOXp);
    touchHandle->Digital.PORTXm = xUser_GetPortByGPIO((uint32_t)touchHandle->Digital.GPIOXm);

//    touchHandle->Analog.xSemaSignal = xTaskManager_SemaphoreCreate();
//    configASSERT(touchHandle->Analog.xSemaSignal != NULL);

    touchHandle->Analog.adc16ChannelConfigStruct.enableDifferentialConversion = false;
    touchHandle->Analog.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    _vAnalogTouch_InitADC(touchHandle);
}
/*************************************************************************************************/
