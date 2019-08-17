/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USE_PROTOTYPE 0

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
    kPIN_MUX_DirectionInput = 0U, /* Input direction */
    kPIN_MUX_DirectionOutput = 1U, /* Output direction */
    kPIN_MUX_DirectionInputOrOutput = 2U /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

typedef enum _pinIdx
{
    PIN0_IDX,
    PIN1_IDX,
    PIN2_IDX,
    PIN3_IDX,
    PIN4_IDX,
    PIN5_IDX,
    PIN6_IDX,
    PIN7_IDX,
    PIN8_IDX,
    PIN9_IDX,
    PIN10_IDX,
    PIN11_IDX,
    PIN12_IDX,
    PIN13_IDX,
    PIN14_IDX,
    PIN15_IDX,
    PIN16_IDX,
    PIN17_IDX,
    PIN18_IDX,
    PIN19_IDX,
    PIN20_IDX,
    PIN21_IDX,
    PIN22_IDX,
    PIN23_IDX,
    PIN24_IDX,
    PIN25_IDX,
} pinIdx;

#if USE_PROTOTYPE
/**
 * \defgroup Button GPIO Configuration
 * @{
 */
#define GPIO_HOME_BUTTON_CLK_ENABLE() 	CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_HOME_BUTTON_PORT			PORTC
#define GPIO_HOME_BUTTON_GPIO  			GPIOC
#define GPIO_HOME_BUTTON_PIN			PIN5_IDX
#define GPIO_HOME_BUTTON_ALT			kPORT_MuxAsGpio
#define GPIO_HOME_BUTTON_MODE			kGPIO_DigitalInput
#define GPIO_HOME_BUTTON_INT			kPORT_InterruptFallingEdge
#define GPIO_HOME_BUTTON_INIT_VAL 		0
#define GPIO_HOME_BUTTON_PULL			kPORT_PullDisable
#define GPIO_HOME_BUTTON_SLEWRATE		kPORT_SlowSlewRate
#define GPIO_HOME_BUTTON_OPENDRAIN		kPORT_OpenDrainDisable
#define GPIO_HOME_BUTTON_DFILTER		kPORT_PassiveFilterDisable
#define GPIO_HOME_BUTTON_LOCK			kPORT_UnlockRegister
#define GPIO_HOME_BUTTON_DRIVESTRENGTH	kPORT_LowDriveStrength
/**@}*/

/**
 * \defgroup LCD GPIO Configuration
 * @{
 */
#define GPIO_TFT_RST_CLK_ENABLE() 	CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_TFT_RST_PORT			PORTC
#define GPIO_TFT_RST_GPIO  			GPIOC
#define GPIO_TFT_RST_PIN			PIN3_IDX
#define GPIO_TFT_RST_ALT			kPORT_MuxAsGpio
#define GPIO_TFT_RST_MODE			kGPIO_DigitalOutput
#define GPIO_TFT_RST_INT			0 //No interrupt
#define GPIO_TFT_RST_INIT_VAL 		0
#define GPIO_TFT_RST_PULL			kPORT_PullDisable
#define GPIO_TFT_RST_SLEWRATE		kPORT_SlowSlewRate
#define GPIO_TFT_RST_OPENDRAIN		kPORT_OpenDrainDisable
#define GPIO_TFT_RST_DFILTER		kPORT_PassiveFilterDisable
#define GPIO_TFT_RST_LOCK			kPORT_UnlockRegister
#define GPIO_TFT_RST_DRIVESTRENGTH	kPORT_LowDriveStrength

#define GPIO_TFT_BKL_CLK_ENABLE()  	CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_TFT_BKL_PORT			PORTC
#define GPIO_TFT_BKL_GPIO   		GPIOC
#define GPIO_TFT_BKL_PIN			PIN1_IDX
#define GPIO_TFT_BKL_ALT			kPORT_MuxAlt4 //PWM
#define GPIO_TFT_BKL_MODE			kGPIO_DigitalOutput
#define GPIO_TFT_BKL_INT			0 //No interrupt
#define GPIO_TFT_BKL_INIT_VAL 		0
#define GPIO_TFT_BKL_PULL			kPORT_PullDisable
#define GPIO_TFT_BKL_SLEWRATE		kPORT_SlowSlewRate
#define GPIO_TFT_BKL_OPENDRAIN		kPORT_OpenDrainDisable
#define GPIO_TFT_BKL_DFILTER		kPORT_PassiveFilterDisable
#define GPIO_TFT_BKL_LOCK			kPORT_UnlockRegister
#define GPIO_TFT_BKL_DRIVESTRENGTH	kPORT_LowDriveStrength

#define GPIO_TFT_DC_CLK_ENABLE()  	CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_TFT_DC_PORT			PORTC
#define GPIO_TFT_DC_GPIO   			GPIOC
#define GPIO_TFT_DC_PIN				PIN2_IDX
#define GPIO_TFT_DC_ALT				kPORT_MuxAsGpio
#define GPIO_TFT_DC_MODE			kGPIO_DigitalOutput
#define GPIO_TFT_DC_INT				0 //No interrupt
#define GPIO_TFT_DC_INIT_VAL 		1
#define GPIO_TFT_DC_PULL			kPORT_PullDisable
#define GPIO_TFT_DC_SLEWRATE		kPORT_SlowSlewRate
#define GPIO_TFT_DC_OPENDRAIN		kPORT_OpenDrainDisable
#define GPIO_TFT_DC_DFILTER			kPORT_PassiveFilterDisable
#define GPIO_TFT_DC_LOCK			kPORT_UnlockRegister
#define GPIO_TFT_DC_DRIVESTRENGTH	kPORT_LowDriveStrength

#define GPIO_TFT_CS_CLK_ENABLE()    CLOCK_EnableClock(kCLOCK_PortD)
#define GPIO_TFT_CS_PORT            PORTD
#define GPIO_TFT_CS_GPIO            GPIOD
#define GPIO_TFT_CS_PIN             PIN0_IDX
#define GPIO_TFT_CS_ALT             kPORT_MuxAsGpio
#define GPIO_TFT_CS_MODE            kGPIO_DigitalOutput
#define GPIO_TFT_CS_INT             0 //No interrupt
#define GPIO_TFT_CS_INIT_VAL        1
#define GPIO_TFT_CS_PULL            kPORT_PullDisable
#define GPIO_TFT_CS_SLEWRATE        kPORT_SlowSlewRate
#define GPIO_TFT_CS_OPENDRAIN       kPORT_OpenDrainDisable
#define GPIO_TFT_CS_DFILTER         kPORT_PassiveFilterDisable
#define GPIO_TFT_CS_LOCK            kPORT_UnlockRegister
#define GPIO_TFT_CS_DRIVESTRENGTH   kPORT_LowDriveStrength

#define GPIO_TFT_SO_PORT            PORTD
#define GPIO_TFT_SO_PIN             PIN2_IDX

#define GPIO_TFT_SI_PORT            PORTD
#define GPIO_TFT_SI_PIN             PIN3_IDX

#define GPIO_TFT_CLK_PORT           PORTD
#define GPIO_TFT_CLK_PIN            PIN1_IDX

/**@}*/

/**
 * \defgroup AFE4400 GPIO Configuration
 * @{
 */
#define GPIO_AFE4400_ADCRDY_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_AFE4400_ADCRDY_PORT           PORTC
#define GPIO_AFE4400_ADCRDY_GPIO           GPIOC
#define GPIO_AFE4400_ADCRDY_PIN            PIN4_IDX
#define GPIO_AFE4400_ADCRDY_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_ADCRDY_MODE           kGPIO_DigitalInput
#define GPIO_AFE4400_ADCRDY_INT            kPORT_InterruptRisingEdge
#define GPIO_AFE4400_ADCRDY_INIT_VAL       0
#define GPIO_AFE4400_ADCRDY_PULL           kPORT_PullDisable
#define GPIO_AFE4400_ADCRDY_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_ADCRDY_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_ADCRDY_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_ADCRDY_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_ADCRDY_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_CS_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortD)
#define GPIO_AFE4400_CS_PORT           PORTD
#define GPIO_AFE4400_CS_GPIO           GPIOD
#define GPIO_AFE4400_CS_PIN            PIN4_IDX
#define GPIO_AFE4400_CS_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_CS_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_CS_INT            0 //No interrupt
#define GPIO_AFE4400_CS_INIT_VAL       1
#define GPIO_AFE4400_CS_PULL           kPORT_PullDisable
#define GPIO_AFE4400_CS_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_CS_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_CS_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_CS_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_CS_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_RST_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_AFE4400_RST_PORT           PORTB
#define GPIO_AFE4400_RST_GPIO           GPIOB
#define GPIO_AFE4400_RST_PIN            PIN20_IDX
#define GPIO_AFE4400_RST_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_RST_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_RST_INT            0 //No interrupt
#define GPIO_AFE4400_RST_INIT_VAL       0
#define GPIO_AFE4400_RST_PULL           kPORT_PullDisable
#define GPIO_AFE4400_RST_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_RST_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_RST_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_RST_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_RST_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_PDN_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_AFE4400_PDN_PORT           PORTC
#define GPIO_AFE4400_PDN_GPIO           GPIOC
#define GPIO_AFE4400_PDN_PIN            PIN11_IDX
#define GPIO_AFE4400_PDN_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_PDN_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_PDN_INT            0 //No interrupt
#define GPIO_AFE4400_PDN_INIT_VAL       0
#define GPIO_AFE4400_PDN_PULL           kPORT_PullDisable
#define GPIO_AFE4400_PDN_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_PDN_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_PDN_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_PDN_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_PDN_DRIVESTRENGTH  kPORT_LowDriveStrength
/**@}*/

/**
 * \defgroup ECG Acquisition ADC GPIO Configuration
 * @{
 */
#define GPIO_ECG_IN_CLK_ENABLE()           CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ECG_IN_PORT                   PORTB
#define GPIO_ECG_IN_GPIO                   GPIOB
#define GPIO_ECG_IN_PIN                    PIN2_IDX
#define GPIO_ECG_IN_ALT                    kPORT_PinDisabledOrAnalog
#define GPIO_ECG_IN_ADC_BASE_ADDR          ADC0
#define GPIO_ECG_IN_ADC_CHANNEL_GROUP      0
#define GPIO_ECG_IN_ADC_CHANNEL            12
#define GPIO_ECG_IN_ADC_IRQ                ADC0_IRQn
/**@}*/

/**
 * \defgroup Touch Screen Pins
 * @{
 */
#define GPIO_ATOUCH_YP_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ATOUCH_YP_PORT           PORTB
#define GPIO_ATOUCH_YP_GPIO           GPIOB
#define GPIO_ATOUCH_YP_PIN            PIN10_IDX
#define GPIO_ATOUCH_YP_ALT            kPORT_MuxAsGpio
#define GPIO_ATOUCH_YP_MODE           kGPIO_DigitalInput
#define GPIO_ATOUCH_YP_INT            kPORT_InterruptOrDMADisabled
#define GPIO_ATOUCH_YP_INIT_VAL       0
#define GPIO_ATOUCH_YP_PULL           kPORT_PullDisable
#define GPIO_ATOUCH_YP_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_ATOUCH_YP_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_ATOUCH_YP_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_ATOUCH_YP_LOCK           kPORT_UnlockRegister
#define GPIO_ATOUCH_YP_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_ATOUCH_YM_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ATOUCH_YM_PORT           PORTB
#define GPIO_ATOUCH_YM_GPIO           GPIOB
#define GPIO_ATOUCH_YM_PIN            PIN19_IDX
#define GPIO_ATOUCH_YM_ALT            kPORT_MuxAsGpio
#define GPIO_ATOUCH_YM_MODE           kGPIO_DigitalInput
#define GPIO_ATOUCH_YM_INT            kPORT_InterruptOrDMADisabled
#define GPIO_ATOUCH_YM_INIT_VAL       0
#define GPIO_ATOUCH_YM_PULL           kPORT_PullDisable
#define GPIO_ATOUCH_YM_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_ATOUCH_YM_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_ATOUCH_YM_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_ATOUCH_YM_LOCK           kPORT_UnlockRegister
#define GPIO_ATOUCH_YM_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_ATOUCH_XP_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ATOUCH_XP_PORT           PORTB
#define GPIO_ATOUCH_XP_GPIO           GPIOB
#define GPIO_ATOUCH_XP_PIN            PIN18_IDX
#define GPIO_ATOUCH_XP_ALT            kPORT_MuxAsGpio
#define GPIO_ATOUCH_XP_MODE           kGPIO_DigitalInput
#define GPIO_ATOUCH_XP_INT            kPORT_InterruptOrDMADisabled
#define GPIO_ATOUCH_XP_INIT_VAL       0
#define GPIO_ATOUCH_XP_PULL           kPORT_PullDisable
#define GPIO_ATOUCH_XP_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_ATOUCH_XP_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_ATOUCH_XP_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_ATOUCH_XP_LOCK           kPORT_UnlockRegister
#define GPIO_ATOUCH_XP_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_ATOUCH_XM_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ATOUCH_XM_PORT           PORTB
#define GPIO_ATOUCH_XM_GPIO           GPIOB
#define GPIO_ATOUCH_XM_PIN            PIN11_IDX
#define GPIO_ATOUCH_XM_ALT            kPORT_MuxAsGpio
#define GPIO_ATOUCH_XM_MODE           kGPIO_DigitalInput
#define GPIO_ATOUCH_XM_INT            kPORT_InterruptOrDMADisabled
#define GPIO_ATOUCH_XM_INIT_VAL       0
#define GPIO_ATOUCH_XM_PULL           kPORT_PullDisable
#define GPIO_ATOUCH_XM_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_ATOUCH_XM_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_ATOUCH_XM_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_ATOUCH_XM_LOCK           kPORT_UnlockRegister
#define GPIO_ATOUCH_XM_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_ATOUCH_BASE_ADDR              ADC1
#define GPIO_ATOUCH_ADC_CHANNEL_GROUP      0
#define GPIO_ATOUCH_ADC_XM_CHANNEL         15
#define GPIO_ATOUCH_ADC_YP_CHANNEL         14
#define GPIO_ATOUCH_ADC_IRQ                ADC1_IRQn
/**@}*/

/**
 * \defgroup TSC2007 Touch Controller Configuration
 * @{
 */
#define GPIO_TOUCH_PEN_IRQ_CLK_ENABLE() 	CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_TOUCH_PEN_IRQ_PORT				PORTA
#define GPIO_TOUCH_PEN_IRQ_GPIO  			GPIOA
#define GPIO_TOUCH_PEN_IRQ_PIN				PIN13_IDX
#define GPIO_TOUCH_PEN_IRQ_ALT				kPORT_MuxAsGpio
#define GPIO_TOUCH_PEN_IRQ_MODE				kGPIO_DigitalInput
#define GPIO_TOUCH_PEN_IRQ_INT				kPORT_InterruptFallingEdge
#define GPIO_TOUCH_PEN_IRQ_INIT_VAL 		0
#define GPIO_TOUCH_PEN_IRQ_PULL				kPORT_PullDisable
#define GPIO_TOUCH_PEN_IRQ_SLEWRATE			kPORT_SlowSlewRate
#define GPIO_TOUCH_PEN_IRQ_OPENDRAIN		kPORT_OpenDrainDisable
#define GPIO_TOUCH_PEN_IRQ_DFILTER			kPORT_PassiveFilterDisable
#define GPIO_TOUCH_PEN_IRQ_LOCK				kPORT_UnlockRegister
#define GPIO_TOUCH_PEN_IRQ_DRIVESTRENGTH	kPORT_LowDriveStrength
#define GPIO_TOUCH_PEN_IRQn					PORTA_IRQn
/**@}*/

#else
/**
 * \defgroup LCD GPIO Configuration
 * @{
 */
#define GPIO_TFT_RST_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_TFT_RST_PORT           PORTA
#define GPIO_TFT_RST_GPIO           GPIOA
#define GPIO_TFT_RST_PIN            PIN15_IDX
#define GPIO_TFT_RST_ALT            kPORT_MuxAsGpio
#define GPIO_TFT_RST_MODE           kGPIO_DigitalOutput
#define GPIO_TFT_RST_INT            0 //No interrupt
#define GPIO_TFT_RST_INIT_VAL       0
#define GPIO_TFT_RST_PULL           kPORT_PullDisable
#define GPIO_TFT_RST_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_TFT_RST_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_TFT_RST_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_TFT_RST_LOCK           kPORT_UnlockRegister
#define GPIO_TFT_RST_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_TFT_BKL_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_TFT_BKL_PORT           PORTC
#define GPIO_TFT_BKL_GPIO           GPIOC
#define GPIO_TFT_BKL_PIN            PIN1_IDX
#define GPIO_TFT_BKL_ALT            kPORT_MuxAlt4 //PWM
#define GPIO_TFT_BKL_MODE           kGPIO_DigitalOutput
#define GPIO_TFT_BKL_INT            0 //No interrupt
#define GPIO_TFT_BKL_INIT_VAL       0
#define GPIO_TFT_BKL_PULL           kPORT_PullDisable
#define GPIO_TFT_BKL_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_TFT_BKL_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_TFT_BKL_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_TFT_BKL_LOCK           kPORT_UnlockRegister
#define GPIO_TFT_BKL_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_TFT_DC_CLK_ENABLE()    CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_TFT_DC_PORT            PORTA
#define GPIO_TFT_DC_GPIO            GPIOA
#define GPIO_TFT_DC_PIN             PIN14_IDX
#define GPIO_TFT_DC_ALT             kPORT_MuxAsGpio
#define GPIO_TFT_DC_MODE            kGPIO_DigitalOutput
#define GPIO_TFT_DC_INT             0 //No interrupt
#define GPIO_TFT_DC_INIT_VAL        1
#define GPIO_TFT_DC_PULL            kPORT_PullDisable
#define GPIO_TFT_DC_SLEWRATE        kPORT_SlowSlewRate
#define GPIO_TFT_DC_OPENDRAIN       kPORT_OpenDrainDisable
#define GPIO_TFT_DC_DFILTER         kPORT_PassiveFilterDisable
#define GPIO_TFT_DC_LOCK            kPORT_UnlockRegister
#define GPIO_TFT_DC_DRIVESTRENGTH   kPORT_LowDriveStrength

#define GPIO_TFT_CS_CLK_ENABLE()    CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_TFT_CS_PORT            PORTB
#define GPIO_TFT_CS_GPIO            GPIOB
#define GPIO_TFT_CS_PIN             PIN20_IDX
#define GPIO_TFT_CS_ALT             kPORT_MuxAsGpio
#define GPIO_TFT_CS_MODE            kGPIO_DigitalOutput
#define GPIO_TFT_CS_INT             0 //No interrupt
#define GPIO_TFT_CS_INIT_VAL        1
#define GPIO_TFT_CS_PULL            kPORT_PullDisable
#define GPIO_TFT_CS_SLEWRATE        kPORT_SlowSlewRate
#define GPIO_TFT_CS_OPENDRAIN       kPORT_OpenDrainDisable
#define GPIO_TFT_CS_DFILTER         kPORT_PassiveFilterDisable
#define GPIO_TFT_CS_LOCK            kPORT_UnlockRegister
#define GPIO_TFT_CS_DRIVESTRENGTH   kPORT_LowDriveStrength

/**@}*/

/**
 * \defgroup AFE4400 GPIO Configuration
 * @{
 */
#define GPIO_AFE4400_ADCRDY_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_AFE4400_ADCRDY_PORT           PORTC
#define GPIO_AFE4400_ADCRDY_GPIO           GPIOC
#define GPIO_AFE4400_ADCRDY_PIN            PIN3_IDX
#define GPIO_AFE4400_ADCRDY_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_ADCRDY_MODE           kGPIO_DigitalInput
#define GPIO_AFE4400_ADCRDY_INT            kPORT_InterruptRisingEdge
#define GPIO_AFE4400_ADCRDY_INIT_VAL       0
#define GPIO_AFE4400_ADCRDY_PULL           kPORT_PullDisable
#define GPIO_AFE4400_ADCRDY_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_ADCRDY_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_ADCRDY_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_ADCRDY_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_ADCRDY_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_PDALM_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_AFE4400_PDALM_PORT           PORTC
#define GPIO_AFE4400_PDALM_GPIO           GPIOC
#define GPIO_AFE4400_PDALM_PIN            PIN4_IDX
#define GPIO_AFE4400_PDALM_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_PDALM_MODE           kGPIO_DigitalInput
#define GPIO_AFE4400_PDALM_INT            kPORT_InterruptRisingEdge
#define GPIO_AFE4400_PDALM_INIT_VAL       0
#define GPIO_AFE4400_PDALM_PULL           kPORT_PullDisable
#define GPIO_AFE4400_PDALM_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_PDALM_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_PDALM_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_PDALM_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_PDALM_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_DIAGEND_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_AFE4400_DIAGEND_PORT           PORTC
#define GPIO_AFE4400_DIAGEND_GPIO           GPIOC
#define GPIO_AFE4400_DIAGEND_PIN            PIN5_IDX
#define GPIO_AFE4400_DIAGEND_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_DIAGEND_MODE           kGPIO_DigitalInput
#define GPIO_AFE4400_DIAGEND_INT            kPORT_InterruptRisingEdge
#define GPIO_AFE4400_DIAGEND_INIT_VAL       0
#define GPIO_AFE4400_DIAGEND_PULL           kPORT_PullDisable
#define GPIO_AFE4400_DIAGEND_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_DIAGEND_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_DIAGEND_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_DIAGEND_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_DIAGEND_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_LEDALM_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_AFE4400_LEDALM_PORT           PORTC
#define GPIO_AFE4400_LEDALM_GPIO           GPIOC
#define GPIO_AFE4400_LEDALM_PIN            PIN6_IDX
#define GPIO_AFE4400_LEDALM_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_LEDALM_MODE           kGPIO_DigitalInput
#define GPIO_AFE4400_LEDALM_INT            kPORT_InterruptRisingEdge
#define GPIO_AFE4400_LEDALM_INIT_VAL       0
#define GPIO_AFE4400_LEDALM_PULL           kPORT_PullDisable
#define GPIO_AFE4400_LEDALM_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_LEDALM_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_LEDALM_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_LEDALM_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_LEDALM_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_CS_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_AFE4400_CS_PORT           PORTB
#define GPIO_AFE4400_CS_GPIO           GPIOB
#define GPIO_AFE4400_CS_PIN            PIN10_IDX
#define GPIO_AFE4400_CS_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_CS_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_CS_INT            0 //No interrupt
#define GPIO_AFE4400_CS_INIT_VAL       1
#define GPIO_AFE4400_CS_PULL           kPORT_PullDisable
#define GPIO_AFE4400_CS_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_CS_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_CS_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_CS_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_CS_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_RST_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_AFE4400_RST_PORT           PORTA
#define GPIO_AFE4400_RST_GPIO           GPIOA
#define GPIO_AFE4400_RST_PIN            PIN16_IDX
#define GPIO_AFE4400_RST_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_RST_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_RST_INT            0 //No interrupt
#define GPIO_AFE4400_RST_INIT_VAL       0
#define GPIO_AFE4400_RST_PULL           kPORT_PullDisable
#define GPIO_AFE4400_RST_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_RST_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_RST_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_RST_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_RST_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_PDN_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_AFE4400_PDN_PORT           PORTA
#define GPIO_AFE4400_PDN_GPIO           GPIOA
#define GPIO_AFE4400_PDN_PIN            PIN17_IDX
#define GPIO_AFE4400_PDN_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_PDN_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_PDN_INT            0 //No interrupt
#define GPIO_AFE4400_PDN_INIT_VAL       0
#define GPIO_AFE4400_PDN_PULL           kPORT_PullDisable
#define GPIO_AFE4400_PDN_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_PDN_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_PDN_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_PDN_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_PDN_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_AFE4400_POWER_EN_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_AFE4400_POWER_EN_PORT           PORTB
#define GPIO_AFE4400_POWER_EN_GPIO           GPIOB
#define GPIO_AFE4400_POWER_EN_PIN            PIN1_IDX
#define GPIO_AFE4400_POWER_EN_ALT            kPORT_MuxAsGpio
#define GPIO_AFE4400_POWER_EN_MODE           kGPIO_DigitalOutput
#define GPIO_AFE4400_POWER_EN_INT            0 //No interrupt
#define GPIO_AFE4400_POWER_EN_INIT_VAL       1
#define GPIO_AFE4400_POWER_EN_PULL           kPORT_PullDisable
#define GPIO_AFE4400_POWER_EN_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_AFE4400_POWER_EN_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_AFE4400_POWER_EN_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_AFE4400_POWER_EN_LOCK           kPORT_UnlockRegister
#define GPIO_AFE4400_POWER_EN_DRIVESTRENGTH  kPORT_HighDriveStrength
/**@}*/

/**
 * \defgroup ECG Acquisition ADC GPIO Configuration
 * @{
 */
#define GPIO_ECG_POWER_EN_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_ECG_POWER_EN_PORT           PORTC
#define GPIO_ECG_POWER_EN_GPIO           GPIOC
#define GPIO_ECG_POWER_EN_PIN            PIN18_IDX
#define GPIO_ECG_POWER_EN_ALT            kPORT_MuxAsGpio
#define GPIO_ECG_POWER_EN_MODE           kGPIO_DigitalOutput
#define GPIO_ECG_POWER_EN_INT            0 //No interrupt
#define GPIO_ECG_POWER_EN_INIT_VAL       0
#define GPIO_ECG_POWER_EN_PULL           kPORT_PullDisable
#define GPIO_ECG_POWER_EN_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_ECG_POWER_EN_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_ECG_POWER_EN_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_ECG_POWER_EN_LOCK           kPORT_UnlockRegister
#define GPIO_ECG_POWER_EN_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_ECG_IN_CLK_ENABLE()           CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ECG_IN_ALT                    kPORT_PinDisabledOrAnalog
#define GPIO_ECG_IN_ADC_BASE_ADDR          ADC0
#define GPIO_ECG_IN_ADC_CHANNEL_GROUP      0
#define GPIO_ECG_IN_ADC_CHANNEL            23
#define GPIO_ECG_IN_ADC_IRQ                ADC0_IRQn

#define GPIO_ECG_IN_NFILT_CLK_ENABLE()           CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_ECG_IN_NFILT_PORT                   PORTB
#define GPIO_ECG_IN_NFILT_GPIO                   GPIOB
#define GPIO_ECG_IN_NFILT_PIN                    PIN3_IDX
#define GPIO_ECG_IN_NFILT_ALT                    kPORT_PinDisabledOrAnalog
#define GPIO_ECG_IN_NFILT_ADC_BASE_ADDR          ADC0
#define GPIO_ECG_IN_NFILT_ADC_CHANNEL_GROUP      0
#define GPIO_ECG_IN_NFILT_ADC_CHANNEL            13
#define GPIO_ECG_IN_NFILT_ADC_IRQ                ADC0_IRQn
/**@}*/

/**
 * \defgroup TSC2007 Touch Controller Configuration
 * @{
 */
#define GPIO_TOUCH_PEN_IRQ_CLK_ENABLE()     CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_TOUCH_PEN_IRQ_PORT             PORTA
#define GPIO_TOUCH_PEN_IRQ_GPIO             GPIOA
#define GPIO_TOUCH_PEN_IRQ_PIN              PIN13_IDX
#define GPIO_TOUCH_PEN_IRQ_ALT              kPORT_MuxAsGpio
#define GPIO_TOUCH_PEN_IRQ_MODE             kGPIO_DigitalInput
#define GPIO_TOUCH_PEN_IRQ_INT              kPORT_InterruptFallingEdge
#define GPIO_TOUCH_PEN_IRQ_INIT_VAL         0
#define GPIO_TOUCH_PEN_IRQ_PULL             kPORT_PullDisable
#define GPIO_TOUCH_PEN_IRQ_SLEWRATE         kPORT_SlowSlewRate
#define GPIO_TOUCH_PEN_IRQ_OPENDRAIN        kPORT_OpenDrainDisable
#define GPIO_TOUCH_PEN_IRQ_DFILTER          kPORT_PassiveFilterDisable
#define GPIO_TOUCH_PEN_IRQ_LOCK             kPORT_UnlockRegister
#define GPIO_TOUCH_PEN_IRQ_DRIVESTRENGTH    kPORT_LowDriveStrength
#define GPIO_TOUCH_PEN_IRQn                 PORTA_IRQn
/**@}*/

/**
 * \defgroup BQ27441 Battery Gauge Configuration
 * @{
 */
#define GPIO_GAUGE_GPOUT_CLK_ENABLE()     CLOCK_EnableClock(kCLOCK_PortA)
#define GPIO_GAUGE_GPOUT_PORT             PORTA
#define GPIO_GAUGE_GPOUT_GPIO             GPIOA
#define GPIO_GAUGE_GPOUT_PIN              PIN4_IDX
#define GPIO_GAUGE_GPOUT_ALT              kPORT_MuxAsGpio
#define GPIO_GAUGE_GPOUT_MODE             kGPIO_DigitalInput
#define GPIO_GAUGE_GPOUT_INT              kPORT_InterruptFallingEdge
#define GPIO_GAUGE_GPOUT_INIT_VAL         0
#define GPIO_GAUGE_GPOUT_PULL             kPORT_PullDisable
#define GPIO_GAUGE_GPOUT_SLEWRATE         kPORT_SlowSlewRate
#define GPIO_GAUGE_GPOUT_OPENDRAIN        kPORT_OpenDrainDisable
#define GPIO_GAUGE_GPOUT_DFILTER          kPORT_PassiveFilterDisable
#define GPIO_GAUGE_GPOUT_LOCK             kPORT_UnlockRegister
#define GPIO_GAUGE_GPOUT_DRIVESTRENGTH    kPORT_LowDriveStrength
#define GPIO_GAUGE_GPOUT_IRQn             PORTA_IRQn
/**@}*/

/**
 * \defgroup BQ24075 Battery Charger Configuration
 * @{
 */
#define GPIO_CHG_SYSOFF_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_CHG_SYSOFF_PORT           PORTC
#define GPIO_CHG_SYSOFF_GPIO           GPIOC
#define GPIO_CHG_SYSOFF_PIN            PIN7_IDX
#define GPIO_CHG_SYSOFF_ALT            kPORT_MuxAsGpio
#define GPIO_CHG_SYSOFF_MODE           kGPIO_DigitalOutput
#define GPIO_CHG_SYSOFF_INT            0 //No interrupt
#define GPIO_CHG_SYSOFF_INIT_VAL       0
#define GPIO_CHG_SYSOFF_PULL           kPORT_PullDisable
#define GPIO_CHG_SYSOFF_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_CHG_SYSOFF_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_CHG_SYSOFF_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_CHG_SYSOFF_LOCK           kPORT_UnlockRegister
#define GPIO_CHG_SYSOFF_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_CHG_CE_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_CHG_CE_PORT           PORTC
#define GPIO_CHG_CE_GPIO           GPIOC
#define GPIO_CHG_CE_PIN            PIN8_IDX
#define GPIO_CHG_CE_ALT            kPORT_MuxAsGpio
#define GPIO_CHG_CE_MODE           kGPIO_DigitalOutput
#define GPIO_CHG_CE_INT            0 //No interrupt
#define GPIO_CHG_CE_INIT_VAL       0
#define GPIO_CHG_CE_PULL           kPORT_PullDisable
#define GPIO_CHG_CE_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_CHG_CE_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_CHG_CE_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_CHG_CE_LOCK           kPORT_UnlockRegister
#define GPIO_CHG_CE_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_CHG_EN1_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_CHG_EN1_PORT           PORTC
#define GPIO_CHG_EN1_GPIO           GPIOC
#define GPIO_CHG_EN1_PIN            PIN12_IDX
#define GPIO_CHG_EN1_ALT            kPORT_MuxAsGpio
#define GPIO_CHG_EN1_MODE           kGPIO_DigitalOutput
#define GPIO_CHG_EN1_INT            0 //No interrupt
#define GPIO_CHG_EN1_INIT_VAL       0
#define GPIO_CHG_EN1_PULL           kPORT_PullDisable
#define GPIO_CHG_EN1_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_CHG_EN1_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_CHG_EN1_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_CHG_EN1_LOCK           kPORT_UnlockRegister
#define GPIO_CHG_EN1_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_CHG_EN2_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_CHG_EN2_PORT           PORTC
#define GPIO_CHG_EN2_GPIO           GPIOC
#define GPIO_CHG_EN2_PIN            PIN13_IDX
#define GPIO_CHG_EN2_ALT            kPORT_MuxAsGpio
#define GPIO_CHG_EN2_MODE           kGPIO_DigitalOutput
#define GPIO_CHG_EN2_INT            0 //No interrupt
#define GPIO_CHG_EN2_INIT_VAL       1
#define GPIO_CHG_EN2_PULL           kPORT_PullDisable
#define GPIO_CHG_EN2_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_CHG_EN2_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_CHG_EN2_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_CHG_EN2_LOCK           kPORT_UnlockRegister
#define GPIO_CHG_EN2_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_CHG_PGOOD_CLK_ENABLE()     CLOCK_EnableClock(kCLOCK_PortD)
#define GPIO_CHG_PGOOD_PORT             PORTD
#define GPIO_CHG_PGOOD_GPIO             GPIOD
#define GPIO_CHG_PGOOD_PIN              PIN6_IDX
#define GPIO_CHG_PGOOD_ALT              kPORT_MuxAsGpio
#define GPIO_CHG_PGOOD_MODE             kGPIO_DigitalInput
#define GPIO_CHG_PGOOD_INT              kPORT_InterruptFallingEdge
#define GPIO_CHG_PGOOD_INIT_VAL         0
#define GPIO_CHG_PGOOD_PULL             kPORT_PullDisable
#define GPIO_CHG_PGOOD_SLEWRATE         kPORT_SlowSlewRate
#define GPIO_CHG_PGOOD_OPENDRAIN        kPORT_OpenDrainDisable
#define GPIO_CHG_PGOOD_DFILTER          kPORT_PassiveFilterDisable
#define GPIO_CHG_PGOOD_LOCK             kPORT_UnlockRegister
#define GPIO_CHG_PGOOD_DRIVESTRENGTH    kPORT_LowDriveStrength

#define GPIO_CHG_CHARGING_CLK_ENABLE()     CLOCK_EnableClock(kCLOCK_PortB)
#define GPIO_CHG_CHARGING_PORT             PORTB
#define GPIO_CHG_CHARGING_GPIO             GPIOB
#define GPIO_CHG_CHARGING_PIN              PIN0_IDX
#define GPIO_CHG_CHARGING_ALT              kPORT_MuxAsGpio
#define GPIO_CHG_CHARGING_MODE             kGPIO_DigitalInput
#define GPIO_CHG_CHARGING_INT              kPORT_InterruptFallingEdge
#define GPIO_CHG_CHARGING_INIT_VAL         0
#define GPIO_CHG_CHARGING_PULL             kPORT_PullDisable
#define GPIO_CHG_CHARGING_SLEWRATE         kPORT_SlowSlewRate
#define GPIO_CHG_CHARGING_OPENDRAIN        kPORT_OpenDrainDisable
#define GPIO_CHG_CHARGING_DFILTER          kPORT_PassiveFilterDisable
#define GPIO_CHG_CHARGING_LOCK             kPORT_UnlockRegister
#define GPIO_CHG_CHARGING_DRIVESTRENGTH    kPORT_LowDriveStrength
/**@}*/

/**
 * \defgroup SPI FLash Configuration
 * @{
 */
#define GPIO_FLASH_RST_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_FLASH_RST_PORT           PORTC
#define GPIO_FLASH_RST_GPIO           GPIOC
#define GPIO_FLASH_RST_PIN            PIN16_IDX
#define GPIO_FLASH_RST_ALT            kPORT_MuxAsGpio
#define GPIO_FLASH_RST_MODE           kGPIO_DigitalOutput
#define GPIO_FLASH_RST_INT            0 //No interrupt
#define GPIO_FLASH_RST_INIT_VAL       0
#define GPIO_FLASH_RST_PULL           kPORT_PullDisable
#define GPIO_FLASH_RST_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_FLASH_RST_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_FLASH_RST_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_FLASH_RST_LOCK           kPORT_UnlockRegister
#define GPIO_FLASH_RST_DRIVESTRENGTH  kPORT_LowDriveStrength

#define GPIO_FLASH_WP_CLK_ENABLE()   CLOCK_EnableClock(kCLOCK_PortC)
#define GPIO_FLASH_WP_PORT           PORTC
#define GPIO_FLASH_WP_GPIO           GPIOC
#define GPIO_FLASH_WP_PIN            PIN17_IDX
#define GPIO_FLASH_WP_ALT            kPORT_MuxAsGpio
#define GPIO_FLASH_WP_MODE           kGPIO_DigitalOutput
#define GPIO_FLASH_WP_INT            0 //No interrupt
#define GPIO_FLASH_WP_INIT_VAL       0
#define GPIO_FLASH_WP_PULL           kPORT_PullDisable
#define GPIO_FLASH_WP_SLEWRATE       kPORT_SlowSlewRate
#define GPIO_FLASH_WP_OPENDRAIN      kPORT_OpenDrainDisable
#define GPIO_FLASH_WP_DFILTER        kPORT_PassiveFilterDisable
#define GPIO_FLASH_WP_LOCK           kPORT_UnlockRegister
#define GPIO_FLASH_WP_DRIVESTRENGTH  kPORT_LowDriveStrength


/**@}*/

#endif

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void SDHC_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void BOARD_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Calls initialization functions.
 * @return void
 */
void BOARD_InitBootPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void HOMEButton_InitPin(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void ECGACQ_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void TFT_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void DSPI0_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void DSPI0_DeinitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void DSPI1_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void DSPI1_DeinitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void DSPI2_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void DSPI2_DeinitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void I2C0_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void I2C0_DeinitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void I2C1_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void I2C1_DeinitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void AFE4400_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void TSC2007_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void BQ27441_InitPins(void);

/*************************************************************************************************/
/**
 * @brief Configures pin routing and optionally pin electrical features.
 * @return void
 */
void BQ4075_InitPins(void);
#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
