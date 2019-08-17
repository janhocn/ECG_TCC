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

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
 PinsProfile:
 - !!product 'Pins v2.0'
 - !!processor 'MK64FN1M0xxx12'
 - !!package 'MK64FN1M0VLL12'
 - !!mcu_data 'ksdk2_0'
 - !!processor_version '1.0.1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART 0 transmit data source select: UART0_TX pin */

/*************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/*************************************************************************************************/
void BOARD_InitPins(void)
{

#if USE_PROTOTYPE
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, PIN22_IDX, kPORT_MuxAsGpio); /* PORTB22 (pin 68) is configured as LED */
    PORT_SetPinMux(PORTB, PIN23_IDX, kPORT_MuxAsGpio); /* PORTB23 (pin 69) is configured as LED */

    PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt3); /* PORTB16 (pin 62) is configured as UART0_RX */
    PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt3); /* PORTB17 (pin 63) is configured as UART0_TX */

    PORT_SetPinMux(PORTC, PIN16_IDX, kPORT_MuxAsGpio); /* PORTC16 (pin 90) is configured as External LED */

    //LEDs
    GPIO_PinInit(GPIOC, PIN16_IDX, &(gpio_pin_config_t )
            { kGPIO_DigitalOutput, (0) });
    GPIO_PinInit(GPIOB, PIN23_IDX, &(gpio_pin_config_t )
            { kGPIO_DigitalOutput, (0) });
    GPIO_PinInit(GPIOB, PIN22_IDX, &(gpio_pin_config_t )
            { kGPIO_DigitalOutput, (1) });

    HOMEButton_InitPin();
#endif

    SDHC_InitPins();

    SIM->SOPT5 = ((SIM->SOPT5 & (~(SIM_SOPT5_UART0TXSRC_MASK))) /* Mask bits to zero which are setting */
    | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX) /* UART 0 transmit data source select: UART0_TX pin */
    );
}

/*************************************************************************************************/
void SDHC_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortE); /* Port E Clock Gate Control: Clock enabled */
    const port_pin_config_t porte0_pin1_config = { kPORT_PullUp, /* Internal pull-up resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_HighDriveStrength, /* High drive strength is configured */
    kPORT_MuxAlt4, /* Pin is configured as SDHC0_D1 */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN0_IDX, &porte0_pin1_config); /* PORTE0 (pin 1) is configured as SDHC0_D1 */
    const port_pin_config_t porte1_pin2_config = { kPORT_PullUp, /* Internal pull-up resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_HighDriveStrength, /* High drive strength is configured */
    kPORT_MuxAlt4, /* Pin is configured as SDHC0_D0 */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN1_IDX, &porte1_pin2_config); /* PORTE1 (pin 2) is configured as SDHC0_D0 */
    const port_pin_config_t porte2_pin3_config = { kPORT_PullUp, /* Internal pull-up resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_HighDriveStrength, /* High drive strength is configured */
    kPORT_MuxAlt4, /* Pin is configured as SDHC0_DCLK */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN2_IDX, &porte2_pin3_config); /* PORTE2 (pin 3) is configured as SDHC0_DCLK */
    const port_pin_config_t porte3_pin4_config = { kPORT_PullUp, /* Internal pull-up resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_HighDriveStrength, /* High drive strength is configured */
    kPORT_MuxAlt4, /* Pin is configured as SDHC0_CMD */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN3_IDX, &porte3_pin4_config); /* PORTE3 (pin 4) is configured as SDHC0_CMD */
    const port_pin_config_t porte4_pin5_config = { kPORT_PullUp, /* Internal pull-up resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_HighDriveStrength, /* High drive strength is configured */
    kPORT_MuxAlt4, /* Pin is configured as SDHC0_D3 */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN4_IDX, &porte4_pin5_config); /* PORTE4 (pin 5) is configured as SDHC0_D3 */
    const port_pin_config_t porte5_pin6_config = { kPORT_PullUp, /* Internal pull-up resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_HighDriveStrength, /* High drive strength is configured */
    kPORT_MuxAlt4, /* Pin is configured as SDHC0_D2 */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN5_IDX, &porte5_pin6_config); /* PORTE5 (pin 6) is configured as SDHC0_D2 */
    const port_pin_config_t porte6_pin7_config = { kPORT_PullDown, /* Internal pull-down resistor is enabled */
    kPORT_FastSlewRate, /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable, /* Passive filter is disabled */
    kPORT_OpenDrainDisable, /* Open drain is disabled */
    kPORT_LowDriveStrength, /* Low drive strength is configured */
    kPORT_MuxAsGpio, /* Pin is configured as PTE6 */
    kPORT_UnlockRegister /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN6_IDX, &porte6_pin7_config); /* PORTE6 (pin 7) is configured as PTE6 */
}

/*************************************************************************************************/
void DSPI0_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortD); /* Port D Clock Gate Control: Clock enabled */

    PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_MuxAlt2); /* PORTD0 (pin 93) is configured as SPI0_PCS0 */
    PORT_SetPinMux(PORTD, PIN1_IDX, kPORT_MuxAlt2); /* PORTD1 (pin 94) is configured as SPI0_SCK */
    PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_MuxAlt2); /* PORTD2 (pin 95) is configured as SPI0_SOUT */
    PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAlt2); /* PORTD3 (pin 96) is configured as SPI0_SIN */
}

/*************************************************************************************************/
void DSPI0_DeinitPins(void)
{
    PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_PinDisabledOrAnalog); /* PORTD0 (pin 93) is disabled */
    PORT_SetPinMux(PORTD, PIN1_IDX, kPORT_PinDisabledOrAnalog); /* PORTD1 (pin 94) is configured as ADC0_SE5b */
    PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_PinDisabledOrAnalog); /* PORTD2 (pin 95) is disabled */
    PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_PinDisabledOrAnalog); /* PORTD3 (pin 96) is disabled */
}

/*************************************************************************************************/
void DSPI1_InitPins(void)
{
#if USE_PROTOTYPE
    CLOCK_EnableClock(kCLOCK_PortD); /* Port D Clock Gate Control: Clock enabled */

    PORT_SetPinMux(PORTD, PIN5_IDX, kPORT_MuxAlt7); /* PORTD5 (pin 98) is configured as SPI1_SCK */
    PORT_SetPinMux(PORTD, PIN6_IDX, kPORT_MuxAlt7); /* PORTD6 (pin 99) is configured as SPI1_SOUT */
    PORT_SetPinMux(PORTD, PIN7_IDX, kPORT_MuxAlt7); /* PORTD7 (pin 100) is configured as SPI1_SIN */
#else
    CLOCK_EnableClock(kCLOCK_PortB); /* Port B Clock Gate Control: Clock enabled */

    PORT_SetPinMux(PORTB, PIN11_IDX, kPORT_MuxAlt2); /* PORTB11 (pin 59) is configured as SPI1_SCK */
    PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt2); /* PORTB16 (pin 62) is configured as SPI1_SOUT */
    PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt2); /* PORTB17 (pin 63) is configured as SPI1_SIN */
#endif
}

/*************************************************************************************************/
void DSPI1_DeinitPins(void)
{
#if USE_PROTOTYPE
    PORT_SetPinMux(PORTD, PIN5_IDX, kPORT_PinDisabledOrAnalog); /* PORTD5 (pin 98) is configured as ADC0_SE6b */
    PORT_SetPinMux(PORTD, PIN6_IDX, kPORT_PinDisabledOrAnalog); /* PORTD6 (pin 99) is configured as ADC0_SE7b */
    PORT_SetPinMux(PORTD, PIN7_IDX, kPORT_PinDisabledOrAnalog); /* PORTD7 (pin 100) is disabled */
#else
    PORT_SetPinMux(PORTB, PIN11_IDX, kPORT_PinDisabledOrAnalog); /* PORTB11 (pin 59) is configured as ADC1_SE15 */
    PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_PinDisabledOrAnalog); /* PORTB16 (pin 62) is disabled */
    PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_PinDisabledOrAnalog); /* PORTB17 (pin 63) is disabled */
#endif
}

/*************************************************************************************************/
void DSPI2_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortB); /* Port B Clock Gate Control: Clock enabled */
    PORT_SetPinMux(PORTB, PIN20_IDX, kPORT_MuxAlt2); /* PORTB20 (pin 66) is configured as SPI0_PCS0 */
    PORT_SetPinMux(PORTB, PIN21_IDX, kPORT_MuxAlt2); /* PORTB21 (pin 67) is configured as SPI0_SCK */
    PORT_SetPinMux(PORTB, PIN22_IDX, kPORT_MuxAlt2); /* PORTB22 (pin 68) is configured as SPI0_SOUT */
    PORT_SetPinMux(PORTB, PIN23_IDX, kPORT_MuxAlt2); /* PORTB23 (pin 69) is configured as SPI0_SIN */
}

/*************************************************************************************************/
void DSPI2_DeinitPins(void)
{
    PORT_SetPinMux(PORTB, PIN20_IDX, kPORT_PinDisabledOrAnalog); /* PORTB20 (pin 66) is configured as SPI0_PCS0 */
    PORT_SetPinMux(PORTB, PIN21_IDX, kPORT_PinDisabledOrAnalog); /* PORTB21 (pin 67) is configured as SPI0_SCK */
    PORT_SetPinMux(PORTB, PIN22_IDX, kPORT_PinDisabledOrAnalog); /* PORTB22 (pin 68) is configured as SPI0_SOUT */
    PORT_SetPinMux(PORTB, PIN23_IDX, kPORT_PinDisabledOrAnalog); /* PORTB23 (pin 69) is configured as SPI0_SIN */
}

/*************************************************************************************************/
void HOMEButton_InitPin(void)
{
#if USE_PROTOTYPE
    //RST
    GPIO_HOME_BUTTON_CLK_ENABLE();

    port_pin_config_t portConfig = {
    GPIO_HOME_BUTTON_PULL,
    GPIO_HOME_BUTTON_SLEWRATE,
    GPIO_HOME_BUTTON_DFILTER,
    GPIO_HOME_BUTTON_OPENDRAIN,
    GPIO_HOME_BUTTON_DRIVESTRENGTH,
    GPIO_HOME_BUTTON_ALT,
    GPIO_HOME_BUTTON_LOCK };

    PORT_SetPinConfig(GPIO_HOME_BUTTON_PORT, GPIO_HOME_BUTTON_PIN, &portConfig);

    //Switch is connected as interrupt
    PORT_SetPinInterruptConfig(GPIO_HOME_BUTTON_PORT, GPIO_HOME_BUTTON_PIN,
    GPIO_HOME_BUTTON_INT);
    GPIO_PinInit(GPIO_HOME_BUTTON_GPIO, GPIO_HOME_BUTTON_PIN, &(gpio_pin_config_t )
            { GPIO_HOME_BUTTON_MODE, GPIO_HOME_BUTTON_INIT_VAL });
#endif
}

/*************************************************************************************************/
void ECGACQ_InitPins(void)
{
#if !USE_PROTOTYPE
    // POWER ENABLE
    GPIO_ECG_POWER_EN_CLK_ENABLE();

    port_pin_config_t portConfig =
    {
        GPIO_ECG_POWER_EN_PULL,
        GPIO_ECG_POWER_EN_SLEWRATE,
        GPIO_ECG_POWER_EN_DFILTER,
        GPIO_ECG_POWER_EN_OPENDRAIN,
        GPIO_ECG_POWER_EN_DRIVESTRENGTH,
        GPIO_ECG_POWER_EN_ALT,
        GPIO_ECG_POWER_EN_LOCK};

    PORT_SetPinConfig(GPIO_ECG_POWER_EN_PORT, GPIO_ECG_POWER_EN_PIN, &portConfig);

    GPIO_PinInit(GPIO_ECG_POWER_EN_GPIO, GPIO_ECG_POWER_EN_PIN, &(gpio_pin_config_t )
            {   GPIO_ECG_POWER_EN_MODE, GPIO_ECG_POWER_EN_INIT_VAL});
#else
    GPIO_ECG_IN_CLK_ENABLE();
    PORT_SetPinMux(GPIO_ECG_IN_PORT, GPIO_ECG_IN_PIN, GPIO_ECG_IN_ALT);
#endif
}

/*************************************************************************************************/
void TFT_InitPins(void)
{
    //RST
    GPIO_TFT_RST_CLK_ENABLE();

    port_pin_config_t portConfig = {
    GPIO_TFT_RST_PULL,
    GPIO_TFT_RST_SLEWRATE,
    GPIO_TFT_RST_DFILTER,
    GPIO_TFT_RST_OPENDRAIN,
    GPIO_TFT_RST_DRIVESTRENGTH,
    GPIO_TFT_RST_ALT,
    GPIO_TFT_RST_LOCK };

    PORT_SetPinConfig(GPIO_TFT_RST_PORT, GPIO_TFT_RST_PIN, &portConfig);

    GPIO_PinInit(GPIO_TFT_RST_GPIO, GPIO_TFT_RST_PIN, &(gpio_pin_config_t )
            { GPIO_TFT_RST_MODE, GPIO_TFT_RST_INIT_VAL });

    //BKL
    GPIO_TFT_BKL_CLK_ENABLE();

    portConfig.pullSelect = GPIO_TFT_BKL_PULL;
    portConfig.slewRate = GPIO_TFT_BKL_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_TFT_BKL_DFILTER;
    portConfig.openDrainEnable = GPIO_TFT_BKL_OPENDRAIN;
    portConfig.driveStrength = GPIO_TFT_BKL_DRIVESTRENGTH;
    portConfig.mux = GPIO_TFT_BKL_ALT;
    portConfig.lockRegister = GPIO_TFT_BKL_LOCK;

    PORT_SetPinConfig(GPIO_TFT_BKL_PORT, GPIO_TFT_BKL_PIN, &portConfig);

    GPIO_PinInit(GPIO_TFT_BKL_GPIO, GPIO_TFT_BKL_PIN, &(gpio_pin_config_t )
            { GPIO_TFT_BKL_MODE, GPIO_TFT_BKL_INIT_VAL });

    //DC
    GPIO_TFT_DC_CLK_ENABLE();

    portConfig.pullSelect = GPIO_TFT_DC_PULL;
    portConfig.slewRate = GPIO_TFT_DC_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_TFT_DC_DFILTER;
    portConfig.openDrainEnable = GPIO_TFT_DC_OPENDRAIN;
    portConfig.driveStrength = GPIO_TFT_DC_DRIVESTRENGTH;
    portConfig.mux = GPIO_TFT_DC_ALT;
    portConfig.lockRegister = GPIO_TFT_DC_LOCK;

    PORT_SetPinConfig(GPIO_TFT_DC_PORT, GPIO_TFT_DC_PIN, &portConfig);

    GPIO_PinInit(GPIO_TFT_DC_GPIO, GPIO_TFT_DC_PIN, &(gpio_pin_config_t )
            { GPIO_TFT_DC_MODE, GPIO_TFT_DC_INIT_VAL });

    //CS
    GPIO_TFT_CS_CLK_ENABLE();

    portConfig.pullSelect = GPIO_TFT_CS_PULL;
    portConfig.slewRate = GPIO_TFT_CS_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_TFT_CS_DFILTER;
    portConfig.openDrainEnable = GPIO_TFT_CS_OPENDRAIN;
    portConfig.driveStrength = GPIO_TFT_CS_DRIVESTRENGTH;
    portConfig.mux = GPIO_TFT_CS_ALT;
    portConfig.lockRegister = GPIO_TFT_CS_LOCK;

    PORT_SetPinConfig(GPIO_TFT_CS_PORT, GPIO_TFT_CS_PIN, &portConfig);

    GPIO_PinInit(GPIO_TFT_CS_GPIO, GPIO_TFT_CS_PIN, &(gpio_pin_config_t )
            { GPIO_TFT_CS_MODE, GPIO_TFT_CS_INIT_VAL });

#if USE_PROTOTYPE
    DSPI0_InitPins();
#else
    DSPI2_InitPins();
#endif
}

/*************************************************************************************************/
void BQ4075_InitPins(void)
{
    GPIO_CHG_SYSOFF_CLK_ENABLE();

    port_pin_config_t portConfig = {
    GPIO_CHG_SYSOFF_PULL,
    GPIO_CHG_SYSOFF_SLEWRATE,
    GPIO_CHG_SYSOFF_DFILTER,
    GPIO_CHG_SYSOFF_OPENDRAIN,
    GPIO_CHG_SYSOFF_DRIVESTRENGTH,
    GPIO_CHG_SYSOFF_ALT,
    GPIO_CHG_SYSOFF_LOCK };

    PORT_SetPinConfig(GPIO_CHG_SYSOFF_PORT, GPIO_CHG_SYSOFF_PIN, &portConfig);

    GPIO_PinInit(GPIO_CHG_SYSOFF_GPIO, GPIO_CHG_SYSOFF_PIN, &(gpio_pin_config_t )
            { GPIO_CHG_SYSOFF_MODE, GPIO_CHG_SYSOFF_INIT_VAL });

    GPIO_CHG_CE_CLK_ENABLE();

    portConfig.pullSelect = GPIO_CHG_CE_PULL;
    portConfig.slewRate = GPIO_CHG_CE_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_CHG_CE_DFILTER;
    portConfig.openDrainEnable = GPIO_CHG_CE_OPENDRAIN;
    portConfig.driveStrength = GPIO_CHG_CE_DRIVESTRENGTH;
    portConfig.mux = GPIO_CHG_CE_ALT;
    portConfig.lockRegister = GPIO_CHG_CE_LOCK;

    PORT_SetPinConfig(GPIO_CHG_CE_PORT, GPIO_CHG_CE_PIN, &portConfig);

    GPIO_PinInit(GPIO_CHG_CE_GPIO, GPIO_CHG_CE_PIN, &(gpio_pin_config_t )
            { GPIO_CHG_CE_MODE, GPIO_CHG_CE_INIT_VAL });

    GPIO_CHG_EN1_CLK_ENABLE();

    portConfig.pullSelect = GPIO_CHG_EN1_PULL;
    portConfig.slewRate = GPIO_CHG_EN1_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_CHG_EN1_DFILTER;
    portConfig.openDrainEnable = GPIO_CHG_EN1_OPENDRAIN;
    portConfig.driveStrength = GPIO_CHG_EN1_DRIVESTRENGTH;
    portConfig.mux = GPIO_CHG_EN1_ALT;
    portConfig.lockRegister = GPIO_CHG_EN1_LOCK;

    PORT_SetPinConfig(GPIO_CHG_EN1_PORT, GPIO_CHG_EN1_PIN, &portConfig);

    GPIO_PinInit(GPIO_CHG_EN1_GPIO, GPIO_CHG_EN1_PIN, &(gpio_pin_config_t )
            { GPIO_CHG_EN1_MODE, GPIO_CHG_EN1_INIT_VAL });

    GPIO_CHG_EN2_CLK_ENABLE();

    portConfig.pullSelect = GPIO_CHG_EN2_PULL;
    portConfig.slewRate = GPIO_CHG_EN2_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_CHG_EN2_DFILTER;
    portConfig.openDrainEnable = GPIO_CHG_EN2_OPENDRAIN;
    portConfig.driveStrength = GPIO_CHG_EN2_DRIVESTRENGTH;
    portConfig.mux = GPIO_CHG_EN2_ALT;
    portConfig.lockRegister = GPIO_CHG_EN2_LOCK;

    PORT_SetPinConfig(GPIO_CHG_EN2_PORT, GPIO_CHG_EN2_PIN, &portConfig);

    GPIO_PinInit(GPIO_CHG_EN2_GPIO, GPIO_CHG_EN2_PIN, &(gpio_pin_config_t )
            { GPIO_CHG_EN2_MODE, GPIO_CHG_EN2_INIT_VAL });

}

/*************************************************************************************************/
void BQ27441_InitPins(void)
{
    port_pin_config_t portConfig = {
    GPIO_GAUGE_GPOUT_PULL,
    GPIO_GAUGE_GPOUT_SLEWRATE,
    GPIO_GAUGE_GPOUT_DFILTER,
    GPIO_GAUGE_GPOUT_OPENDRAIN,
    GPIO_GAUGE_GPOUT_DRIVESTRENGTH,
    GPIO_GAUGE_GPOUT_ALT,
    GPIO_GAUGE_GPOUT_LOCK };

    GPIO_GAUGE_GPOUT_CLK_ENABLE();

    portConfig.pullSelect = GPIO_GAUGE_GPOUT_PULL;
    portConfig.slewRate = GPIO_GAUGE_GPOUT_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_GAUGE_GPOUT_DFILTER;
    portConfig.openDrainEnable = GPIO_GAUGE_GPOUT_OPENDRAIN;
    portConfig.driveStrength = GPIO_GAUGE_GPOUT_DRIVESTRENGTH;
    portConfig.mux = GPIO_GAUGE_GPOUT_ALT;
    portConfig.lockRegister = GPIO_GAUGE_GPOUT_LOCK;

    PORT_SetPinConfig(GPIO_GAUGE_GPOUT_PORT, GPIO_GAUGE_GPOUT_PIN, &portConfig);
    PORT_SetPinInterruptConfig(GPIO_GAUGE_GPOUT_PORT, GPIO_GAUGE_GPOUT_PIN,
            GPIO_GAUGE_GPOUT_INT);
    GPIO_PinInit(GPIO_GAUGE_GPOUT_GPIO, GPIO_GAUGE_GPOUT_PIN, &(gpio_pin_config_t )
            { GPIO_GAUGE_GPOUT_MODE, GPIO_GAUGE_GPOUT_INIT_VAL });
    I2C1_InitPins();
}

/*************************************************************************************************/
void TSC2007_InitPins(void)
{

    port_pin_config_t portConfig = {
    GPIO_TOUCH_PEN_IRQ_PULL,
    GPIO_TOUCH_PEN_IRQ_SLEWRATE,
    GPIO_TOUCH_PEN_IRQ_DFILTER,
    GPIO_TOUCH_PEN_IRQ_OPENDRAIN,
    GPIO_TOUCH_PEN_IRQ_DRIVESTRENGTH,
    GPIO_TOUCH_PEN_IRQ_ALT,
    GPIO_TOUCH_PEN_IRQ_LOCK };

    GPIO_TOUCH_PEN_IRQ_CLK_ENABLE();

    portConfig.pullSelect = GPIO_TOUCH_PEN_IRQ_PULL;
    portConfig.slewRate = GPIO_TOUCH_PEN_IRQ_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_TOUCH_PEN_IRQ_DFILTER;
    portConfig.openDrainEnable = GPIO_TOUCH_PEN_IRQ_OPENDRAIN;
    portConfig.driveStrength = GPIO_TOUCH_PEN_IRQ_DRIVESTRENGTH;
    portConfig.mux = GPIO_TOUCH_PEN_IRQ_ALT;
    portConfig.lockRegister = GPIO_TOUCH_PEN_IRQ_LOCK;

    PORT_SetPinConfig(GPIO_TOUCH_PEN_IRQ_PORT, GPIO_TOUCH_PEN_IRQ_PIN, &portConfig);
    PORT_SetPinInterruptConfig(GPIO_TOUCH_PEN_IRQ_PORT, GPIO_TOUCH_PEN_IRQ_PIN,
            GPIO_TOUCH_PEN_IRQ_INT);
    GPIO_PinInit(GPIO_TOUCH_PEN_IRQ_GPIO, GPIO_TOUCH_PEN_IRQ_PIN, &(gpio_pin_config_t )
            { GPIO_TOUCH_PEN_IRQ_MODE, GPIO_TOUCH_PEN_IRQ_INIT_VAL });

    I2C0_InitPins();
}

/*************************************************************************************************/
void AFE4400_InitPins(void)
{
    //RST
    GPIO_AFE4400_RST_CLK_ENABLE();

    port_pin_config_t portConfig = {
    GPIO_AFE4400_RST_PULL,
    GPIO_AFE4400_RST_SLEWRATE,
    GPIO_AFE4400_RST_DFILTER,
    GPIO_AFE4400_RST_OPENDRAIN,
    GPIO_AFE4400_RST_DRIVESTRENGTH,
    GPIO_AFE4400_RST_ALT,
    GPIO_AFE4400_RST_LOCK };

    PORT_SetPinConfig(GPIO_AFE4400_RST_PORT, GPIO_AFE4400_RST_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_RST_GPIO, GPIO_AFE4400_RST_PIN, &(gpio_pin_config_t )
            { GPIO_AFE4400_RST_MODE, GPIO_AFE4400_RST_INIT_VAL });

    //PDN
    GPIO_AFE4400_PDN_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_PDN_PULL;
    portConfig.slewRate = GPIO_AFE4400_PDN_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_PDN_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_PDN_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_PDN_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_PDN_ALT;
    portConfig.lockRegister = GPIO_AFE4400_PDN_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_PDN_PORT, GPIO_AFE4400_PDN_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_PDN_GPIO, GPIO_AFE4400_PDN_PIN, &(gpio_pin_config_t )
            { GPIO_AFE4400_PDN_MODE, GPIO_AFE4400_PDN_INIT_VAL });

    //CS
    GPIO_AFE4400_CS_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_CS_PULL;
    portConfig.slewRate = GPIO_AFE4400_CS_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_CS_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_CS_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_CS_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_CS_ALT;
    portConfig.lockRegister = GPIO_AFE4400_CS_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_CS_PORT, GPIO_AFE4400_CS_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_CS_GPIO, GPIO_AFE4400_CS_PIN, &(gpio_pin_config_t )
            { GPIO_AFE4400_CS_MODE, GPIO_AFE4400_CS_INIT_VAL });

    //ADC Ready
    GPIO_AFE4400_ADCRDY_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_ADCRDY_PULL;
    portConfig.slewRate = GPIO_AFE4400_ADCRDY_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_ADCRDY_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_ADCRDY_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_ADCRDY_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_ADCRDY_ALT;
    portConfig.lockRegister = GPIO_AFE4400_ADCRDY_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_ADCRDY_PORT, GPIO_AFE4400_ADCRDY_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_ADCRDY_GPIO, GPIO_AFE4400_ADCRDY_PIN, &(gpio_pin_config_t )
            { GPIO_AFE4400_ADCRDY_MODE, GPIO_AFE4400_ADCRDY_INIT_VAL });

#if !USE_PROTOTYPE
    //PDALM
    GPIO_AFE4400_PDALM_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_PDALM_PULL;
    portConfig.slewRate = GPIO_AFE4400_PDALM_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_PDALM_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_PDALM_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_PDALM_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_PDALM_ALT;
    portConfig.lockRegister = GPIO_AFE4400_PDALM_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_PDALM_PORT, GPIO_AFE4400_PDALM_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_PDALM_GPIO, GPIO_AFE4400_PDALM_PIN, &(gpio_pin_config_t )
            {   GPIO_AFE4400_PDALM_MODE, GPIO_AFE4400_PDALM_INIT_VAL});

    //DIAGEND
    GPIO_AFE4400_DIAGEND_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_DIAGEND_PULL;
    portConfig.slewRate = GPIO_AFE4400_DIAGEND_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_DIAGEND_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_DIAGEND_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_DIAGEND_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_DIAGEND_ALT;
    portConfig.lockRegister = GPIO_AFE4400_DIAGEND_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_DIAGEND_PORT, GPIO_AFE4400_DIAGEND_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_DIAGEND_GPIO, GPIO_AFE4400_DIAGEND_PIN, &(gpio_pin_config_t )
            {   GPIO_AFE4400_DIAGEND_MODE, GPIO_AFE4400_DIAGEND_INIT_VAL});

    //LEDALM
    GPIO_AFE4400_LEDALM_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_LEDALM_PULL;
    portConfig.slewRate = GPIO_AFE4400_LEDALM_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_LEDALM_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_LEDALM_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_LEDALM_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_LEDALM_ALT;
    portConfig.lockRegister = GPIO_AFE4400_LEDALM_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_LEDALM_PORT, GPIO_AFE4400_LEDALM_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_LEDALM_GPIO, GPIO_AFE4400_LEDALM_PIN, &(gpio_pin_config_t )
            {   GPIO_AFE4400_LEDALM_MODE, GPIO_AFE4400_LEDALM_INIT_VAL});

    //POWER ENABLE
    GPIO_AFE4400_POWER_EN_CLK_ENABLE();

    portConfig.pullSelect = GPIO_AFE4400_POWER_EN_PULL;
    portConfig.slewRate = GPIO_AFE4400_POWER_EN_SLEWRATE;
    portConfig.passiveFilterEnable = GPIO_AFE4400_POWER_EN_DFILTER;
    portConfig.openDrainEnable = GPIO_AFE4400_POWER_EN_OPENDRAIN;
    portConfig.driveStrength = GPIO_AFE4400_POWER_EN_DRIVESTRENGTH;
    portConfig.mux = GPIO_AFE4400_POWER_EN_ALT;
    portConfig.lockRegister = GPIO_AFE4400_POWER_EN_LOCK;

    PORT_SetPinConfig(GPIO_AFE4400_POWER_EN_PORT, GPIO_AFE4400_POWER_EN_PIN, &portConfig);

    GPIO_PinInit(GPIO_AFE4400_POWER_EN_GPIO, GPIO_AFE4400_POWER_EN_PIN, &(gpio_pin_config_t )
            {   GPIO_AFE4400_POWER_EN_MODE, GPIO_AFE4400_POWER_EN_INIT_VAL});
#endif

    DSPI1_InitPins();
}

/*************************************************************************************************/
void I2C0_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortE);                           /* Port E Clock Gate Control: Clock enabled */

    const port_pin_config_t porte24_pin31_config = {
      kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
      kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
      kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
      kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
      kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
      kPORT_MuxAlt5,                                           /* Pin is configured as I2C0_SCL */
      kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN24_IDX, &porte24_pin31_config); /* PORTE24 (pin 31) is configured as I2C0_SCL */
    const port_pin_config_t porte25_pin32_config = {
      kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
      kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
      kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
      kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
      kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
      kPORT_MuxAlt5,                                           /* Pin is configured as I2C0_SDA */
      kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTE, PIN25_IDX, &porte25_pin32_config); /* PORTE25 (pin 32) is configured as I2C0_SDA */
}

/*************************************************************************************************/
void I2C0_DeinitPins(void)
{
    PORT_SetPinMux(PORTE, PIN24_IDX, kPORT_PinDisabledOrAnalog); /* PORTE24 (pin 31) is configured as ADC0_SE17 */
    PORT_SetPinMux(PORTE, PIN25_IDX, kPORT_PinDisabledOrAnalog); /* PORTE25 (pin 32) is configured as ADC0_SE18 */
}

/*************************************************************************************************/
void I2C1_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortC); /* Port C Clock Gate Control: Clock enabled */

    PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_MuxAlt2); /* PORTC10 (pin 82) is configured as I2C1_SCL */
    PORT_SetPinMux(PORTC, PIN11_IDX, kPORT_MuxAlt2); /* PORTC11 (pin 83) is configured as I2C1_SDA */
}

/*************************************************************************************************/
void I2C1_DeinitPins(void)
{
    PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_PinDisabledOrAnalog); /* PORTC10 (pin 82) is configured as ADC1_SE6b */
    PORT_SetPinMux(PORTC, PIN11_IDX, kPORT_PinDisabledOrAnalog); /* PORTC11 (pin 83) is configured as ADC1_SE7b */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
