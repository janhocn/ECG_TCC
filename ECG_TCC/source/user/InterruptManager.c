/**
 ******************************************************************************
 * @file    InteruptManager.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    2 December 2017
 * @brief   Module to manage all interrupt handlers
 ******************************************************************************
 */
#include "InterruptManager.h"
#include "RTC.h"
#include "ecgAcqTask.h"
#include "spo2AcqTask.h"
#include "PhysButtonTask.h"
#include "fsl_sdmmc_host.h"
#include "fsl_port.h"
#include "TouchScreen.h"
#include "pin_mux.h"
#include "BatteryCharger.h"
/*************************************************************************************************/
/**
 * @brief ADC1 IRQ Handler
 */
void ADC1_IRQHandler(void)
{
#if	USE_ANALOGTOUCH
    vTouchScreen_ADCIRQHandler();
#endif
}

/*************************************************************************************************/
/**
 * @brief ADC0 IRQ Handler
 */
void ADC0_IRQHandler(void)
{
    vEcgAcqTask_IRQ_AnalogRead();
}

/*************************************************************************************************/
/**
 * @brief RTC Second IRQ Handler
 */
void RTC_Seconds_IRQHandler(void)
{
    vRTC_IRQ_RTC();
}

/*************************************************************************************************/
/**
 * @brief PORTA IRQ Handler
 */
void PORTA_IRQHandler(void)
{
    volatile uint32_t status;
    status = PORT_GetPinsInterruptFlags(PORTA);
#if !USE_ANALOGTOUCH
    vTouchScreen_PENIRQHandler(status);
#endif

    vBatteryCharger_GPOUT_IRQHandler(status);
}

/*************************************************************************************************/
/**
 * @brief PORTB IRQ Handler
 */
void PORTB_IRQHandler(void)
{
}

/*************************************************************************************************/
/**
 * @brief PORTC IRQ Handler
 */
void PORTC_IRQHandler(void)
{
    volatile uint32_t status;
    status = PORT_GetPinsInterruptFlags(PORTC);

#if USE_PROTOTYPE
    vButtonTask_IRQ_ButtonHome(status);
#endif
    vSPO2Task_IRQ_DataReady(status);
}

/*************************************************************************************************/
/**
 * @brief PORTE IRQ Handler
 */
void PORTE_IRQHandler(void)
{
    volatile uint32_t status;
    status = PORT_GetPinsInterruptFlags(PORTE);

    CardInsertDetectHandle(status);     //fsl_host SDCARD NXP Files
}
/*************************************************************************************************/
