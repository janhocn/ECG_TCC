/**
 ******************************************************************************
 * @file    prioAssigner.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    01 January 2018
 * @brief   Module to manage Settings Window
 ******************************************************************************
 */
#include "FreeRTOSConfig.h"

typedef enum _RTOSPriorities
{
    _PRIO_RTOS0 = 0x00,
    _PRIO_RTOS1,
    _PRIO_RTOS2,
    _PRIO_RTOS3,
    _PRIO_RTOS4,
    _PRIO_RTOS5,
    _PRIO_RTOS6,
    _PRIO_RTOS7,
    _PRIO_RTOSMAX = configMAX_PRIORITIES,
} RTOSPriorities;

typedef enum _NVICPriorities
{
    _PRIO_NVICBASE = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
    _PRIO_NVIC0 = 0x00,
    _PRIO_NVIC1,
    _PRIO_NVIC2,
    _PRIO_NVIC3,
    _PRIO_NVIC4,
    _PRIO_NVIC5,
    _PRIO_NVIC6,
    _PRIO_NVIC7,
    _PRIO_NVIC8,
    _PRIO_NVIC9,
    _PRIO_NVIC10,
    _PRIO_NVIC11,
    _PRIO_NVIC12,
    _PRIO_NVIC13,
    _PRIO_NVIC14,
    _PRIO_NVIC15,
	_PRIO_NVICMAX = _PRIO_NVIC15
} NVICPriorities;

/* Priorty Assignment for Interrupts */
#define ADC_ECG_INT_PRIO _PRIO_NVICBASE
#define AFE4400_SPI_INT_PRIO _PRIO_NVICBASE
#define SDCARD_DETECT_INT_PRIO _PRIO_NVICBASE + 1
#define SDCARD_INT_PRIO _PRIO_NVICBASE + 1
#define RTC_INT_PRIO _PRIO_NVICBASE + 2
#define PORTA_INT_PRIO _PRIO_NVICBASE + 3
#define PORTC_INT_PRIO _PRIO_NVICBASE + 3
#define TSC2007_DMA_PRIO _PRIO_NVICBASE + 4
#define ADC_TOUCH_PRIO _PRIO_NVICBASE + 4
#define BATCHARGER_I2C_PRIO _PRIO_NVICBASE + 5
#define HX8357_SPI_INT_PRIO _PRIO_NVICMAX

/* Priorty Assignment for FreeRTOS Tasks */
#define physButtonTask_PRIORITY (_PRIO_RTOSMAX)
#define EcgProcessTask_PRIORITY (_PRIO_RTOS7)
#define SpO2ProcessTask_PRIORITY (_PRIO_RTOS7)
#define EcgPrintHRTask_PRIORITY (_PRIO_RTOS7)
#define ToggleLEDTask_PRIORITY (_PRIO_RTOS7)
#define RTCTask_PRIORITY (_PRIO_RTOS7)
#define EcgDrawOnGraphTask_PRIORITY (_PRIO_RTOS6)
#define SpO2DrawOnGraphTask_PRIORITY (_PRIO_RTOS6)
#define LogTask_PRIORITY (_PRIO_RTOS7)
#define powerManagerTask_PRIORITY (_PRIO_RTOS5)
#define touchUpdateTask_PRIORITY (_PRIO_RTOS3)
#define UpdateDisplayTask_PRIORITY (_PRIO_RTOS4)

