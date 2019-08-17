/**
 ******************************************************************************
 * @file    ecgAcqTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 August 2017
 * @brief   Module to manage the ecg acquisition tasks
 ******************************************************************************
 */

#ifndef TASKS_ECGACQTASK_H_
#define TASKS_ECGACQTASK_H_

#include "fsl_adc16.h"
#include "fsl_pit.h"
#include "fsl_smc.h"
#include "fsl_pmc.h"
#include "arm_math.h"

/*************************************************************************************************/
/**
 * @def If 1 plot the QRS detection on screen (!be carefull)
 */
#define SHOW_QRS_ON_DISPLAY 0

/*************************************************************************************************/
/**
 * @def If 1 use the LPTMR as the adc timer trigger
 */
#define USE_LPTMR_AS_TIMER  0

/*************************************************************************************************/
/**
 * @def ADC reading sample period in microseconds
 */
#define adcSamplePeriodus 300

/*************************************************************************************************/
/**
 * @def ADC reading sample period in milliseconds
 */
#define adcSamplePeriodms (adcSamplePeriodus/1000.0f)

/*************************************************************************************************/
/**
 * @def QRS sampling period in milliseconds
 */
#define adcQRSSamplePeriodms (5.1)

/*************************************************************************************************/
/**
 * @def Numeber of samples to downsamplig the QRS detection to ~200Hz
 */
#define adcNumSamplesQRS     (uint16_t)(adcQRSSamplePeriodms/(adcSamplePeriodms))

/*************************************************************************************************/
/**
 * @def Moving window time in seconds
 */
#define timeWindowMovingAVG  (0.150)

/*************************************************************************************************/
/**
 * @def Time in millisecons per sample per pixel
 */
#define timePerPixel 6

/*************************************************************************************************/
/**
 * @def Number of samples to be processed before sending to the screen
 */
#define SAMPLES_PER_BLOCK 10

/*************************************************************************************************/
/**
 * @def Number of samples to be processed per pixel
 */
#define SAMPLES_PER_PIXEL (uint16_t) (timePerPixel / adcSamplePeriodms)

/** ADC/DAC Definitions. */
#define ADC0_BASE_ADDR ADC0
#define ADC0_CH 12U
#define ADC_CHANNEL_GROUP 0U
#define ADC_IRQ_ID ADC0_IRQn

#if USE_LPTMR_AS_TIMER
#define TIMER_IRQ_ID LPTMR0_IRQn
#define TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
#else
#define TIMER_IRQ_ID PIT0_IRQn
#define TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#endif

/*************************************************************************************************/
/**
 * @brief ADC ECG IRQ Handler (Reads sample by sample of ECG and performs the QRS detection at ~200Hz)
 * @return void
 */
void vEcgAcqTask_IRQ_AnalogRead(void);

/*************************************************************************************************/
/**
 * @brief Initializes all related Tasks
 * @return void
 */
void vEcgAcqTaskInit(void);

/*************************************************************************************************/
/**
 * @brief Suspend all related Tasks and interrupts
 * @return void
 */
void vEcgAcqTaskSuspend(void);

/*************************************************************************************************/
/**
 * @brief Resume all related Tasks and interrupts
 * @return void
 */
void vEcgAcqTaskResume(void);

#endif /* TASKS_ECGACQTASK_H_ */
