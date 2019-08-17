/**
 ******************************************************************************
 * @file    RTC.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 September 2017
 * @brief   Internal RTC Module
 ******************************************************************************
 */

#include "board.h"
#include "fsl_rtc.h"

#include "clock_config.h"
#include "task_manager.h"
#include "InterruptManager.h"
#include "prioAssigner.h"
#include "ClockTask.h"

/*************************************************************************************************/
/**
 * @brief Enable the RTC 32KHz oscillator
 * @return void
 */
static void BOARD_SetRtcClockSource(void);

/*************************************************************************************************/
/**
 * @var RTC date/time structure
 */
static rtc_datetime_t date;

/*************************************************************************************************/
static void BOARD_SetRtcClockSource(void)
{
    RTC->CR |= RTC_CR_OSCE_MASK;
}

/*************************************************************************************************/
void vRTC_IRQ_RTC(void)
{
    TaskMngTaskHandle_t taskHandler;
    taskHandler = xClockTaskGetHandler();
    if (taskHandler)
    {
        xTaskManager_TaskNotifyGive(taskHandler);
    }
}

/*************************************************************************************************/
void vRTC_Init(void)
{
    rtc_config_t rtcConfig;

    RTC_GetDefaultConfig(&rtcConfig);
    RTC_Init(RTC, &rtcConfig);

    /* Select RTC clock source */
    BOARD_SetRtcClockSource();

    /* RTC time counter has to be stopped before setting the date & time in the TSR register */
    RTC_StopTimer(RTC);
    /* Enable RTC alarm interrupt */
    RTC_EnableInterrupts(RTC, kRTC_AlarmInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(RTC_Seconds_IRQn);
    NVIC_SetPriority(RTC_Seconds_IRQn, RTC_INT_PRIO);
    /* Start the RTC time counter */
    RTC_StartTimer(RTC);

    RTC_EnableInterrupts(RTC, kRTC_SecondsInterruptEnable);
}

/*************************************************************************************************/
void vRTC_GetDateTime(uint8_t *sec, uint8_t *min, uint8_t *hour, uint8_t *day, uint8_t *month, uint16_t *year)
{

    RTC_GetDatetime(RTC, &date);

    *sec = date.second;
    *min = date.minute;
    *hour = date.hour;

    *day = date.day;
    *month = date.month;
    *year = date.year;
}

/*************************************************************************************************/
void vRTC_SetDate(uint8_t day, uint8_t month, uint16_t year)
{

    date.day = day;
    date.month = month;
    date.year = year;

    RTC_StopTimer(RTC);
    RTC_SetDatetime(RTC, &date);
    RTC_StartTimer(RTC);

}

/*************************************************************************************************/
void vRTC_SetTime(uint8_t sec, uint8_t min, uint8_t hour)
{

    date.second = sec;
    date.minute = min;
    date.hour = hour;

    RTC_StopTimer(RTC);
    RTC_SetDatetime(RTC, &date);
    RTC_StartTimer(RTC);
}
/*************************************************************************************************/
