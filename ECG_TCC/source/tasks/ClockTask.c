/**
 ******************************************************************************
 * @file    ClockTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 August 2017
 * @brief   Module manage the clock task
 ******************************************************************************
 */
#include "ClockTask.h"

#include "RTC.h"
#include "StatusBar.h"
#include "FreeRTOS.h"
#include "prioAssigner.h"
#include "task.h"
#include "queue.h"
#include "task_manager.h"
#include "displayRefreshTask.h"
#include "task_manager.h"
/** @var FreeRTOS Task Handler*/
TaskMngTaskHandle_t xRTCTaskHandle = NULL;

/*************************************************************************************************/
/**
 * @brief Task to Update the clock on the screen
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vClockTask(void *pvParameters);

/*************************************************************************************************/
static void vClockTask(void *pvParameters)
{
    DateTime_t dateTime;
    for (;;)
    {
        ui32TaskManager_TaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (bTaskManager_isSystemInitOK())
        {
            vRTC_GetDateTime(&dateTime.sec, &dateTime.min, &dateTime.hour, &dateTime.day, &dateTime.month,
                    &dateTime.year);
            vStatusBar_SetTime(dateTime.sec, dateTime.min, dateTime.hour);
            xTaskManager_TaskNotifyGive(xUpdateDisplayTaskGetHandler());
        }
    }
    vTaskManager_TaskDelete( NULL);
}

/*************************************************************************************************/
TaskMngTaskHandle_t xClockTaskGetHandler(void)
{
    return xRTCTaskHandle;
}

/*************************************************************************************************/
void vClockTaskInit(void)
{
    xTaskManager_TaskCreate(vClockTask, "vClockTask",
    configMINIMAL_STACK_SIZE + 300,
    NULL, RTCTask_PRIORITY, &xRTCTaskHandle);

    configASSERT(xRTCTaskHandle != NULL);
}
/*************************************************************************************************/
