/**
 ******************************************************************************
 * @file    touchUpdateTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 September 2017
 * @brief   Module to manage the touch update
 ******************************************************************************
 */
#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "touchUpdateTask.h"

#include "TouchScreen.h"
#include "task_manager.h"
#include "GUI.h"
#include "LCD.h"
#include "LCDConf.h"
#include "task_manager.h"

/*************************************************************************************************/
/**
 * @brief Task to update the touch panel states
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vTouchTask(void *pvParameters);

/*************************************************************************************************/
static void vTouchTask(void *pvParameters)
{
    const portTickType xUpdateFrequency = (refreshTouchPeriod / portTICK_PERIOD_MS);

    vTouchScreen_Init();

    for (;;)
    {
        if (bTaskManager_isSystemInitOK())
        {
            vTouchScreen_Exec();
        }
        vTaskManager_Delay(xUpdateFrequency);
    }
    vTaskManager_TaskDelete( NULL);
}

/*************************************************************************************************/
void vTouchTaskInit(void)
{
    xTaskManager_TaskCreate(vTouchTask, "TouchTask",
    configMINIMAL_STACK_SIZE,
    NULL, touchUpdateTask_PRIORITY, NULL);
}
/*************************************************************************************************/
