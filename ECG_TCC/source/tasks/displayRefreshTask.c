/**
 ******************************************************************************
 * @file    displayRefreshTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 August 2017
 * @brief   Module manage the display refresh task
 ******************************************************************************
 */

#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "task_manager.h"
#include "displayRefreshTask.h"
#include "task_manager.h"
#include "Display.h"
#include "GUI.h"
#include "task_manager.h"
/** @var FreeRTOS Task Handler*/
TaskMngTaskHandle_t xUpdateDisplayHandle = NULL;

/*************************************************************************************************/
/**
 * @brief Task to refresh the display
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vUpdateDisplayTask(void *pvParameters);

/*************************************************************************************************/
static void vUpdateDisplayTask(void *pvParameters)
{

    for (;;)
    {
        switch (vDisplay_GetState())
        {
            case _WINDOW_ECGGraph:
            case _WINDOW_SPO2Graph:
                ui32TaskManager_TaskNotifyTake(pdFALSE,portMAX_DELAY);
                vDisplay_RefreshDisplay();
                break;

            default:
                vDisplay_RefreshDisplay();
                GUI_X_Delay(updateDisplayPeriodms);
                break;
        }
    }
    vTaskManager_TaskDelete( NULL);
}

/*************************************************************************************************/
void vUpdateDisplayTaskSuspend(void)
{
    if (xUpdateDisplayHandle)
        vTaskSuspend(xUpdateDisplayHandle);
}

/*************************************************************************************************/
void vUpdateDisplayTaskResume(void)
{
    if (xUpdateDisplayHandle)
        vTaskResume(xUpdateDisplayHandle);
}

/*************************************************************************************************/
TaskMngTaskHandle_t xUpdateDisplayTaskGetHandler(void)
{
    return xUpdateDisplayHandle;
}

/*************************************************************************************************/
void vUpdateDisplayTaskInit(void)
{
    xTaskManager_TaskCreate(vUpdateDisplayTask, "UpdateDisplayTask",
    configMINIMAL_STACK_SIZE + 250,
    NULL, UpdateDisplayTask_PRIORITY, &xUpdateDisplayHandle);

    configASSERT(xUpdateDisplayHandle != NULL);
}
/*************************************************************************************************/
