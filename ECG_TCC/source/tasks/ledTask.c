/**
 ******************************************************************************
 * @file    ledTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 August 2017
 * @brief   Module manage the heart beat LED
 ******************************************************************************
 */
#include <ledTask.h>
#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user.h"
#include "task_manager.h"
#if USE_PROTOTYPE

/*************************************************************************************************/
/**
 * @brief Task to blink a LED
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vToggleLEDTask(void *pvParameters);

/*************************************************************************************************/
static void vToggleLEDTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = ((pinTogglePeriod / 2) / portTICK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        vUser_ToggleFRDM_LED()
        ;
//		vUser_ToggleEXT_LED2()
//		;
    }
    vTaskManager_TaskDelete(NULL);
}

/*************************************************************************************************/
void vLedTaskInit(void)
{
    xTaskManager_TaskCreate(vToggleLEDTask, "ToggleLED", configMINIMAL_STACK_SIZE,
            NULL, ToggleLEDTask_PRIORITY, NULL);

}
/*************************************************************************************************/
#endif
