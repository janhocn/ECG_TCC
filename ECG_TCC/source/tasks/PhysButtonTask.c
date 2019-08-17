/**
 ******************************************************************************
 * @file    PhysButtonTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    12 September 2017
 * @brief   Module to manage home button
 ******************************************************************************
 */

#include "PhysButtonTask.h"
#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Display.h"
#include "board.h"
#include "pin_mux.h"
#include "MK64F12.h"
#include "task_manager.h"
#include "InterruptManager.h"
#include "task_manager.h"
#include "displayRefreshTask.h"
#include "task_manager.h"
#include "SettingsScreen.h"

#if USE_PROTOTYPE

/*************************************************************************************************/
/**
 * @brief Task to Process the button pressed state
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vButtonTask(void *pvParameters);

/** @var Semaphore Handler*/
static TaskMngSemaphoreHandle_t xButtonSemaphore;

/*************************************************************************************************/
void vButtonTask_IRQ_ButtonHome(uint32_t status)
{
    if (status == (1 << GPIO_HOME_BUTTON_PIN))
    {
        GPIO_ClearPinsInterruptFlags(GPIOC, 1 << GPIO_HOME_BUTTON_PIN);
#if	USE_ANALOGTOUCH
        if(!bSettings_isInTouchCalibration())
#endif
        {
        	xTaskManager_SemaphoreGive(xButtonSemaphore);
        }
    }
}

/*************************************************************************************************/
static void vButtonTask(void *pvParameters)
{
    if (xButtonSemaphore == NULL)
    {
        xButtonSemaphore = xTaskManager_SemaphoreCreate();
        xTaskManager_SemaphoreTake(xButtonSemaphore, portMAX_DELAY);
    }

    for (;;)
    {
        /* Block on the semaphore to wait for an interrupt event.  The semaphore
         is 'given' from vButtonISRHandler() below.  Using portMAX_DELAY as the
         block time will cause the task to block indefinitely provided
         INCLUDE_vTaskSuspend is set to 1 in FreeRTOSConfig.h. */
        if (xTaskManager_SemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdTRUE)
        {
            /* The button must have been pushed for this line to be executed.*/
            vDisplay_SetWindow(_WINDOW_HOME);
            xTaskManager_TaskNotifyGive(xUpdateDisplayTaskGetHandler());
        }
        /* Wait a short time then clear any pending button pushes as a crude
         method of debouncing the switch.  xSemaphoreTake() uses a block time of
         zero this time so it returns immediately rather than waiting for the
         interrupt to occur. */
        vTaskManager_Delay((200 / portTICK_RATE_MS));
        xTaskManager_SemaphoreTake(xButtonSemaphore, 0);
    }
    vTaskManager_TaskDelete( NULL);
}

/*************************************************************************************************/
void vPhysButtonTaskInit(void)
{
    xTaskManager_TaskCreate(vButtonTask, "ButtonTask",
    configMINIMAL_STACK_SIZE,
            NULL, physButtonTask_PRIORITY, NULL);
    /****************Enable interrupt on PORTC**********************/
    EnableIRQ(PORTC_IRQn);
    NVIC_SetPriority(PORTC_IRQn, PORTC_INT_PRIO);
    /********************************************************************/
}

#endif
/*************************************************************************************************/
