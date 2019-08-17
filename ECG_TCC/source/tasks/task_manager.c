/**
 ******************************************************************************
 * @file    task_manager.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 August 2017
 * @brief   Module that provides functions to work with the RTOS tasks
 ******************************************************************************
 */

#include "task_manager.h"

#include "FileSystem.h"
#include "MCU_Flash.h"
#include "PWM.h"
#include "RTC.h"
#include "BatteryCharger.h"
#include "SPO2.h"
#include "ClockTask.h"
#include "powerManagerTask.h"
#include "PhysButtonTask.h"
#include "touchUpdateTask.h"
#include "displayRefreshTask.h"
#include "spo2AcqTask.h"
#include "ecgAcqTask.h"
#include "ledTask.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "GUI.h"
#include "Display.h"
#include "Messages.h"
#include "user.h"
#include "logTask.h"

static bool isSystemInitialized = false;

static TimerHandle_t xTimerReset;
/*************************************************************************************************/
/**
 * @brief Task to initialize all tasks and other system functions
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vTaskManagerInitAll(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief This function is used to verify if the RTOS function is being called from within an interrupt
 * @return xPSR Register.
 */
__STATIC_INLINE uint32_t __ReadExceptionNumber(void)
{
    uint32_t u32_xPSR;

    u32_xPSR = __get_xPSR();

    // Exception Number bit 8 ao bit 0
    u32_xPSR = u32_xPSR & 0x000001FF;

    return (u32_xPSR);
}

/*************************************************************************************************/
static void vTaskManagerInitAll(void *pvParameters)
{
    status_t status = kStatus_SDCardNotInserted;
    vUser_InitDMA();
    vRTC_Init();
    vPWM_Init();
    stMCU_Flash_Init();
    vMessages_Init();
    vDisplay_GUI_Init();
    AFE4400_InitPins();

    GUI_SetFont(&GUI_Font16B_1);
    GUI_DispStringHCenterAt(xMessage_getMessageInitialization()->title, (LCD_GetXSize() - 1) / 2, 0);
    GUI_DispNextLine();
    GUI_DispChars('*', 100);

    GUI_DispString(xMessage_getMessageInitialization()->initSDCard);

    GUI_DispString(xMessage_getMessageInitialization()->insertSDCard);

    xTimerStart(xTimerReset, 0);

    do
    {
        status = stFileSystem_Init();
    }
    while (status == kStatus_SDCardNotInserted);

    xTimerStop(xTimerReset, 0);
    xTimerDelete(xTimerReset, 0);

    if (status == kStatus_Success)
    {
        GUI_DispString(xMessage_getMessageInitialization()->SDCardOK);
    }
    else
    {
        GUI_DispString(xMessage_getMessageInitialization()->SDCardNOK);
    }

    vLogTaskInit();

#if USE_PROTOTYPE
    vLedTaskInit();
#endif

    vPowerManagerTaskInit();

    GUI_DispString(xMessage_getMessageInitialization()->initClock);
#if USE_PROTOTYPE
    vPhysButtonTaskInit();
#endif
    vClockTaskInit();

    GUI_DispString(xMessage_getMessageInitialization()->initDisplay);
    vUpdateDisplayTaskInit();
    vTouchTaskInit();

    GUI_DispString(xMessage_getMessageInitialization()->initECG);
    vEcgAcqTaskInit();
    vEcgAcqTaskSuspend();

    GUI_DispString(xMessage_getMessageInitialization()->initSPO2);
    vSPO2AcqTaskInit();
    vSPO2AcqTaskSuspend();

    GUI_DispString(xMessage_getMessageInitialization()->initDone);
    vTaskManager_Delay(1000);

    vDisplay_WindowsInit();

    isSystemInitialized = true;

    vTaskDelete(NULL);
}
/*************************************************************************************************/
bool bTaskManager_isSystemInitOK(void)
{
    return isSystemInitialized;
}

/*************************************************************************************************/
void vTimerCallback(TimerHandle_t xTimer)
{
    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT(xTimer);
    NVIC_SystemReset();
}

/*************************************************************************************************/
void vTaskManagerInit(void)
{
    xTimerReset = xTimerCreate("Reset", 13000, pdFALSE, (void *) 0, vTimerCallback);
    xTaskCreate(vTaskManagerInitAll, "InitAll", configMINIMAL_STACK_SIZE + 8000,
    NULL, configMAX_PRIORITIES - 1, NULL);

    vTaskStartScheduler();
}

/*************************************************************************************************/
void vApplicationIdleHook(void)
{

}

/*************************************************************************************************/
void vApplicationMallocFailedHook(void)
{
    while (1)
    {

    }
}

/*************************************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    while (1)
    {

    }
}

/*************************************************************************************************/
TaskMngTickType_t xTaskManager_getTickCount(void)
{
    if (__ReadExceptionNumber())
    {
        return xTaskGetTickCountFromISR();
    }
    else
    {
        return xTaskGetTickCount();
    }
}

/*************************************************************************************************/
void vTaskManager_Delay(uint32_t delayms)
{
    if (delayms == portMAX_DELAY)
    {
        vTaskDelay(delayms);
    }
    else
    {
        vTaskDelay(delayms / portTICK_RATE_MS);
    }
}

/*************************************************************************************************/
TaskMngBaseType_t xTaskManager_TaskCreate(
        TaskMngTaskFunction_t TaskCode,
        const char * const pTaskName,
        const uint16_t u16StackDepth,
        void * const pvParameters,
        uint32_t p_u32Priority,
        TaskMngTaskHandle_t * const CreatedTask)
{
    if (__ReadExceptionNumber())
    {
        //Cannot create task from within an interrupt
        while (1)
            ;
    }

    return (xTaskCreate(TaskCode, pTaskName, u16StackDepth, pvParameters, p_u32Priority, CreatedTask));
}

/*************************************************************************************************/
void vTaskManager_TaskResume(TaskMngTaskHandle_t xTaskToResume)
{
    configASSERT(xTaskToResume != NULL);
    if (__ReadExceptionNumber())
    {
        xTaskResumeFromISR(xTaskToResume);
    }
    else
    {
        vTaskResume(xTaskToResume);
    }
}

/*************************************************************************************************/
void vTaskManager_TaskSuspend(TaskMngTaskHandle_t xTaskToSuspend)
{
    vTaskSuspend(xTaskToSuspend);
}

/*************************************************************************************************/
void vTaskManager_TaskDelete(TaskMngTaskHandle_t xTaskToDelete)
{
    configASSERT(xTaskToDelete != NULL);
    vTaskDelete(xTaskToDelete);
}

/*************************************************************************************************/
TaskMngStreamBufferHandle_t xTaskManager_StreamBufferCreate(size_t xBufferSizeBytes, size_t xTriggerLevelBytes)
{
    return (xStreamBufferCreate(xBufferSizeBytes, xTriggerLevelBytes));
}

/*************************************************************************************************/
void xTaskManager_StreamBufferReset(TaskMngStreamBufferHandle_t xStreamBuffer)
{
    xStreamBufferReset(xStreamBuffer);
}

/*************************************************************************************************/
void xTaskManager_StreamBufferDelete(TaskMngStreamBufferHandle_t xStreamBuffer)
{
    vStreamBufferDelete(xStreamBuffer);
}

/*************************************************************************************************/
size_t xTaskManager_StreamBufferSend(
        TaskMngStreamBufferHandle_t xStreamBuffer,
        const void *pvTxData,
        size_t xDataLengthBytes,
        TickType_t p_u32TicksToWait)
{
    size_t ret;
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT(xStreamBuffer != NULL);

    if (__ReadExceptionNumber())
    {
        ret = xStreamBufferSendFromISR(xStreamBuffer, pvTxData, xDataLengthBytes, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        TickType_t xBlockTime;
        xBlockTime = ((p_u32TicksToWait >= portMAX_DELAY ) ?
        portMAX_DELAY :
                                                             (p_u32TicksToWait / portTICK_RATE_MS));
        ret = xStreamBufferSend(xStreamBuffer, pvTxData, xDataLengthBytes, xBlockTime);
    }

    return ret;

}

/*************************************************************************************************/
size_t xTaskManager_StreamBufferReceive(
        TaskMngStreamBufferHandle_t xStreamBuffer,
        void *pvTxData,
        size_t xDataLengthBytes,
        TickType_t p_u32TicksToWait)
{
    size_t ret;
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT(xStreamBuffer != NULL);

    if (__ReadExceptionNumber())
    {
        ret = xStreamBufferReceiveFromISR(xStreamBuffer, pvTxData, xDataLengthBytes, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        TickType_t xBlockTime;
        xBlockTime = ((p_u32TicksToWait >= portMAX_DELAY ) ?
        portMAX_DELAY :
                                                             (p_u32TicksToWait / portTICK_RATE_MS));
        ret = xStreamBufferReceive(xStreamBuffer, pvTxData, xDataLengthBytes, xBlockTime);
    }

    return ret;
}

/*************************************************************************************************/
TaskMngBaseType_t xTaskManager_TaskNotifyGive(TaskMngTaskHandle_t xTaskToNotify)
{
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskMngBaseType_t ret;

    configASSERT(xTaskToNotify != NULL);

    if (__ReadExceptionNumber())
    {
        vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
        ret = pdPASS;
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        ret = xTaskNotifyGive(xTaskToNotify);
    }
    return ret;
}

/*************************************************************************************************/
uint32_t ui32TaskManager_TaskNotifyTake(TaskMngBaseType_t xClearCountOnExit, TickType_t xTicksToWait)
{
    TickType_t xBlockTime;
    xBlockTime = ((xTicksToWait >= portMAX_DELAY ) ?
    portMAX_DELAY :
                                                     (xTicksToWait / portTICK_RATE_MS));
    return ulTaskNotifyTake(xClearCountOnExit, xBlockTime);
}

/*************************************************************************************************/
TaskMngQueueHandle_t xTaskManager_QueueCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize)
{
    return (xQueueCreate(uxQueueLength, uxItemSize));
}

/*************************************************************************************************/
TaskMngBaseType_t xTaskManager_QueueSend(
        TaskMngQueueHandle_t xQueue,
        const void * const pvItemToQueue,
        TickType_t xTicksToWait)
{
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskMngBaseType_t ret;

    configASSERT(xQueue != NULL);

    if (__ReadExceptionNumber())
    {
        ret = xQueueSendFromISR(xQueue, pvItemToQueue, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        TickType_t xBlockTime;
        xBlockTime = ((xTicksToWait >= portMAX_DELAY ) ?
        portMAX_DELAY :
                                                         (xTicksToWait / portTICK_RATE_MS));
        ret = xQueueSend(xQueue, pvItemToQueue, xBlockTime);
    }
    return ret;
}

/*************************************************************************************************/
TaskMngBaseType_t xTaskManager_QueueReceive(TaskMngQueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait)
{
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskMngBaseType_t ret;

    configASSERT(xQueue != NULL);

    if (__ReadExceptionNumber())
    {
        ret = xQueueReceiveFromISR(xQueue, pvBuffer, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        TickType_t xBlockTime;
        xBlockTime = ((xTicksToWait >= portMAX_DELAY ) ?
        portMAX_DELAY :
                                                         (xTicksToWait / portTICK_RATE_MS));
        ret = xQueueReceive(xQueue, pvBuffer, xBlockTime);
    }
    return ret;
}

/*************************************************************************************************/
TaskMngSemaphoreHandle_t xTaskManager_SemaphoreCreate(void)
{
    return (xSemaphoreCreateBinary());
}

/*************************************************************************************************/
void vTaskManager_SemaphoreDelete(TaskMngSemaphoreHandle_t xSemaphore)
{
    configASSERT(xSemaphore != NULL);
    vSemaphoreDelete(xSemaphore);
}

/*************************************************************************************************/
TaskMngSemaphoreHandle_t xTaskManager_MutexCreate(void)
{
    return (xSemaphoreCreateMutex());
}

/*************************************************************************************************/
TaskMngBaseType_t xTaskManager_SemaphoreGive(TaskMngSemaphoreHandle_t xSemaphore)
{
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskMngBaseType_t ret;

    configASSERT(xSemaphore != NULL);

    if (__ReadExceptionNumber())
    {
        ret = xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        ret = xSemaphoreGive(xSemaphore);
    }
    return ret;
}

/*************************************************************************************************/
TaskMngBaseType_t xTaskManager_SemaphoreTake(TaskMngSemaphoreHandle_t xSemaphore, TickType_t xTicksToWait)
{
    TaskMngBaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TaskMngBaseType_t ret;

    configASSERT(xSemaphore != NULL);

    if (__ReadExceptionNumber())
    {
        ret = xSemaphoreTakeFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        TickType_t xBlockTime;
        xBlockTime = ((xTicksToWait >= portMAX_DELAY ) ?
        portMAX_DELAY :
                                                         (xTicksToWait / portTICK_RATE_MS));
        ret = xSemaphoreTake(xSemaphore, xBlockTime);
    }
    return ret;
}
/*************************************************************************************************/
