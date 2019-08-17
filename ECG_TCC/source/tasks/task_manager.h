/**
 ******************************************************************************
 * @file    task_manager.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 August 2017
 * @brief   Module that provides functions to work with the RTOS tasks
 ******************************************************************************
 */

#ifndef TASKS_TASK_MANAGER_H_
#define TASKS_TASK_MANAGER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "stream_buffer.h"
#include <stdbool.h>
#include <stdint.h>
/**
 * @addtogroup  FreeRTOS types
 * @{
 */
typedef TaskHandle_t TaskMngTaskHandle_t;
typedef SemaphoreHandle_t TaskMngSemaphoreHandle_t;
typedef StreamBufferHandle_t TaskMngStreamBufferHandle_t;
typedef QueueHandle_t TaskMngQueueHandle_t;
typedef TaskFunction_t TaskMngTaskFunction_t;
typedef BaseType_t TaskMngBaseType_t;
typedef TickType_t TaskMngTickType_t;
/**@}*/

void vTaskManagerInit(void);
bool bTaskManager_isSystemInitOK(void);
TaskMngBaseType_t xTaskManager_TaskCreate(TaskMngTaskFunction_t TaskCode, const char * const pTaskName, const uint16_t u16StackDepth, void * const pvParameters, uint32_t p_u32Priority, TaskMngTaskHandle_t * const CreatedTask);
void vTaskManager_TaskDelete(TaskMngTaskHandle_t xTaskToDelete);
TaskMngTickType_t xTaskManager_getTickCount(void);
void vTaskManager_Delay(uint32_t delayms);
void vTaskManager_TaskResume(TaskMngTaskHandle_t xTaskToResume);
void vTaskManager_TaskSuspend(TaskMngTaskHandle_t xTaskToSuspend);
TaskMngSemaphoreHandle_t xTaskManager_MutexCreate(void);
TaskMngSemaphoreHandle_t xTaskManager_SemaphoreCreate(void);
TaskMngBaseType_t xTaskManager_SemaphoreGive(TaskMngSemaphoreHandle_t xSemaphore);
TaskMngBaseType_t xTaskManager_SemaphoreTake(TaskMngSemaphoreHandle_t xSemaphore, TickType_t xTicksToWait);
void vTaskManager_SemaphoreDelete(TaskMngSemaphoreHandle_t xSemaphore);
TaskMngQueueHandle_t xTaskManager_QueueCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize);
TaskMngBaseType_t xTaskManager_QueueSend(TaskMngQueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait);
TaskMngBaseType_t xTaskManager_QueueReceive(TaskMngQueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait);
TaskMngStreamBufferHandle_t xTaskManager_StreamBufferCreate(size_t xBufferSizeBytes, size_t xTriggerLevelBytes);
void xTaskManager_StreamBufferDelete(TaskMngStreamBufferHandle_t xStreamBuffer);
void xTaskManager_StreamBufferReset(TaskMngStreamBufferHandle_t xStreamBuffer);
size_t xTaskManager_StreamBufferSend(TaskMngStreamBufferHandle_t xStreamBuffer, const void *pvTxData, size_t xDataLengthBytes, TickType_t p_u32TicksToWait);
size_t xTaskManager_StreamBufferReceive(TaskMngStreamBufferHandle_t xStreamBuffer, void *pvTxData, size_t xDataLengthBytes, TickType_t p_u32TicksToWait);
TaskMngBaseType_t xTaskManager_TaskNotifyGive(TaskMngTaskHandle_t xTaskToNotify);
uint32_t ui32TaskManager_TaskNotifyTake(TaskMngBaseType_t xClearCountOnExit, TickType_t xTicksToWait);

#endif /* TASKS_TASK_MANAGER_H_ */
