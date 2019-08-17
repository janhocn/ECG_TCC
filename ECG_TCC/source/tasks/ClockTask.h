/**
 ******************************************************************************
 * @file    ClockTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 August 2017
 * @brief   Module manage the clock task
 ******************************************************************************
 */

#ifndef TASKS_CLOCKTASK_H_
#define TASKS_CLOCKTASK_H_

#include "task_manager.h"

/*************************************************************************************************/
/**
 * @brief Initialzie the module and tasks
 * @return void.
 */
void vClockTaskInit(void);

/*************************************************************************************************/
/**
 * @brief Get the task handler
 * @return FreeRTOS Handler to the task.
 */
TaskMngTaskHandle_t xClockTaskGetHandler(void);
#endif /* TASKS_CLOCKTASK_H_ */
