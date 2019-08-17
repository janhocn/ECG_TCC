/**
 ******************************************************************************
 * @file    displayRefreshTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 August 2017
 * @brief   Module manage the display refresh task
 ******************************************************************************
 */

#ifndef TASKS_DISPLAYREFRESHTASK_H_
#define TASKS_DISPLAYREFRESHTASK_H_

#include "task_manager.h"
#include "task_manager.h"
/*************************************************************************************************/

/*************************************************************************************************/
/**
 * @def Display update period in miliseconds
 */
#define updateDisplayPeriodms 10

/*************************************************************************************************/
/**
 * @brief Initialzie the module and task
 * @return void.
 */
void vUpdateDisplayTaskInit(void);

/*************************************************************************************************/
/**
 * @brief Resume the task
 * @return void.
 */
void vUpdateDisplayTaskResume(void);

/*************************************************************************************************/
/**
 * @brief Suspend the task
 * @return void.
 */
void vUpdateDisplayTaskSuspend(void);

/*************************************************************************************************/
/**
 * @brief Get the task handler
 * @return FreeRTOS Handler to the task.
 */
TaskMngTaskHandle_t xUpdateDisplayTaskGetHandler(void);
/*************************************************************************************************/
#endif /* TASKS_DISPLAYREFRESHTASK_H_ */
