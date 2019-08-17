/**
 ******************************************************************************
 * @file    touchUpdateTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 September 2017
 * @brief   Module to manage the touch update
 ******************************************************************************
 */
#ifndef TASKS_TOUCHUPDATETASK_H_
#define TASKS_TOUCHUPDATETASK_H_


/*************************************************************************************************/
/**
 * @def Touch update period in miliseconds
 */
#define refreshTouchPeriod 20

/*************************************************************************************************/
/**
 * @brief Initialzie the module and tasks
 * @return void.
 */
void vTouchTaskInit(void);

#endif /* TASKS_TOUCHUPDATETASK_H_ */
