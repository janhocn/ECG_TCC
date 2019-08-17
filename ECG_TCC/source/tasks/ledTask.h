/**
 ******************************************************************************
 * @file    ledTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 August 2017
 * @brief   Module manage the heart beat LED
 ******************************************************************************
 */
#ifndef TASKS_LEDTASK_H_
#define TASKS_LEDTASK_H_

#if USE_PROTOTYPE
/*************************************************************************************************/
/**
 * @def LED blinking period in miliseconds
 */
#define pinTogglePeriod 400U

/*************************************************************************************************/
/**
 * @brief Initialzie the module and task
 * @return void.
 */
void vLedTaskInit(void);
#endif

#endif /* TASKS_LEDTASK_H_ */
