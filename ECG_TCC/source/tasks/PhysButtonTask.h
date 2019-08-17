/**
 ******************************************************************************
 * @file    PhysButtonTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    12 September 2017
 * @brief   Module to manage home button
 ******************************************************************************
 */

#ifndef TASKS_PHYSBUTTONTASK_H_
#define TASKS_PHYSBUTTONTASK_H_

#include "arm_math.h"

#if USE_PROTOTYPE

/*************************************************************************************************/
/**
 * @brief Initialzie the module and tasks
 * @return void.
 */
void vPhysButtonTaskInit(void);

/*************************************************************************************************/
/**
 * @brief Home Button IRQ Hander
 * @brief status true if the interrupt is for this handler (this pin)
 * @return void.
 */
void vButtonTask_IRQ_ButtonHome(uint32_t status);

#endif
#endif /* TASKS_PHYSBUTTONTASK_H_ */
