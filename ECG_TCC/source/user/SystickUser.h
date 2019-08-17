/**
 ******************************************************************************
 * @file    SystickUser.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 November 2017
 * @brief   Module to use systick to perform time based functions
 ******************************************************************************
 */
#ifndef USER_SYSTICKUSER_H_
#define USER_SYSTICKUSER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*************************************************************************************************/
/**
 * @brief Initializes the module
 * @return void
 */
void vSystickUser_Init(void);

/*************************************************************************************************/
/**
 * @brief Increment the tick counter (it should be called from within a timer interrupt)
 * @return void
 */
void vSystickUser_IncTick(void);

/*************************************************************************************************/
/**
 * @brief Delay function based on the tick
 * @param Delay Delay units based on the period of the tick increment
 * @return void
 */
void vSystickUser_Delay(uint32_t Delay);

/*************************************************************************************************/
/**
 * @brief Check if the waitMs time has peassed since startTick
 * @param startTick Ticks get from from u32SystickUser_getTick
 * @param waitMs Time to check if it is greater than startTick
 * @return true if waitMs is greater than startTick
 */
bool bSystickUser_Wait(uint32_t startTick, uint32_t waitMs);

/*************************************************************************************************/
/**
 * @brief get The tick count
 * @return tick count
 */
uint32_t u32SystickUser_getTick(void);

#endif /* USER_SYSTICKUSER_H_ */
