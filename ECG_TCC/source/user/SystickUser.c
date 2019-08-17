/**
 ******************************************************************************
 * @file    SystickUser.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 November 2017
 * @brief   Module to use systick to perform time based functions
 ******************************************************************************
 */

#include "SystickUser.h"

/*************************************************************************************************/
/**
 * @var Tick Counter
 */
static volatile uint32_t su32Tick = 0;

/*************************************************************************************************/
void vSystickUser_Init(void)
{
    su32Tick = 0;
}

/*************************************************************************************************/
uint32_t u32SystickUser_getTick(void)
{
    return su32Tick;
}

/*************************************************************************************************/
void vSystickUser_IncTick(void)
{
    su32Tick++;
}

/*************************************************************************************************/
void vSystickUser_Delay(uint32_t Delay)
{
    uint32_t tickstart = 0;
    tickstart = u32SystickUser_getTick();
    while ((u32SystickUser_getTick() - tickstart) < Delay);
}

/*************************************************************************************************/
bool bSystickUser_Wait(uint32_t startTick, uint32_t waitMs)
{
    if ((u32SystickUser_getTick() - startTick) >= waitMs)
    {
        return true;
    }
    return false;
}
/*************************************************************************************************/
