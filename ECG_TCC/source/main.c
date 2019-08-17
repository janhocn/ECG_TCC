/**
 ******************************************************************************
 * @file    main.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 July 2017
 * @brief   Main Module
 ******************************************************************************
 */

/* Standard includes. */
#include <main.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
/* Freescale includes. */
#include "board.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dac.h"
#include "fsl_sysmpu.h"
/* User includes. */
#include "main.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "task_manager.h"
/*************************************************************************************************/
int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    SYSMPU_Enable(SYSMPU, false);
    SystemCoreClockUpdate();

    /** Initializes OS and all modules */
    vTaskManagerInit();

    /** Should not reach here. */
    for (;;)
        ;

    NVIC_SystemReset();
    return 0;
}
/*************************************************************************************************/
