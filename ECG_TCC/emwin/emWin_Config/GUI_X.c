/*********************************************************************
 *                SEGGER Microcontroller GmbH & Co. KG                *
 *        Solutions for real time microcontroller applications        *
 **********************************************************************
 *                                                                    *
 *        (c) 1996 - 2013  SEGGER Microcontroller GmbH & Co. KG       *
 *                                                                    *
 *        Internet: www.segger.com    Support:  support@segger.com    *
 *                                                                    *
 **********************************************************************

 ** emWin V5.22 - Graphical user interface for embedded applications **
 All  Intellectual Property rights  in the Software belongs to  SEGGER.
 emWin is protected by  international copyright laws.  Knowledge of the
 source code may not be used to write a similar product.  This file may
 only be used in accordance with the following terms:

 The software has been licensed to  NXP Semiconductors USA, Inc.  whose
 registered  office  is  situated  at 411 E. Plumeria Drive, San  Jose,
 CA 95134, USA  solely for  the  purposes  of  creating  libraries  for
 NXPs M0, M3/M4 and  ARM7/9 processor-based  devices,  sublicensed  and
 distributed under the terms and conditions of the NXP End User License
 Agreement.
 Full source code is available at: www.segger.com

 We appreciate your understanding and fairness.
 ----------------------------------------------------------------------
 File        : GUI_X.C
 Purpose     : Config / System dependent externals for GUI
 ---------------------------END-OF-HEADER------------------------------
 */

#include "TouchScreen.h"
#include "GUI.h"
#include "GUIConf.h"
/* FreeRTOS include files */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "SystickUser.h"
/*********************************************************************
 *
 *       Global data
 */
static xSemaphoreHandle xQueueMutex;
static xSemaphoreHandle xSemaTxDone;

#ifndef GUI_MEMORY_ADDR
static uint32_t s_gui_memory[(GUI_NUMBYTES + 3) / 4]; /* needs to be word aligned */
#define GUI_MEMORY_ADDR ((uint32_t)s_gui_memory)
#endif

void GUI_X_Config(void)
{
    /* Assign work memory area to emWin */
    GUI_ALLOC_AssignMemory((void *) GUI_MEMORY_ADDR, GUI_NUMBYTES);

    GUITASK_SetMaxTask(8);
    /* Select default font */
    GUI_SetDefaultFont(GUI_FONT_6X8);
}

/*********************************************************************
 *
 *      Timing:
 *                 GUI_X_GetTime()
 *                 GUI_X_Delay(int)

 Some timing dependent routines require a GetTime
 and delay function. Default time unit (tick), normally is
 1 ms.
 */

GUI_TIMER_TIME GUI_X_GetTime(void)
{

    return ((GUI_TIMER_TIME) xTaskGetTickCount() * portTICK_PERIOD_MS);
}

void GUI_X_Delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

/*********************************************************************
 *
 *       GUI_X_Init()
 *
 * Note:
 *     GUI_X_Init() is called from GUI_Init is a possibility to init
 *     some hardware which needs to be up and running before the GUI.
 *     If not required, leave this routine blank.
 */

void GUI_X_Init(void)
{
}

/*********************************************************************
 *
 *       GUI_X_ExecIdle
 *
 * Note:
 *  Called if WM is in idle state
 */

void GUI_X_ExecIdle(void)
{
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

/*********************************************************************
 *
 *      Multitasking:
 *
 *                 GUI_X_InitOS()
 *                 GUI_X_GetTaskId()
 *                 GUI_X_Lock()
 *                 GUI_X_Unlock()
 *
 * Note:
 *   The following routines are required only if emWin is used in a
 *   true multi task environment, which means you have more than one
 *   thread using the emWin API.
 *   In this case the
 *                       #define GUI_OS 1
 *  needs to be in GUIConf.h
 */

void GUI_X_InitOS(void)
{
    /* Create Mutex lock */
    xQueueMutex = xSemaphoreCreateMutex();
    configASSERT(xQueueMutex != NULL);

    /* Queue Semaphore */
    xSemaTxDone = xSemaphoreCreateBinary();
    configASSERT(xSemaTxDone != NULL);
}

void GUI_X_Unlock(void)
{
    xSemaphoreGive(xQueueMutex);
}

void GUI_X_Lock(void)
{
    if (xQueueMutex == NULL)
    {
        GUI_X_InitOS();
    }
    xSemaphoreTake(xQueueMutex, portMAX_DELAY);
}

U32 GUI_X_GetTaskId(void)
{
    return ((U32) xTaskGetCurrentTaskHandle());
}

/*********************************************************************
 *
 *      Event driving (optional with multitasking)
 *
 *                 GUI_X_WaitEvent()
 *                 GUI_X_WaitEventTimed()
 *                 GUI_X_SignalEvent()
 */

void GUI_X_WaitEvent(void)
{
    while (xSemaphoreTake(xSemaTxDone, portMAX_DELAY) != pdTRUE)
        ;
}

void GUI_X_SignalEvent(void)
{
    xSemaphoreGive(xSemaTxDone);
}

/*********************************************************************
 *
 *      Logging: OS dependent

 Note:
 Logging is used in higher debug levels only. The typical target
 build does not use logging and does therefor not require any of
 the logging routines below. For a release build without logging
 the routines below may be eliminated to save some space.
 (If the linker is not function aware and eliminates unreferenced
 functions automatically)

 */

void GUI_X_Log(const char *s)
{
    GUI_USE_PARA(s);
}
void GUI_X_Warn(const char *s)
{
    GUI_USE_PARA(s);
}
void GUI_X_ErrorOut(const char *s)
{
    GUI_USE_PARA(s);
}

void GUI_TOUCH_X_ActivateY(void)
{
#if USE_ANALOGTOUCH
    vTouchScreen_ActivateY();
#endif
}

void GUI_TOUCH_X_ActivateX(void)
{
#if USE_ANALOGTOUCH
    vTouchScreen_ActivateX();
#endif
}

int GUI_TOUCH_X_MeasureX(void)
{
#if USE_ANALOGTOUCH
    int xPos = ui16TouchScreen_MeasureY();

    return xPos;
#else
    return 0;
#endif
}

int GUI_TOUCH_X_MeasureY(void)
{
#if USE_ANALOGTOUCH
    int yPos = ui16TouchScreen_MeasureY();

    return yPos;
#else
    return 0;
#endif
}

/*************************** End of file ****************************/
