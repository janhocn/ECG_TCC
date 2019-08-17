/**
 ******************************************************************************
 * @file    Display.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 July 2017
 * @brief   Module to Manage the display states
 ******************************************************************************
 */
#include "LCDConf.h"
#include <stdio.h>
#include <stdlib.h>
#include "FRAMEWIN.h"
#include "Home.h"
#include "GraphECG.h"
#include "GraphSPO2.h"
#include "SettingsScreen.h"
#include "Display.h"
#include "BUTTON.h"
#include "MESSAGEBOX.h"
#include "StatusBar.h"
#include "ECGBackground1.h"
#include "Messages.h"
#include "ecgAcqTask.h"
#include "spo2AcqTask.h"
#include "FreeRTOS.h"
#include "task.h"

/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
typedef struct _DispalyPack
{
    WM_HWIN hWinCurrentWindow; /** @< Holds the handler of the current visible window */
    WM_HWIN hAboutBox; /** @< handler of the message box */
    stateDisplayTabs stUpdateDisplayTabs; /** @< Holds the state of the window*/
} sDispalyPack_t;

static sDispalyPack_t sDispalyPack;
/*************************************************************************************************/
/**
 * @brief Exhibit the software version info
 * @return void.
 */
static void vDisplay_ShowVersion(void);

/*************************************************************************************************/
/**
 * @brief Exhibit the debug info
 * @return void.
 */
static void vDisplay_ShowDebugInfo(void);

/*************************************************************************************************/
/**
 * @brief Exhibit the help info
 * @return void.
 */
static void vDisplay_ShowHelpInfo(void);
/*************************************************************************************************/
/**
 * @brief Emwin Callback for the Message Box Button
 * @param pMsg Message from the Emwin core
 * @return void.
 */
static void svDisplay_CallbackMessageBoxButton(WM_MESSAGE *pMsg);

/*************************************************************************************************/
/**
 * @brief Creates and customize a message box
 * @param sMessage Message to be informed
 * @param sCaption Caption title in the frame
 * @param Flags Emwin Flags for the message box
 * @param pFont Font for the text
 * @return void.
 */
static void _CreateMessageBox(const char * sMessage, const char * sCaption, int Flags, const GUI_FONT * pFont);

/*************************************************************************************************/
/**
 * @brief Creates and customize a message box
 * @param sMessage Message to be informed
 * @param sCaption Caption title in the frame
 * @param Flags Emwin Flags for the message box
 * @param pFont Font for the text
 * @return void.
 */
static void _CreateMessageBoxSmall(const char * sMessage, const char * sCaption, int Flags, const GUI_FONT * pFont);


/*************************************************************************************************/
/**
 * @brief Sets the actual window state
 * @param st Window State (see stateDisplayTabs).
 * @return void
 */
static void __vDisplay_SetState(stateDisplayTabs st);

/*************************************************************************************************/
static void _CreateMessageBoxSmall(const char * sMessage, const char * sCaption, int Flags, const GUI_FONT * pFont)
{
    WM_HWIN hItem;
    GUI_RECT Rect;

    sDispalyPack.hAboutBox = MESSAGEBOX_Create(sMessage, sCaption, Flags);
    //
    // Change font of message box window
    //
    FRAMEWIN_SetFont(sDispalyPack.hAboutBox, pFont);
    //
    // Adjust size
    //
    WM_GetWindowRectEx(sDispalyPack.hAboutBox, &Rect);
    WM_SetWindowPos(sDispalyPack.hAboutBox, Rect.x0 - 15, Rect.y0 - 15, Rect.x1 - Rect.x0 + 1 + 80,
            Rect.y1 - Rect.y0 + 1 + 30);
    //
    // Change font of button widget
    //
    hItem = WM_GetDialogItem(sDispalyPack.hAboutBox, GUI_ID_OK);
    BUTTON_SetFont(hItem, pFont);
    WM_SetCallback(hItem, svDisplay_CallbackMessageBoxButton);
    //
    // Adjust size of button widget
    //
    WM_GetWindowRectEx(hItem, &Rect);
    WM_SetWindowPos(hItem, Rect.x0, Rect.y0 + 16, Rect.x1 - Rect.x0 + 1 + 50, Rect.y1 - Rect.y0 + 1 + 10);
    //
    // Change font of text widget
    //
    hItem = WM_GetDialogItem(sDispalyPack.hAboutBox, GUI_ID_TEXT0);
    TEXT_SetFont(hItem, pFont);
    //
    // Adjust size text widget
    //
    WM_GetWindowRectEx(hItem, &Rect);
    WM_SetWindowPos(hItem, Rect.x0, Rect.y0, Rect.x1 - Rect.x0 + 1 + 60, Rect.y1 - Rect.y0 + 1 + 18);
}

/*************************************************************************************************/
static void _CreateMessageBox(const char * sMessage, const char * sCaption, int Flags, const GUI_FONT * pFont)
{
    WM_HWIN hItem;
    GUI_RECT Rect;

    sDispalyPack.hAboutBox = MESSAGEBOX_Create(sMessage, sCaption, Flags);
    //
    // Change font of message box window
    //
    FRAMEWIN_SetFont(sDispalyPack.hAboutBox, pFont);
    //
    // Adjust size
    //
    WM_GetWindowRectEx(sDispalyPack.hAboutBox, &Rect);
    WM_SetWindowPos(sDispalyPack.hAboutBox, Rect.x0 - 50, Rect.y0 - 35, Rect.x1 - Rect.x0 + 1 + 150,
            Rect.y1 - Rect.y0 + 1 + 115);
    //
    // Change font of button widget
    //
    hItem = WM_GetDialogItem(sDispalyPack.hAboutBox, GUI_ID_OK);
    BUTTON_SetFont(hItem, pFont);
    WM_SetCallback(hItem, svDisplay_CallbackMessageBoxButton);
    //
    // Adjust size of button widget
    //
    WM_GetWindowRectEx(hItem, &Rect);
    WM_SetWindowPos(hItem, Rect.x0, Rect.y0 + 101, Rect.x1 - Rect.x0 + 1 + 50, Rect.y1 - Rect.y0 + 1 + 10);
    //
    // Change font of text widget
    //
    hItem = WM_GetDialogItem(sDispalyPack.hAboutBox, GUI_ID_TEXT0);
    TEXT_SetFont(hItem, pFont);
    //
    // Adjust size text widget
    //
    WM_GetWindowRectEx(hItem, &Rect);
    WM_SetWindowPos(hItem, Rect.x0, Rect.y0, Rect.x1 - Rect.x0 + 1 + 145, Rect.y1 - Rect.y0 + 1 + 103);
}

/*************************************************************************************************/
static void svDisplay_CallbackMessageBoxButton(WM_MESSAGE *pMsg)
{
    switch (pMsg->MsgId)
    {
        case WM_PID_STATE_CHANGED:

            switch (sDispalyPack.stUpdateDisplayTabs)
            {
                case _WINDOW_HOME:
                    if (GUI_PID_IsPressed())
                    {
                        vDisplay_SetWindow(_WINDOW_HOME);
                    }
                    else
                    {
                        vDisplay_SetWindow(_WINDOW_HOME);
                    }
                    break;

                default:
                    break;
            }

        default:
            BUTTON_Callback(pMsg);
            break;
    }
}

/*************************************************************************************************/
void vDisplay_GUI_Init(void)
{
    memset(&sDispalyPack, 0, sizeof(sDispalyPack_t));

    GUI_Init();

#if GUI_SUPPORT_MEMDEV //Support for memory device
    WM_SetCreateFlags(WM_CF_MEMDEV);
#endif

    /*Enables the UTF8 encoding for portuguese language support*/
    GUI_UC_SetEncodeUTF8();

    WM_MOTION_Enable(1);

    GUI_SetBkColor(GUI_BLACK);

    WM_Exec();
}

/*************************************************************************************************/
void vDisplay_WindowsInit(void)
{
    sDispalyPack.stUpdateDisplayTabs = _WINDOW_HOME;
    /* Create multipage widget */
    vHOME_AddHomeScreen();
    vGraphECG_AddGraphECGScreen();
    vGraphSPO2_AddGraphSPO2Screen();
    vSettings_AddSettingsScreen();
    vStatusBar_AddStatusBar(WM_HBKWIN);
    sDispalyPack.hWinCurrentWindow = hHOME_Win();

    WM_Exec();
}

/*************************************************************************************************/
void vDisplay_RefreshDisplay(void)
{
    GUI_Exec();
}

/*************************************************************************************************/
void vDisplay_SetWindow(stateDisplayTabs st)
{
    __vDisplay_SetState(st);
    switch (st)
    {
        default:
        case _WINDOW_HOME:
            vEcgAcqTaskSuspend();
            vSPO2AcqTaskSuspend();
            WM_HideWindow(sDispalyPack.hWinCurrentWindow);
            WM_ShowWindow(hHOME_Win());
            sDispalyPack.hWinCurrentWindow = hHOME_Win();
            break;
        case _WINDOW_ECGGraph:
            vEcgAcqTaskResume();
            vSPO2AcqTaskSuspend();
            WM_HideWindow(sDispalyPack.hWinCurrentWindow);
            WM_ShowWindow(hGraphECG_getWindow());
            sDispalyPack.hWinCurrentWindow = hGraphECG_getWindow();
            break;
        case _WINDOW_SPO2Graph:
            vEcgAcqTaskSuspend();
            vSPO2AcqTaskResume();
            WM_HideWindow(sDispalyPack.hWinCurrentWindow);
            WM_ShowWindow(hGraphSPO2_getWindow());
            sDispalyPack.hWinCurrentWindow = hGraphSPO2_getWindow();
            break;
        case _WINDOW_SETTINGS:
            vSPO2AcqTaskSuspend();
            vEcgAcqTaskSuspend();
            WM_HideWindow(sDispalyPack.hWinCurrentWindow);
            vSettings_HideSettings();
            WM_ShowWindow(hSettings_Win());
            sDispalyPack.hWinCurrentWindow = hSettings_Win();
            break;

        case _WINDOW_ABOUT:
            vEcgAcqTaskSuspend();
            vSPO2AcqTaskSuspend();
            vDisplay_ShowVersion();
            break;

        case _WINDOW_HELP:
            vEcgAcqTaskSuspend();
            vSPO2AcqTaskSuspend();
            vDisplay_ShowHelpInfo();
            break;

        case _WINDOW_DEBUG:
            vEcgAcqTaskSuspend();
            vSPO2AcqTaskSuspend();
            vDisplay_ShowDebugInfo();
            break;
    }

    switch (st)
    {
        case _WINDOW_HOME:
            vHOME_HideButton();
            break;

        case _WINDOW_ECGGraph:
        case _WINDOW_SPO2Graph:
        case _WINDOW_SETTINGS:
            vHOME_ShowButton();
            break;

        default:
            break;

    }

}

/*************************************************************************************************/
static void vDisplay_ShowVersion(void)
{
    _CreateMessageBox(xMessage_getMessageDisplay()->version, xMessage_getMessageDisplay()->projectName,
    GUI_MESSAGEBOX_CF_MODAL, &GUI_Font16B_1);
}

/*************************************************************************************************/
static void vDisplay_ShowHelpInfo(void)
{
    _CreateMessageBox(xMessage_getMessageHelp()->text, xMessage_getMessageHelp()->title,
    GUI_MESSAGEBOX_CF_MODAL, &GUI_Font16B_1);
}

/*************************************************************************************************/
static void vDisplay_ShowDebugInfo(void)
{
    char ucInfo[100];
    uint32_t freeBytes;
    uint32_t usedBytes;
    uint32_t numberTasks;

    freeBytes = GUI_ALLOC_GetNumFreeBytes();
    usedBytes = GUI_ALLOC_GetNumUsedBytes();
    numberTasks = uxTaskGetNumberOfTasks();
    sprintf(ucInfo, xMessage_getMessageDisplay()->debugInfo, freeBytes, usedBytes, numberTasks);
    _CreateMessageBoxSmall(ucInfo, xMessage_getMessageDisplay()->debugTitle, GUI_MESSAGEBOX_CF_MODAL, &GUI_Font16B_1);
}

/*************************************************************************************************/
void vDisplay_ShowMessageBox(char * msg, char * title)
{
    _CreateMessageBoxSmall(msg, title, GUI_MESSAGEBOX_CF_MODAL, &GUI_Font16B_1);
}

/*************************************************************************************************/
static void __vDisplay_SetState(stateDisplayTabs st)
{
    if (st == _WINDOW_HOME)
    {
        switch (sDispalyPack.stUpdateDisplayTabs)
        {
            case _WINDOW_ECGGraph:
            case _WINDOW_SPO2Graph:
                vGraphECG_ClearMainGraph();
                vGraphSPO2_ClearMainGraph();
                break;

            default:
                break;
        }
    }
    sDispalyPack.stUpdateDisplayTabs = st;
}

/*************************************************************************************************/
void vDisplay_DrawBackground(void)
{
    GUI_DrawBitmap(&bmECGBackground1, 0, 0);
}

/*************************************************************************************************/
stateDisplayTabs vDisplay_GetState(void)
{
    return sDispalyPack.stUpdateDisplayTabs;
}
/*************************************************************************************************/
