/**
 ******************************************************************************
 * @file    Home.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    3 July 2017
 * @brief   Module to manage Home Window
 ******************************************************************************
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "Home.h"
#include "Display.h"
#include "BUTTON.h"
#include "IMAGE.h"
#include "ECGIcon.h"
#include "SpO2Icon.h"
#include "AboutIcon.h"
#include "SettingsIcon.h"
#include "HomeIcon.h"
#include "HelpIcon.h"
#include "debugIcon.h"
#include "Messages.h"
#include "GUI_IDS.h"
#include "MackenzieIcon.h"
/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
typedef struct _sHomePack
{
    WM_HWIN hWin; /** @< Handler of the Home window */
    WM_HWIN hButtonHome; /** @< Handler of the Home Button*/
    IMAGE_Handle hMackenzieIcon; /** @< Handler of the Mackenzie icon*/
} sHomePack_t;

static sHomePack_t sHomePack;
/*************************************************************************************************/
/**
 * @brief emWin callback for the Home window
 * @param pMsg emWin message.
 * @return void.
 */
static void _vHome_CallBackHomeWindow(WM_MESSAGE * pMsg);

/*************************************************************************************************/
/**
 * @brief emWin callback for the Home Button
 * @param pMsg emWin message.
 * @return void.
 */
static void _vHome_CallBackButtonHome(WM_MESSAGE * pMsg);

/*************************************************************************************************/
static void _vHome_CallBackHomeWindow(WM_MESSAGE * pMsg)
{
    int NCode, Id, Sel;
    switch (pMsg->MsgId)
    {
        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc); /* Id of widget */
            NCode = pMsg->Data.v; /* Notification code */
            switch (Id)
            {
                case HOME_ICONVIEW_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_SEL_CHANGED:
                            Sel = ICONVIEW_GetSel(pMsg->hWinSrc);
                            ICONVIEW_SetSel(pMsg->hWinSrc, -1);

                            if (vDisplay_GetState() != Sel)
                            {
                                vDisplay_SetWindow(Sel);
                            }
                            break;
                    }
                    break;

                case HOME_BUTTON_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_RELEASED:
                            vDisplay_SetWindow(_WINDOW_HOME);
                            break;
                    }
                    break;
            }
            break;

        case WM_PAINT:
            /*
             * Draw background
             */
            GUI_Clear();
            vDisplay_DrawBackground();
            break;
    }
}

/*************************************************************************************************/
static void _vHome_CallBackButtonHome(WM_MESSAGE * pMsg)
{
    GUI_RECT Rect;

    switch (pMsg->MsgId)
    {
        case WM_PAINT:

            BUTTON_Callback(pMsg);
            WM_GetClientRect(&Rect);
            GUI_DrawBitmap(&bmHomeIcon, (Rect.x1 - (&bmHomeIcon)->XSize) / 2, (Rect.y1 - (&bmHomeIcon)->YSize) / 2);
            break;

        default:
            BUTTON_Callback(pMsg);
            break;
    }
}

/*************************************************************************************************/
WM_HWIN hHOME_Win(void)
{
    return sHomePack.hWin;
}

/*************************************************************************************************/
WM_HWIN hHOME_ButtonWin(void)
{
    return sHomePack.hButtonHome;
}

/*************************************************************************************************/
void vHOME_ShowButton(void)
{
    WM_BringToTop(sHomePack.hButtonHome);
    WM_ShowWindow(sHomePack.hButtonHome);
}

/*************************************************************************************************/
void vHOME_HideButton(void)
{
    WM_HideWindow(sHomePack.hButtonHome);
}

/*************************************************************************************************/
void vHOME_AddHomeScreen(void)
{
    WM_EnableMemdev(WM_HBKWIN);
    WM_SetCallback(WM_HBKWIN, _vHome_CallBackHomeWindow);
    /*
     * Create iconview widget
     */

    memset(&sHomePack, 0, sizeof(sHomePack_t));

    sHomePack.hWin = ICONVIEW_CreateEx(10, 30, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN,
    WM_CF_SHOW | WM_CF_HASTRANS | WM_CF_MEMDEV, 0, HOME_ICONVIEW_ID, 90, 80);
    ICONVIEW_SetFont(sHomePack.hWin, &GUI_Font13B_1);
    /*
     * Add icons to the widget
     */
    ICONVIEW_AddBitmapItem(sHomePack.hWin, &bmECGIcon, xMessage_getMessageHome()->ecgIcon);
    ICONVIEW_AddBitmapItem(sHomePack.hWin, &bmSpO2, xMessage_getMessageHome()->spO2Icon);
    ICONVIEW_AddBitmapItem(sHomePack.hWin, &bmSettingsIcon, xMessage_getMessageHome()->settingsIcon);
    ICONVIEW_AddBitmapItem(sHomePack.hWin, &bmAboutIcon, xMessage_getMessageHome()->aboutIcon);
    ICONVIEW_AddBitmapItem(sHomePack.hWin, &bmHelpIcon, xMessage_getMessageHome()->helpIcon);
    ICONVIEW_AddBitmapItem(sHomePack.hWin, &bmdebugIcon, xMessage_getMessageHome()->debugIcon);

    ICONVIEW_SetBkColor(sHomePack.hWin, ICONVIEW_CI_SEL, GUI_TRANSPARENT | 0xC0000000);

    sHomePack.hButtonHome = BUTTON_CreateEx(LCD_GetXSize() - 1 - (&bmHomeIcon)->XSize - 10,
            LCD_GetYSize() - 1 - (&bmHomeIcon)->YSize - 10, 50, 50,
            WM_HBKWIN, WM_CF_SHOW | WM_CF_MEMDEV, 0, HOME_BUTTON_ID);

    sHomePack.hMackenzieIcon = IMAGE_CreateEx((LCD_GetXSize() - 1) - (&bmMackenzieIcon)->XSize - 5,
            (LCD_GetYSize() - 1) - (&bmMackenzieIcon)->YSize - 5, (&bmMackenzieIcon)->XSize, (&bmMackenzieIcon)->YSize,
            WM_HBKWIN, WM_CF_SHOW | WM_CF_HASTRANS | WM_CF_MEMDEV, 0, MACKENZIE_ICON_ID);
    IMAGE_SetBitmap(sHomePack.hMackenzieIcon, &bmMackenzieIcon);

    WM_SetCallback(sHomePack.hButtonHome, _vHome_CallBackButtonHome);

    vHOME_HideButton();
}

/*************************************************************************************************/
