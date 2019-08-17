/**
 ******************************************************************************
 * @file    StatusBar.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 September 2017
 * @brief   Module to Manage the status bar
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "StatusBar.h"
#include "user.h"
#include "PROGBAR.h"
#include "IMAGE.h"
#include "TEXT.h"
#include "RTC.h"
#include "GUI_IDS.h"
#include "LightningIcon.h"
#include "LogBarIcon.h"

/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
static struct _sStatusBar
{
    WM_HWIN hWinStatusBar; /** @< Handler of the status bar window */
    GUI_RECT Bar; /** @< Rectangle for the status bar */
    PROGBAR_Handle hBattery; /** @< Handler of the battery progress bar*/
    TEXT_Handle hHour; /** @< Handler of the Time Text*/
    TEXT_Handle hTextDate; /** @< Handler of the Date Text*/
    IMAGE_Handle hLightningIcon; /** @< Handler of the charging icon*/
    IMAGE_Handle hLoggingIcon; /** @< Handler of the charging icon*/
    DateTime_t Time; /** @< Date/Time Object*/
    bool showBattery; /** @< Indicates that the battery icon is on */
} sStatusBar;

/*************************************************************************************************/
/**
 * @brief Draws the Battery Icon on the screen
 * @return void.
 */
static void _vStatusBar_DrawBatteryIcon(void);

/*************************************************************************************************/
/**
 * @brief Emwin Callback for the Status Bar window creation
 * @param pMsg Message from the Emwin core
 * @return void.
 */
static void _callbackBar(WM_MESSAGE * pMsg);

/*************************************************************************************************/
static void _callbackBar(WM_MESSAGE * pMsg)
{

    switch (pMsg->MsgId)
    {
        case WM_CREATE:

            sStatusBar.hHour = TEXT_CreateEx(HOUR_POS_X, HOUR_POS_Y, HOUR_WIDTH, HOUR_HEIGHT, pMsg->hWin, WM_CF_SHOW,
            TEXT_CF_LEFT, STATUS_BAR_HOUR_ID, "10:21:23");
            TEXT_SetFont(sStatusBar.hHour, &GUI_Font16B_1);
            TEXT_SetTextColor(sStatusBar.hHour, GUI_WHITE);
            TEXT_SetWrapMode(sStatusBar.hHour, GUI_WRAPMODE_WORD);

            sStatusBar.hTextDate = TEXT_CreateEx(DATE_POS_X, DATE_POS_Y, DATE_WIDTH, DATE_HEIGHT, pMsg->hWin,
            WM_CF_SHOW,
            TEXT_CF_LEFT, STATUS_BAR_DATE_ID, "20/09/2017");

            TEXT_SetFont(sStatusBar.hTextDate, &GUI_Font16B_1);
            TEXT_SetTextColor(sStatusBar.hTextDate, GUI_WHITE);
            TEXT_SetWrapMode(sStatusBar.hTextDate, GUI_WRAPMODE_WORD);

            sStatusBar.showBattery = true;
            vStatusBar_SetDate(3, 4, 2018);
            vRTC_GetDateTime(&sStatusBar.Time.sec, &sStatusBar.Time.min, &sStatusBar.Time.hour, &sStatusBar.Time.day,
                    &sStatusBar.Time.month, &sStatusBar.Time.year);
            vStatusBar_SetTime(sStatusBar.Time.sec, sStatusBar.Time.min, sStatusBar.Time.hour);
            vStatusBar_SetDate(sStatusBar.Time.day, sStatusBar.Time.month, sStatusBar.Time.year);

            sStatusBar.hLoggingIcon = IMAGE_CreateEx(BATTERY_POS_X - bmLightningIcon.XSize - bmlogIcon.XSize - 5, BATTERY_POS_Y,
                    bmlogIcon.XSize, bmlogIcon.YSize, sStatusBar.hWinStatusBar,
                    WM_CF_HIDE | WM_CF_HASTRANS | WM_CF_MEMDEV, 0, STATUS_BAR_BAT_LOGGING_ICON);
            IMAGE_SetBitmap(sStatusBar.hLoggingIcon, &bmlogIcon);

            _vStatusBar_DrawBatteryIcon();
            break;

        case WM_PAINT:
            GUI_SetColor(GUI_BLUE);
            GUI_FillRectEx(&sStatusBar.Bar);

            /* Drawing the Battery Top Rectangle */
            if (sStatusBar.showBattery)
            {
                GUI_SetColor(GUI_LIGHTGRAY);
            }
            else
            {
                GUI_SetColor(GUI_BLUE);
            }
            GUI_FillRect(BATTERY_TOP_POS_X, BATTERY_TOP_POS_Y, BATTERY_TOP_POS_X2, BATTERY_TOP_POS_Y2);
            break;
    }
}

/*************************************************************************************************/
void vStatusBar_SetTime(uint8_t sec, uint8_t min, uint8_t hour)
{
    char TimeText[10];
    sStatusBar.Time.sec = sec;
    sStatusBar.Time.min = min;
    sStatusBar.Time.hour = hour;

    vRTC_SetTime(sec, min, hour);
    if (sStatusBar.hHour)
    {
        sprintf(TimeText, "%02d:%02d:%02d", sStatusBar.Time.hour, sStatusBar.Time.min, sStatusBar.Time.sec);
        TEXT_SetText(sStatusBar.hHour, TimeText);
    }
}

/*************************************************************************************************/
void vStatusBar_SetDate(uint8_t day, uint8_t month, uint16_t year)
{

    char DateText[12];
    sStatusBar.Time.day = day;
    sStatusBar.Time.month = month;
    sStatusBar.Time.year = year;

    vRTC_SetDate(day, month, year);
    if (sStatusBar.hTextDate)
    {
        sprintf(DateText, "%02d/%02d/%04d", sStatusBar.Time.day, sStatusBar.Time.month, sStatusBar.Time.year);
        TEXT_SetText(sStatusBar.hTextDate, DateText);
    }
}

/*************************************************************************************************/
void vStatusBar_AddStatusBar(WM_HWIN hWinParent)
{

    sStatusBar.Bar.x0 = STATUS_BAR_POS_X;
    sStatusBar.Bar.x1 = STATUS_BAR_WIDTH;
    sStatusBar.Bar.y0 = STATUS_BAR_POS_Y;
    sStatusBar.Bar.y1 = STATUS_BAR_HEIGHT;

    sStatusBar.hWinStatusBar = WM_CreateWindowAsChild(sStatusBar.Bar.x0, sStatusBar.Bar.y0, sStatusBar.Bar.x1,
            sStatusBar.Bar.y1, hWinParent,
            WM_CF_SHOW | WM_CF_MEMDEV, _callbackBar, 0);
}

/*************************************************************************************************/
WM_HWIN hStatusBar_Win(void)
{
    return sStatusBar.hWinStatusBar;
}

/*************************************************************************************************/
void vStatusBar_setBatteryDetected(bool isBatteryDetected)
{
    if (isBatteryDetected)
    {
        sStatusBar.showBattery = true;
        WM_ShowWindow(sStatusBar.hBattery);
    }
    else
    {
        sStatusBar.showBattery = false;
        WM_HideWindow(sStatusBar.hBattery);
    }
    WM_Paint(sStatusBar.hWinStatusBar);
}

/*************************************************************************************************/
void vStatusBar_setCharging(bool isCharging)
{
    if (isCharging)
    {
        WM_ShowWindow(sStatusBar.hLightningIcon);
        WM_BringToTop(sStatusBar.hLightningIcon);
    }
    else
    {
        WM_HideWindow(sStatusBar.hLightningIcon);
    }
}
/*************************************************************************************************/
void vStatusBar_setBatteryPercentage(uint8_t value)
{
    GUI_COLOR colorWarning = GUI_RED;
    if (value > 50 && value < 100)
    {
        colorWarning = GUI_GREEN;
    }
    else if (value > 25)
    {
        colorWarning = GUI_YELLOW;
    }
    else
    {
        colorWarning = GUI_RED;
    }
    PROGBAR_SetBarColor(sStatusBar.hBattery, 0, colorWarning);
    PROGBAR_SetValue(sStatusBar.hBattery, value);
}

/*************************************************************************************************/
void vStatusBar_setLogging(bool isLogging)
{
    if (isLogging)
    {
        WM_ShowWindow(sStatusBar.hLoggingIcon);
        WM_BringToTop(sStatusBar.hLoggingIcon);
    }
    else
    {
        WM_HideWindow(sStatusBar.hLoggingIcon);
    }
}

/*************************************************************************************************/
static void _vStatusBar_DrawBatteryIcon(void)
{
    sStatusBar.hBattery = PROGBAR_CreateEx(BATTERY_POS_X, BATTERY_POS_Y, BATTERY_WIDTH, BATTERY_HEIGHT,
            sStatusBar.hWinStatusBar, WM_CF_SHOW, PROGBAR_CF_HORIZONTAL, STATUS_BAR_BAT_ID);
    PROGBAR_SetMinMax(sStatusBar.hBattery, 0, 100);
    PROGBAR_SetValue(sStatusBar.hBattery, 0);
    WM_HideWindow(sStatusBar.hBattery);
    sStatusBar.showBattery = false;

    sStatusBar.hLightningIcon = IMAGE_CreateEx(BATTERY_POS_X - bmLightningIcon.XSize, BATTERY_POS_Y,
            bmLightningIcon.XSize, bmLightningIcon.YSize, sStatusBar.hWinStatusBar,
            WM_CF_HIDE | WM_CF_HASTRANS | WM_CF_MEMDEV, 0, STATUS_BAR_BAT_CHARGING_ICON);
    IMAGE_SetBitmap(sStatusBar.hLightningIcon, &bmLightningIcon);

}
/*************************************************************************************************/
