/**
 ******************************************************************************
 * @file    SettingsScreen.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 July 2017
 * @brief   Module to manage Settings Window
 ******************************************************************************
 */

#include <stdio.h>
#include "SLIDER.h"
#include "PROGBAR.h"
#include "TEXT.h"
#include "user.h"
#include "ICONVIEW.h"
#include "BUTTON.h"
#include "SWIPELIST.h"
#include "DROPDOWN.h"
#include "LISTWHEEL.h"
#include "CALENDAR.h"
#include "SPINBOX.h"
#include "DIALOG.h"
#include "TEXT.h"
#include "Display.h"
#include "SettingsScreen.h"
#include "BatteryCharger.h"
#include "MCU_Flash.h"
#include "PWM.h"
#include "RTC.h"
#include "StatusBar.h"
#include "TouchIcon.h"
#include "user.h"
#include "CalendarIcon.h"
#include "BatteryIcon.h"
#include "ClockIcon.h"
#include "Messages.h"
#include "BrightnessIcon.h"
#include "LanguageIcon.h"
#include "Home.h"
#include "GUI_IDS.h"

/*************************************************************************************************/
/**
 * @enum States for the state machine touch calibration
 */
typedef enum _stateTouchCal
{
    _WAIT_NONE, /** @< Undefined state */
    _WAIT_FIRST_PRESS, /** @< Wating for pressing the screen to start calibration */
    _WAIT_RELEASE_FIRST, /** @<  Wating for releasing the screen to start calibration */
    _WAIT_FIRST_CIRCLE, /** @< Wating for pressing the first calibration point */
    _WAIT_RELEASE_FIRST_CIRCLE, /** @< Wating for releasing the first calibration point */
    _WAIT_SECOND_CIRCLE, /** @< Wating for pressing the second calibration point */
    _WAIT_RELEASE_SECOND_CIRCLE, /** @< Wating for releasing the second calibration point */
    _WAIT_THIRD_CIRCLE, /** @< Wating for pressing the third calibration point */
    _WAIT_RELEASE_THIRD_CIRCLE, /** @< Wating for releasing the third calibration point */
    _WAIT_FOURTH_CIRCLE, /** @< Wating for pressing the fourth calibration point */
    _WAIT_RELEASE_FOURTH_CIRCLE, /** @< Wating for releasing the fourth calibration point */
    _WAIT_FIFITH_CIRCLE, /** @< Wating for pressing the fifth calibration point */
    _WAIT_RELEASE_FIFITH_CIRCLE, /** @< Wating for releasing the fifth calibration point */
    _WAIT_FINISH_PRESS, /** @< Wating for pressing the screen to finish calibration */
    _WAIT_FINISH /** @< WCalibration finished, saving data */
} stateTouchCal;

typedef enum _stateBatInfo
{
    _BATINFO_HIDE, _BATINFO_SHOW, _BATINFO_WAITPRESS,
} stateBatInfo;
/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
static struct _sSettingsPack
{
    WM_HWIN hWinSettings; /** @< emWin Window handler for the settings window */
    WM_HWIN hCalendar; /** @< emWin Widget handler for the calendar */
    SLIDER_Handle hSlider0; /** @< emWin Widget handler for the brightness slider */
    SWIPELIST_Handle hSwipe; /** @< emWin Widget handler for the swipe window */
    BUTTON_Handle hButtonOK; /** @< emWin Widget handler for the calendar button Ok */
    BUTTON_Handle hButtonCancel; /** @< emWin Widget handler for the calendar button Cancel */
    CALENDAR_DATE pDate; /** @< structure to work with the date for calendar widget */
    bool isOnCalendarWin; /** @< flag to signal that the calendar is on the screen (Open) */
    /*************************************************************************************************/
    /**
     * @struct Structure containning data to manage the touch calibration process
     */
    struct
    {
        stateTouchCal stateCal; /** @< state machine for the calibration process */
        int aPhysX[5]; /** @< Touch ADC physical value X */
        int aPhysY[5]; /** @< Touch ADC physical value Y */
        int aLogX[5]; /** @< Touch logical value X */
        int aLogY[5]; /** @< Touch logical value Y */
    } TouchCalData;
    stateBatInfo stateBatteryInfo; /** @< Battery Info State */
} sSettingsPack;

/*************************************************************************************************/
/**
 * @var Language List Items
 */
static const char * apTextLangList[] = { "English", "Português",
NULL };

/*************************************************************************************************/
/**
 * @var emWin Diaglog Items
 */
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = { { WINDOW_CreateIndirect, "SettingsWindow", WINDOW_SETTINGS_ID,
        0, 0, XSIZE_PHYS, YSIZE_PHYS, WM_CF_HIDE | WM_CF_MEMDEV, 0x0, 0 }, { SWIPELIST_CreateIndirect, "Swipelist",
        SWIPE_ID, SWIPE_SETTINGS_POS_X, SWIPE_SETTINGS_POS_Y, XSIZE_PHYS,
        YSIZE_PHYS, WM_CF_SHOW | WM_CF_MEMDEV, 0x0, 0 }, };

/*************************************************************************************************/
/**
 * @brief creates the swipe list containing all settings items
 * @param hParent parent window
 * @return void.
 */
static void _vSettings_CreateSwipeList(WM_HWIN hParent);

/*************************************************************************************************/
/**
 * @brief emWin callback for the Settings dialog
 * @param pMsg emWin message.
 * @return void.
 */
static void _vSettings_CallBackDialog(WM_MESSAGE * pMsg);

/*************************************************************************************************/
/**
 * @brief emWin callback for the Settings window
 * @param pMsg emWin message.
 * @return void.
 */
static void _vSettings_CallBackSettingsWindow(WM_MESSAGE *pMsg);

/*************************************************************************************************/
/**
 * @brief Exhibits the touch calibration point circle on the screen
 * @param i index for the touch calibration point
 * @return void.
 */
static void _vSettings_MessageWaitCircle(int i);

/*************************************************************************************************/
/**
 * @brief Print a centered string on the screen
 * @param pString string to be printe
 * @return void.
 */
static void _vDispStringCentered(const char * pString);

/*************************************************************************************************/
/**
 * @brief Creates a screen with battery information
 * @return void.
 */
static void _vSettings_ShowBatteryStatus(void);
/*************************************************************************************************/
static void _vSettings_CallBackSettingsWindow(WM_MESSAGE *pMsg)
{
    switch (pMsg->MsgId)
    {

        case WM_CREATE:
            SWIPELIST_SetDefaultBkColor(SWIPELIST_CI_BK_ITEM_SEL, GUI_BLACK);
            GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _vSettings_CallBackDialog, pMsg->hWin, 0,
                    0);

            break;

        default:
            WM_DefaultProc(pMsg);
            break;

    }
}

/*************************************************************************************************/
static void svSettings_CallBackSwipeList(WM_MESSAGE *pMsg)
{
    int Id, NCode;
    uint8_t hour, min, sec;
    static CALENDAR_DATE _pDate;
    WM_HWIN hItem;

    switch (pMsg->MsgId)
    {

        default:
            SWIPELIST_Callback(pMsg);
            break;

        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;

            switch (Id)
            {
                case BUTTON_SAVE_LANG_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            if (!sSettingsPack.isOnCalendarWin)
                            {

                                hItem = WM_GetDialogItem(pMsg->hWin, LANGUAGE_LIST_ID);
                                vMessages_setLanguage((eLanguage) DROPDOWN_GetSel(hItem));
                                WM_MOTION_Enable(1);

                                if (stMCU_Flash_saveKnownData(_FLASH_USER_LANGUAGE,
                                        (void *) xMessages_getLanguageAddr()) == kStatus_Fail)
                                {
                                    vMessages_Init();
                                }

                                NVIC_SystemReset();
                            }
                            break;
                    }
                    break;

                case BUTTON_SAVE_TIME_ID:
                    switch (NCode)
                    {

                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            hItem = WM_GetDialogItem(pMsg->hWin, HOUR_SPIN_ID);
                            hour = SPINBOX_GetValue(hItem);

                            hItem = WM_GetDialogItem(pMsg->hWin, MIN_SPIN_ID);
                            min = SPINBOX_GetValue(hItem);

                            hItem = WM_GetDialogItem(pMsg->hWin, SEC_SPIN_ID);
                            sec = SPINBOX_GetValue(hItem);

                            vStatusBar_SetTime(sec, min, hour);
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

                case BRIGHTNESS_SLIDER_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            WM_MOTION_Enable(1);
                            break;

                        case WM_NOTIFICATION_VALUE_CHANGED:
                            hItem = WM_GetDialogItem(pMsg->hWin, BRIGHTNESS_SLIDER_ID);
                            vPWM_UpdateDutyCycle(_E_PWM_TFT_BACKLIGHT, (uint8_t) SLIDER_GetValue(hItem));
                            WM_MOTION_Enable(0);
                            break;
                    }
                    break;

                case HOUR_SPIN_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

                case MIN_SPIN_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

                case SEC_SPIN_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

                case LANGUAGE_LIST_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

                case CALENDAR_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;
                    }
                    break;

                case BUTTON_OK_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            sSettingsPack.isOnCalendarWin = false;
                            hItem = WM_GetDialogItem(pMsg->hWin, BUTTON_OK_ID);
                            BUTTON_IsPressed(hItem);
                            CALENDAR_GetSel(sSettingsPack.hCalendar, &_pDate);
                            CALENDAR_SetDate(sSettingsPack.hCalendar, &_pDate);
                            vStatusBar_SetDate(_pDate.Day, _pDate.Month, _pDate.Year);

                            vSettings_HideSettings();
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

                case BUTTON_CANCEL_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_CLICKED:
                            WM_MOTION_Enable(0);
                            break;

                        case WM_NOTIFICATION_RELEASED:
                            sSettingsPack.isOnCalendarWin = false;
                            hItem = WM_GetDialogItem(pMsg->hWin, BUTTON_CANCEL_ID);
                            BUTTON_IsPressed(hItem);
                            vSettings_HideSettings();
                            WM_MOTION_Enable(1);
                            break;
                    }
                    break;

            }
            break;

    }
}

/*************************************************************************************************/
static int svSettings_CallBackSwipeOwnerDraw(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo)
{
    //
    // Distinguish between different commands
    //
    switch (pDrawItemInfo->Cmd)
    {
        case WIDGET_ITEM_DRAW_SEP:
            GUI_DrawGradientH(pDrawItemInfo->x0, pDrawItemInfo->y0,
                    pDrawItemInfo->x1 - (pDrawItemInfo->x1 - pDrawItemInfo->x0) / 2, pDrawItemInfo->y1, GUI_DARKGRAY,
                    GUI_BLACK);
            GUI_DrawGradientH(pDrawItemInfo->x1 - (pDrawItemInfo->x1 - pDrawItemInfo->x0) / 2, pDrawItemInfo->y0,
                    pDrawItemInfo->x1, pDrawItemInfo->y1, GUI_BLACK, GUI_DARKGRAY);
            break;
        case WIDGET_ITEM_DRAW_TEXT:
            //
            // Just set a text mode but let the default owner draw routine handle the rest
            //
            GUI_SetTextMode(GUI_TM_TRANS);
            SWIPELIST_OwnerDraw(pDrawItemInfo);
            break;

        case WIDGET_ITEM_DRAW_BITMAP:
            SWIPELIST_OwnerDraw(pDrawItemInfo);
            break;

        case WIDGET_ITEM_DRAW_BACKGROUND:
            //
            // Handle drawing of the background of the items
            //
            switch (pDrawItemInfo->ItemIndex)
            {
                case _SETTINGS_GENERAL:
                case _SETTINGS_INPUT:
                    GUI_SetColor(GUI_BLACK);
                    GUI_FillRect(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1, pDrawItemInfo->y1);
                    GUI_DrawGradientV(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1,
                            pDrawItemInfo->y1 - (pDrawItemInfo->y1 - pDrawItemInfo->y0) / 2, GUI_BLACK,
                            GUI_GRAY);
                    GUI_SetColor(GUI_LIGHTGRAY);
                    GUI_DrawRoundedRect(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1, pDrawItemInfo->y1, 5);
                    return 0;

                case _SETTINGS_BRIGHTNESS:
                case _SETTINGS_TIME:
                case _SETTINGS_LANGUAGE:
                    GUI_SetColor(GUI_DARKGRAY);
                    GUI_FillRect(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1, pDrawItemInfo->y1);
                    break;

                default:     // Any other item
                    //
                    // Determine     if the item to drawn is the currently selected
                    //
                    if (SWIPELIST_GetSelItem(pDrawItemInfo->hWin) == pDrawItemInfo->ItemIndex)
                    {
                        //
                        // Draw the selected one different
                        //
                        GUI_SetColor(GUI_DARKRED);
                        GUI_FillRect(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1, pDrawItemInfo->y1);
                        GUI_DrawGradientV(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1,
                                pDrawItemInfo->y1 - (pDrawItemInfo->y1 - pDrawItemInfo->y0) * 2 / 3, GUI_BLACK,
                                GUI_DARKRED);
                        GUI_DrawGradientV(pDrawItemInfo->x0,
                                pDrawItemInfo->y1 - (pDrawItemInfo->y1 - pDrawItemInfo->y0) / 3, pDrawItemInfo->x1,
                                pDrawItemInfo->y1, GUI_DARKRED,
                                GUI_BLACK);
                    }
                    else
                    {
                        //
                        // Draw any other items this way
                        //
                        GUI_SetColor(GUI_DARKGRAY);
                        GUI_FillRect(pDrawItemInfo->x0, pDrawItemInfo->y0, pDrawItemInfo->x1, pDrawItemInfo->y1);
                    }
            }
            break;
        default:
            //
            // Anything we do not catch in this routine gets handled by the default owner draw
            //
            return SWIPELIST_OwnerDraw(pDrawItemInfo);
    }
    return 0;
}

/*************************************************************************************************/
static void _vSettings_CallBackDialog(WM_MESSAGE * pMsg)
{
    WM_HWIN hItem;
    WM_HWIN hItem2;
    int NCode;
    int Id;
    int Sel;
    uint8_t sec,min,hour;
    switch (pMsg->MsgId)
    {
        case WM_INIT_DIALOG:
            sSettingsPack.hSwipe = WM_GetDialogItem(pMsg->hWin, SWIPE_ID);
            WM_SetCallback(sSettingsPack.hSwipe, svSettings_CallBackSwipeList);
            WM_MOTION_Enable(1);
            WM_MOTION_SetMoveable(sSettingsPack.hSwipe, WM_CF_MOTION_Y, 1);
            WM_MOTION_SetSpeed(sSettingsPack.hSwipe, GUI_COORD_Y, 1);
            _vSettings_CreateSwipeList(pMsg->hWin);
            SWIPELIST_SetOwnerDraw(sSettingsPack.hSwipe, svSettings_CallBackSwipeOwnerDraw);
            break;

        case WM_PID_STATE_CHANGED:
            if (GUI_PID_IsPressed())
            {
                if (sSettingsPack.stateBatteryInfo == _BATINFO_WAITPRESS)
                {
                    hItem2 = WM_GetDialogItem(pMsg->hWin, SWIPE_ID);
                    WM_ShowWindow(hItem2);
                    WM_MOTION_Enable(1);
                    vHOME_ShowButton();
                    sSettingsPack.stateBatteryInfo = _BATINFO_HIDE;
                }
                else if (sSettingsPack.stateBatteryInfo == _BATINFO_HIDE)
                {
                    switch (sSettingsPack.TouchCalData.stateCal)
                    {
                        case _WAIT_FIRST_PRESS:
                            sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_FIRST;
                            WM_Paint(pMsg->hWin);
                            break;

                        case _WAIT_FIRST_CIRCLE:
                            sSettingsPack.TouchCalData.aPhysX[0] = i32TouchScreen_GetxPhys();
                            sSettingsPack.TouchCalData.aPhysY[0] = i32TouchScreen_GetyPhys();
                            sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_FIRST_CIRCLE;
                            WM_Paint(pMsg->hWin);
                            break;

                        case _WAIT_SECOND_CIRCLE:
                            sSettingsPack.TouchCalData.aPhysX[1] = i32TouchScreen_GetxPhys();
                            sSettingsPack.TouchCalData.aPhysY[1] = i32TouchScreen_GetyPhys();
                            sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_SECOND_CIRCLE;
                            WM_Paint(pMsg->hWin);
                            break;

                        case _WAIT_THIRD_CIRCLE:
                            sSettingsPack.TouchCalData.aPhysX[2] = i32TouchScreen_GetxPhys();
                            sSettingsPack.TouchCalData.aPhysY[2] = i32TouchScreen_GetyPhys();
                            sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_THIRD_CIRCLE;
                            WM_Paint(pMsg->hWin);
                            break;

                        case _WAIT_FOURTH_CIRCLE:
                            sSettingsPack.TouchCalData.aPhysX[3] = i32TouchScreen_GetxPhys();
                            sSettingsPack.TouchCalData.aPhysY[3] = i32TouchScreen_GetyPhys();
                            sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_FOURTH_CIRCLE;
                            WM_Paint(pMsg->hWin);
                            break;

                        case _WAIT_FIFITH_CIRCLE:
                            sSettingsPack.TouchCalData.aPhysX[4] = i32TouchScreen_GetxPhys();
                            sSettingsPack.TouchCalData.aPhysY[4] = i32TouchScreen_GetyPhys();
                            sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_FIFITH_CIRCLE;
                            WM_Paint(pMsg->hWin);
                            break;

                        case _WAIT_FINISH:
                            sSettingsPack.TouchCalData.stateCal = _WAIT_NONE;

                            hItem2 = WM_GetDialogItem(pMsg->hWin, SWIPE_ID);
                            WM_ShowWindow(hItem2);
                            WM_MOTION_Enable(1);
                            vHOME_ShowButton();
                            break;

                        default:
                            WM_DefaultProc(pMsg);
                            break;
                    }
                }
                break;

            }
            else
            {
                switch (sSettingsPack.TouchCalData.stateCal)
                {
                    case _WAIT_RELEASE_FIRST:
                        sSettingsPack.TouchCalData.stateCal = _WAIT_FIRST_CIRCLE;
                        WM_Paint(pMsg->hWin);
                        break;

                    case _WAIT_RELEASE_FIRST_CIRCLE:
                        sSettingsPack.TouchCalData.stateCal = _WAIT_SECOND_CIRCLE;
                        WM_Paint(pMsg->hWin);
                        break;

                    case _WAIT_RELEASE_SECOND_CIRCLE:
                        sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_THIRD_CIRCLE;
                        WM_Paint(pMsg->hWin);
                        break;

                    case _WAIT_RELEASE_THIRD_CIRCLE:
                        sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_FOURTH_CIRCLE;
                        WM_Paint(pMsg->hWin);
                        break;

                    case _WAIT_RELEASE_FOURTH_CIRCLE:
                        sSettingsPack.TouchCalData.stateCal = _WAIT_RELEASE_FIFITH_CIRCLE;
                        WM_Paint(pMsg->hWin);
                        break;

                    case _WAIT_RELEASE_FIFITH_CIRCLE:
                        sSettingsPack.TouchCalData.stateCal = _WAIT_FINISH_PRESS;
                        WM_Paint(pMsg->hWin);
                        break;

                    default:
                        WM_DefaultProc(pMsg);
                        break;
                }
            }

            break;

        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;
            switch (Id)
            {
                case SWIPE_ID:
                    switch (NCode)
                    {
                        case WM_NOTIFICATION_RELEASED:
                            hItem = WM_GetDialogItem(pMsg->hWin, SWIPE_ID);
                            Sel = SWIPELIST_GetReleasedItem(hItem);
                            switch (Sel)
                            {
                                case _SETTINGS_BRIGHTNESS:
                                    break;

                                case _SETTINGS_TIME:
                                    break;

                                case _SETTINGS_BATTERY:
                                    if (!sSettingsPack.isOnCalendarWin)
                                    {
                                        hItem2 = WM_GetDialogItem(pMsg->hWin, SWIPE_ID);
                                        WM_HideWindow(hItem2);
                                        WM_MOTION_Enable(0);
                                        vHOME_HideButton();
                                        sSettingsPack.stateBatteryInfo = _BATINFO_SHOW;
                                    }
                                    break;

                                case _SETTINGS_DATE:
                                    sSettingsPack.isOnCalendarWin = true;
                                    /**************** Creating HOUR Spin **************************************/
                                    //Grab the actual date and time
                                    vRTC_GetDateTime((uint8_t *) &sec, (uint8_t *) &min, (uint8_t *) &hour, (uint8_t *) &sSettingsPack.pDate.Day,
                                            (uint8_t *) &sSettingsPack.pDate.Month, (uint16_t *) &sSettingsPack.pDate.Year);
                                    CALENDAR_SetDate(sSettingsPack.hCalendar, &sSettingsPack.pDate);
                                    WM_MOTION_Enable(0);
                                    vSettings_ShowSettings();
                                    break;

                                case _SETTINGS_TOUCHCAL:
                                    if (!sSettingsPack.isOnCalendarWin)
                                    {
                                        hItem2 = WM_GetDialogItem(pMsg->hWin, SWIPE_ID);
                                        WM_HideWindow(hItem2);
                                        WM_MOTION_Enable(0);
                                        vHOME_HideButton();

                                        sSettingsPack.TouchCalData.stateCal = _WAIT_FIRST_PRESS;
                                    }
                                    break;

                            }
                            break;
                    }
                    break;

            }
            break;

        case WM_PAINT:
            if (sSettingsPack.stateBatteryInfo == _BATINFO_SHOW || sSettingsPack.stateBatteryInfo == _BATINFO_WAITPRESS)
            {
                _vSettings_ShowBatteryStatus();
                sSettingsPack.stateBatteryInfo = _BATINFO_WAITPRESS;
            }
            else if (sSettingsPack.stateBatteryInfo == _BATINFO_HIDE)
            {
                switch (sSettingsPack.TouchCalData.stateCal)
                {
                    case _WAIT_FIRST_PRESS:
                    case _WAIT_RELEASE_FIRST:

                        GUI_SetBkColor(GUI_WHITE);
                        GUI_Clear();
                        if (sSettingsPack.TouchCalData.stateCal == _WAIT_FIRST_PRESS)
                        {
                            /* Set the logical values */
                            sSettingsPack.TouchCalData.aLogX[0] = TOUCHCAL_POINT1_POS_X;
                            sSettingsPack.TouchCalData.aLogY[0] = TOUCHCAL_POINT1_POS_Y;
                            sSettingsPack.TouchCalData.aLogX[1] = TOUCHCAL_POINT2_POS_X;
                            sSettingsPack.TouchCalData.aLogY[1] = TOUCHCAL_POINT2_POS_Y;
                            sSettingsPack.TouchCalData.aLogX[2] = TOUCHCAL_POINT3_POS_X;
                            sSettingsPack.TouchCalData.aLogY[2] = TOUCHCAL_POINT3_POS_Y;
                            sSettingsPack.TouchCalData.aLogX[3] = TOUCHCAL_POINT4_POS_X;
                            sSettingsPack.TouchCalData.aLogY[3] = TOUCHCAL_POINT4_POS_Y;
                            sSettingsPack.TouchCalData.aLogX[4] = TOUCHCAL_POINT5_POS_X;
                            sSettingsPack.TouchCalData.aLogY[4] = TOUCHCAL_POINT5_POS_Y;
                        }
                        GUI_SetFont(&GUI_Font16B_1);
                        GUI_SetColor(GUI_BLACK);
                        _vDispStringCentered(xMessage_getMessageSettings()->pressTouch);
                        GUI_DispStringHCenterAt(xMessage_getMessageSettings()->touchCalibration, LCD_GetXSize() / 2,
                        STATUS_BAR_HEIGHT + 5);
                        break;

                    case _WAIT_FIRST_CIRCLE:
                    case _WAIT_RELEASE_FIRST_CIRCLE:
                        _vSettings_MessageWaitCircle(0);
                        break;

                    case _WAIT_SECOND_CIRCLE:
                    case _WAIT_RELEASE_SECOND_CIRCLE:
                        _vSettings_MessageWaitCircle(1);
                        break;

                    case _WAIT_THIRD_CIRCLE:
                    case _WAIT_RELEASE_THIRD_CIRCLE:
                        _vSettings_MessageWaitCircle(2);
                        break;

                    case _WAIT_FOURTH_CIRCLE:
                    case _WAIT_RELEASE_FOURTH_CIRCLE:
                        _vSettings_MessageWaitCircle(3);
                        break;

                    case _WAIT_FIFITH_CIRCLE:
                    case _WAIT_RELEASE_FIFITH_CIRCLE:
                        _vSettings_MessageWaitCircle(4);
                        break;

                    case _WAIT_FINISH_PRESS:
                    case _WAIT_FINISH:
                        if (sSettingsPack.TouchCalData.stateCal == _WAIT_FINISH_PRESS)
                        {
                            /* Use the physical values to calibrate the touch screen */
                            bTouchScreen_Calibrate(GUI_COORD_X, sSettingsPack.TouchCalData.aLogX[0],
                                    sSettingsPack.TouchCalData.aLogX[1],
                                    sSettingsPack.TouchCalData.aPhysX[0] + sSettingsPack.TouchCalData.aLogX[0] + 1,
                                    sSettingsPack.TouchCalData.aPhysX[1] + sSettingsPack.TouchCalData.aLogX[0] + 1); /* Calibrate X-axis */
                            bTouchScreen_Calibrate(GUI_COORD_Y, sSettingsPack.TouchCalData.aLogY[0],
                                    sSettingsPack.TouchCalData.aLogY[1],
                                    sSettingsPack.TouchCalData.aPhysY[0] - sSettingsPack.TouchCalData.aLogX[0],
                                    sSettingsPack.TouchCalData.aPhysY[1] - sSettingsPack.TouchCalData.aLogX[0]); /* Calibrate Y-axis */

                            sSettingsPack.TouchCalData.stateCal = _WAIT_FINISH;
                        }
                        GUI_SetBkColor(GUI_WHITE);
                        GUI_Clear();
                        GUI_SetFont(&GUI_Font16B_1);
                        GUI_SetColor(GUI_BLACK);
                        _vDispStringCentered(xMessage_getMessageSettings()->calibrationDone);

                        break;

                    default:
                        WM_DefaultProc(pMsg);
                        break;
                }
            }
            break;

        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

/*************************************************************************************************/
void vSettings_HideSettings(void)
{
    WM_HideWindow(sSettingsPack.hCalendar);
    WM_HideWindow(sSettingsPack.hButtonOK);
    WM_HideWindow(sSettingsPack.hButtonCancel);
    WM_ShowWindow(sSettingsPack.hSlider0);

}

/*************************************************************************************************/
void vSettings_ShowSettings(void)
{
    WM_ShowWindow(sSettingsPack.hCalendar);
    WM_ShowWindow(sSettingsPack.hButtonOK);
    WM_ShowWindow(sSettingsPack.hButtonCancel);
    WM_HideWindow(sSettingsPack.hSlider0);
}

/*************************************************************************************************/
void vSettings_AddSettingsScreen(void)
{
    sSettingsPack.hWinSettings = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN,
    WM_CF_HIDE | WM_CF_MEMDEV, _vSettings_CallBackSettingsWindow, 0);
}

/*************************************************************************************************/
static void svSettings_CallbackEditSpinBox(WM_MESSAGE *pMsg)
{
    switch (pMsg->MsgId)
    {
        case WM_PID_STATE_CHANGED:
            if (GUI_PID_IsPressed())
            {
                WM_MOTION_Enable(0);
            }
            else
            {
                WM_MOTION_Enable(1);
            }
            break;

        default:
            EDIT_Callback(pMsg);
            break;
    }
}

/*************************************************************************************************/
void _vSettings_CreateSwipeList(WM_HWIN hParent)
{
    TEXT_Handle hTextHour, hTextMin;
    SPINBOX_Handle hSpinHour, hSpinMin, hSpinSec;
    BUTTON_Handle hButtonSaveTime, hButtonSaveLang;
    DROPDOWN_Handle hDropLangList;
    EDIT_Handle hEdit;
    uint8_t hour, min, sec;

    sSettingsPack.isOnCalendarWin = false;
    sSettingsPack.TouchCalData.stateCal = _WAIT_NONE;
    sSettingsPack.stateBatteryInfo = _BATINFO_HIDE;
    /**************** Adding General Settings Itens **************************************/
    SWIPELIST_AddSepItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->sepTextGeneral, SWIPE_ITEM_SEP_SIZE);
    //Brightness
    SWIPELIST_AddItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->itemBrightness, SWIPE_ITEM_SIZE);
    SWIPELIST_AddItemText(sSettingsPack.hSwipe, _SETTINGS_BRIGHTNESS,
            xMessage_getMessageSettings()->itemTextBrightness);
    //Time
    SWIPELIST_AddItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->itemTime, SWIPE_ITEM_SIZE_TIME);
    SWIPELIST_AddItemText(sSettingsPack.hSwipe, _SETTINGS_TIME, xMessage_getMessageSettings()->itemTextTime);
    //Date
    SWIPELIST_AddItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->itemDate, SWIPE_ITEM_SIZE);
    SWIPELIST_AddItemText(sSettingsPack.hSwipe, _SETTINGS_DATE, xMessage_getMessageSettings()->itemTextDate);
    //Battery
    SWIPELIST_AddItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->itemBattery, SWIPE_ITEM_SIZE);
    SWIPELIST_AddItemText(sSettingsPack.hSwipe, _SETTINGS_BATTERY, xMessage_getMessageSettings()->itemTextBattery);
    SWIPELIST_SetBitmap(sSettingsPack.hSwipe, _SETTINGS_BATTERY, SWIPELIST_BA_LEFT | SWIPELIST_BA_VCENTER,
            &bmBatteryIcon);

    /**************** Adding Input Settings Itens **************************************/
    SWIPELIST_AddSepItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->sepTextInput, SWIPE_ITEM_SEP_SIZE);
    //Touch Calibration
    SWIPELIST_AddItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->itemTouchCal, SWIPE_ITEM_SIZE);
    SWIPELIST_AddItemText(sSettingsPack.hSwipe, _SETTINGS_TOUCHCAL, xMessage_getMessageSettings()->itemTextTouchCal);
    //Language
    SWIPELIST_AddItem(sSettingsPack.hSwipe, xMessage_getMessageSettings()->itemLanguage, SWIPE_ITEM_SIZE_LANG);
    SWIPELIST_AddItemText(sSettingsPack.hSwipe, _SETTINGS_LANGUAGE, xMessage_getMessageSettings()->itemTextLanguage);

    SWIPELIST_AddSepItem(sSettingsPack.hSwipe, "END", SWIPE_ITEM_SEP_SIZE);

    SWIPELIST_SetBitmap(sSettingsPack.hSwipe, _SETTINGS_TOUCHCAL, SWIPELIST_BA_LEFT | SWIPELIST_BA_VCENTER,
            &bmTouchIcon);

    /**************** Creating Brightness Slider **************************************/
    sSettingsPack.hSlider0 = SLIDER_CreateEx(BRIGHTNESS_SLIDER_POS_X, BRIGHTNESS_SLIDER_POS_Y,
    BRIGHTNESS_SLIDER_WIDTH,
    BRIGHTNESS_SLIDER_HEIGHT, hParent,
    WM_CF_SHOW | WM_CF_STAYONTOP | WM_CF_MEMDEV, SLIDER_CF_HORIZONTAL, BRIGHTNESS_SLIDER_ID);
    SLIDER_SetRange(sSettingsPack.hSlider0, BRIGHTNESS_MIN_VALUE, BRIGHTNESS_MAX_VALUE);
    SLIDER_SetValue(sSettingsPack.hSlider0, uiPWM_GetDutyCycle(_E_PWM_TFT_BACKLIGHT));
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_BRIGHTNESS, sSettingsPack.hSlider0,
    BRIGHTNESS_SLIDER_POS_X, BRIGHTNESS_SLIDER_POS_Y);

    SWIPELIST_SetBitmap(sSettingsPack.hSwipe, _SETTINGS_BRIGHTNESS, SWIPELIST_BA_LEFT | SWIPELIST_BA_VCENTER,
            &bmBrightnessIcon);

    /**************** Creating HOUR Spin **************************************/
    //Grab the actual date and time
    vRTC_GetDateTime((uint8_t *) &sec, (uint8_t *) &min, (uint8_t *) &hour, (uint8_t *) &sSettingsPack.pDate.Day,
            (uint8_t *) &sSettingsPack.pDate.Month, (uint16_t *) &sSettingsPack.pDate.Year);

    hSpinHour = SPINBOX_CreateEx(HOUR_SPIN_POS_X, HOUR_SPIN_POS_Y, HOUR_SPIN_WIDTH, HOUR_SPIN_HEIGHT, hParent,
    WM_CF_SHOW, HOUR_SPIN_ID, 0, HOUR_SPIN_MAX_VALUE);
    SPINBOX_SetEdge(hSpinHour, SPINBOX_EDGE_CENTER);
    SPINBOX_SetButtonSize(hSpinHour, HOUR_SPIN_BUTTON_SIZE);
    SPINBOX_SetFont(hSpinHour, &GUI_Font24B_1);
    SPINBOX_SetValue(hSpinHour, hour);
    SPINBOX_SetEditMode(hSpinHour, 1);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_TIME, hSpinHour, HOUR_SPIN_POS_X, HOUR_SPIN_POS_Y);

    hEdit = SPINBOX_GetEditHandle(hSpinHour);
    WM_SetCallback(hEdit, svSettings_CallbackEditSpinBox);

    hTextHour = TEXT_CreateEx(HOUR_TEXT_POS_X, HOUR_TEXT_POS_Y, 5, HOUR_SPIN_HEIGHT, hParent, WM_CF_SHOW, 0,
            HOUR_TEXT_ID, ":");
    TEXT_SetFont(hTextHour, &GUI_Font24B_1);
    TEXT_SetTextColor(hTextHour, GUI_WHITE);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_TIME, hTextHour, HOUR_TEXT_POS_X, HOUR_SPIN_POS_Y);

    /**************** Creating MIN Spin **************************************/
    hSpinMin = SPINBOX_CreateEx(MIN_SPIN_POS_X, MIN_SPIN_POS_Y, MIN_SPIN_WIDTH, MIN_SPIN_HEIGHT, hParent,
    WM_CF_SHOW, MIN_SPIN_ID, 0, MIN_SPIN_MAX_VALUE);
    SPINBOX_SetEdge(hSpinMin, SPINBOX_EDGE_CENTER);
    SPINBOX_SetButtonSize(hSpinMin, MIN_SPIN_BUTTON_SIZE);
    SPINBOX_SetFont(hSpinMin, &GUI_Font24B_1);
    SPINBOX_SetValue(hSpinMin, min);
    SPINBOX_SetEditMode(hSpinMin, 1);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_TIME, hSpinMin, MIN_SPIN_POS_X, MIN_SPIN_POS_Y);

    hEdit = SPINBOX_GetEditHandle(hSpinMin);
    WM_SetCallback(hEdit, svSettings_CallbackEditSpinBox);

    hTextMin = TEXT_CreateEx(MIN_TEXT_POS_X, MIN_TEXT_POS_Y, 5, MIN_SPIN_HEIGHT, hParent, WM_CF_SHOW, 0, MIN_TEXT_ID,
            ":");
    TEXT_SetFont(hTextMin, &GUI_Font24B_1);
    TEXT_SetTextColor(hTextMin, GUI_WHITE);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_TIME, hTextMin, MIN_TEXT_POS_X, HOUR_SPIN_POS_Y);

    /**************** Creating SEC Spin **************************************/
    hSpinSec = SPINBOX_CreateEx(SEC_SPIN_POS_X, SEC_SPIN_POS_Y, SEC_SPIN_WIDTH, SEC_SPIN_HEIGHT, hParent,
    WM_CF_SHOW, SEC_SPIN_ID, 0, SEC_SPIN_MAX_VALUE);
    SPINBOX_SetEdge(hSpinSec, SPINBOX_EDGE_CENTER);
    SPINBOX_SetButtonSize(hSpinSec, SEC_SPIN_BUTTON_SIZE);
    SPINBOX_SetFont(hSpinSec, &GUI_Font24B_1);
    SPINBOX_SetValue(hSpinSec, min);
    SPINBOX_SetEditMode(hSpinSec, 1);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_TIME, hSpinSec, SEC_SPIN_POS_X, SEC_SPIN_POS_Y);

    hEdit = SPINBOX_GetEditHandle(hSpinSec);
    WM_SetCallback(hEdit, svSettings_CallbackEditSpinBox);

    /**************** Creating Button OK **************************************/
    hButtonSaveTime = BUTTON_CreateEx(BUTTON_SAVETIME_POS_X, BUTTON_SAVETIME_POS_Y, BUTTON_SAVETIME_WIDTH,
    BUTTON_SAVETIME_HEIGHT, hParent, WM_CF_SHOW, 0, BUTTON_SAVE_TIME_ID);
    BUTTON_SetText(hButtonSaveTime, xMessage_getMessageSettings()->buttonSaveTime);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_TIME, hButtonSaveTime, BUTTON_SAVETIME_POS_X,
    BUTTON_SAVETIME_POS_Y);

    SWIPELIST_SetBitmap(sSettingsPack.hSwipe, _SETTINGS_TIME, SWIPELIST_BA_LEFT | SWIPELIST_BA_VCENTER, &bmClockIcon);

    /**************** Creating Language List **************************************/
    hDropLangList = DROPDOWN_CreateEx(LANG_LIST_POS_X, LANG_LIST_POS_Y, LANG_LIST_WIDTH, LANG_LIST_HEIGHT, hParent,
    WM_CF_SHOW, 0, LANGUAGE_LIST_ID);
    DROPDOWN_SetFont(hDropLangList, &GUI_Font16B_1);
    DROPDOWN_SetUpMode(hDropLangList, 1);
    DROPDOWN_AddString(hDropLangList, apTextLangList[_E_ENGLISH]);
    DROPDOWN_AddString(hDropLangList, apTextLangList[_E_PORTUGUESE]);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_LANGUAGE, hDropLangList, LANG_LIST_POS_X,
    LANG_LIST_POS_Y);

    /**************** Creating Button Save and Reset **************************************/
    hButtonSaveLang = BUTTON_CreateEx(BUTTON_SAVELANG_POS_X, BUTTON_SAVELANG_POS_Y, BUTTON_SAVELANG_WIDTH,
    BUTTON_SAVELANG_HEIGHT, hParent, WM_CF_SHOW, 0, BUTTON_SAVE_LANG_ID);
    BUTTON_SetText(hButtonSaveLang, xMessage_getMessageSettings()->buttonSaveLang);
    SWIPELIST_ItemAttachWindow(sSettingsPack.hSwipe, _SETTINGS_LANGUAGE, hButtonSaveLang, BUTTON_SAVELANG_POS_X,
    BUTTON_SAVELANG_POS_Y);

    SWIPELIST_SetBitmap(sSettingsPack.hSwipe, _SETTINGS_LANGUAGE, SWIPELIST_BA_LEFT | SWIPELIST_BA_VCENTER,
            &bmLanguageIcon);

    /**************** Creating Calendar **************************************/
    CALENDAR_SetDefaultSize(CALENDAR_SI_HEADER, CALENDAR_HEADER_SIZE);
    CALENDAR_SetDefaultSize(CALENDAR_SI_CELL_X, CALENDAR_CELL_X);
    CALENDAR_SetDefaultSize(CALENDAR_SI_CELL_Y, CALENDAR_CELL_Y);
    CALENDAR_SetDefaultFont(CALENDAR_FI_CONTENT, &GUI_Font13B_1);
    CALENDAR_SetDefaultFont(CALENDAR_FI_HEADER, &GUI_Font16B_1);
    sSettingsPack.hCalendar = CALENDAR_Create(sSettingsPack.hSwipe, CALENDAR_POS_X, CALENDAR_POS_Y,
            sSettingsPack.pDate.Year, sSettingsPack.pDate.Month, sSettingsPack.pDate.Day, 0, CALENDAR_ID, 0);
    CALENDAR_SetDate(sSettingsPack.hCalendar, &sSettingsPack.pDate);

    SWIPELIST_SetBitmap(sSettingsPack.hSwipe, _SETTINGS_DATE, SWIPELIST_BA_LEFT | SWIPELIST_BA_VCENTER,
            &bmCalendarIcon);

    /**************** Creating Button OK **************************************/
    sSettingsPack.hButtonOK = BUTTON_CreateEx(BUTTON_OK_POS_X, BUTTON_OK_POS_Y, BUTTON_OK_WIDTH, BUTTON_OK_HEIGHT,
            sSettingsPack.hSwipe, WM_CF_HIDE | WM_CF_MEMDEV, 0, BUTTON_OK_ID);
    BUTTON_SetText(sSettingsPack.hButtonOK, xMessage_getMessageSettings()->buttonOk);

    /**************** Creating Button CANCEL **************************************/
    sSettingsPack.hButtonCancel = BUTTON_CreateEx(BUTTON_CANCEL_POS_X, BUTTON_CANCEL_POS_Y, BUTTON_CANCEL_WIDTH,
    BUTTON_CANCEL_HEIGHT, sSettingsPack.hSwipe,
    WM_CF_HIDE | WM_CF_MEMDEV, 0, BUTTON_CANCEL_ID);
    BUTTON_SetText(sSettingsPack.hButtonCancel, xMessage_getMessageSettings()->buttonCancel);

    /**************** Finishing **************************************/
    vSettings_HideSettings();
}

/*************************************************************************************************/
WM_HWIN hSettings_Win(void)
{
    return sSettingsPack.hWinSettings;
}

/*************************************************************************************************/
bool bSettings_isInTouchCalibration(void)
{
    if (sSettingsPack.TouchCalData.stateCal != _WAIT_NONE)
    {
        return true;
    }
    return false;
}

/*************************************************************************************************/
static void _vDispStringCentered(const char * pString)
{
    GUI_RECT Rect;
    Rect.x0 = Rect.y0 = STATUS_BAR_HEIGHT;
    Rect.x1 = LCD_GetXSize() - 1;
    Rect.y1 = LCD_GetYSize() - 1;
    GUI_DispStringInRect(pString, &Rect, GUI_TA_HCENTER | GUI_TA_VCENTER);
}

/*************************************************************************************************/
static void _vSettings_MessageWaitCircle(int i)
{
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    GUI_SetFont(&GUI_Font16B_1);
    GUI_SetColor(GUI_BLACK);
    _vDispStringCentered(xMessage_getMessageSettings()->touchRing); /* Ask user to press the touch */
    /* Draw the ring */
    GUI_FillCircle(sSettingsPack.TouchCalData.aLogX[i], sSettingsPack.TouchCalData.aLogY[i], 10);
    GUI_SetColor(GUI_WHITE);
    GUI_FillCircle(sSettingsPack.TouchCalData.aLogX[i], sSettingsPack.TouchCalData.aLogY[i], 5);
    GUI_SetColor(GUI_BLACK);
}
/*************************************************************************************************/
static void _vSettings_ShowBatteryStatus(void)
{
    static BatteryChargerInfo_t batInfo;
    char bufferString[50] = { 0 };

    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    GUI_SetFont(&GUI_Font16B_1);
    GUI_SetColor(GUI_BLACK);
    if(sSettingsPack.stateBatteryInfo == _BATINFO_SHOW)
    {
        vBatteryCharger_UpdateInfo(&batInfo);
    }
    GUI_DispStringHCenterAt(xMessage_getMessageSettings()->batteryStatusTitle, LCD_GetXSize() / 2,
                            STATUS_BAR_HEIGHT + 5);
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusVoltage, batInfo.voltage);
    GUI_DispStringAt(bufferString,0,STATUS_BAR_HEIGHT + 2);
    memset(bufferString, 0, sizeof(bufferString));
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusCurrent, batInfo.current);
    GUI_DispString(bufferString);
    memset(bufferString,0,sizeof(bufferString));
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusPower, batInfo.power);
    GUI_DispString(bufferString);
    memset(bufferString,0,sizeof(bufferString));
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusCapacity, batInfo.designCapacity);
    GUI_DispString(bufferString);
    memset(bufferString,0,sizeof(bufferString));
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusTemperature, batInfo.temperature);
    GUI_DispString(bufferString);
    memset(bufferString,0,sizeof(bufferString));
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusStateCharge, batInfo.stateOfCharge);
    GUI_DispString(bufferString);
    memset(bufferString,0,sizeof(bufferString));
    sprintf(bufferString, xMessage_getMessageSettings()->batteryStatusStateHealth, batInfo.health);
    GUI_DispString(bufferString);
}
