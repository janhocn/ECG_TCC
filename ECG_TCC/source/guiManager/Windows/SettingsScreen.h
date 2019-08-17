/**
 ******************************************************************************
 * @file    SettingsScreen.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 July 2017
 * @brief   Module to manage Settings Window
 ******************************************************************************
 */


#ifndef USER_SETTINGSSCREEN_H_
#define USER_SETTINGSSCREEN_H_

#include "StatusBar.h"
#include "TouchScreen.h"

/**
 * \defgroup Settings Window Items Dimensions
 * @{
 */
#define NUM_CAL_POINTS 5

#define TOUCHCAL_POINT1_POS_X   14
#define TOUCHCAL_POINT1_POS_Y   (STATUS_BAR_HEIGHT + TOUCHCAL_POINT1_POS_X)

#define TOUCHCAL_POINT2_POS_X   XSIZE_PHYS - 1 - TOUCHCAL_POINT1_POS_X
#define TOUCHCAL_POINT2_POS_Y   YSIZE_PHYS - 1 - TOUCHCAL_POINT1_POS_Y

#define TOUCHCAL_POINT3_POS_X   XSIZE_PHYS/2
#define TOUCHCAL_POINT3_POS_Y   YSIZE_PHYS/2

#define TOUCHCAL_POINT4_POS_X   XSIZE_PHYS - 1 - TOUCHCAL_POINT1_POS_X
#define TOUCHCAL_POINT4_POS_Y   (STATUS_BAR_HEIGHT + TOUCHCAL_POINT1_POS_X)

#define TOUCHCAL_POINT5_POS_X   TOUCHCAL_POINT1_POS_X
#define TOUCHCAL_POINT5_POS_Y   YSIZE_PHYS - 1 - TOUCHCAL_POINT1_POS_Y

#define SWIPE_SETTINGS_POS_X	0
#define SWIPE_SETTINGS_POS_Y	STATUS_BAR_HEIGHT + 0
#define SWIPE_ITEM_SEP_SIZE 40
#define SWIPE_ITEM_SIZE 45
#define SWIPE_ITEM_SIZE_TIME (SWIPE_ITEM_SIZE+30)
#define SWIPE_ITEM_SIZE_LANG (SWIPE_ITEM_SIZE+10)

#define BRIGHTNESS_SLIDER_POS_X	250
#define BRIGHTNESS_SLIDER_POS_Y 5
#define BRIGHTNESS_SLIDER_WIDTH 200
#define BRIGHTNESS_SLIDER_HEIGHT (SWIPE_ITEM_SIZE-10)
#define BRIGHTNESS_MAX_VALUE 65
#define BRIGHTNESS_MIN_VALUE 23

#define HOUR_SPIN_POS_X	150
#define HOUR_SPIN_POS_Y	4
#define HOUR_SPIN_WIDTH	80
#define HOUR_SPIN_HEIGHT (SWIPE_ITEM_SIZE-10)
#define HOUR_SPIN_BUTTON_SIZE 20
#define HOUR_SPIN_MAX_VALUE 23

#define MIN_SPIN_POS_X	HOUR_SPIN_POS_X+HOUR_SPIN_WIDTH+35
#define MIN_SPIN_POS_Y	HOUR_SPIN_POS_Y
#define MIN_SPIN_WIDTH	HOUR_SPIN_WIDTH
#define MIN_SPIN_HEIGHT HOUR_SPIN_HEIGHT
#define MIN_SPIN_BUTTON_SIZE HOUR_SPIN_BUTTON_SIZE
#define MIN_SPIN_MAX_VALUE 59

#define SEC_SPIN_POS_X	MIN_SPIN_POS_X+MIN_SPIN_WIDTH+35
#define SEC_SPIN_POS_Y	HOUR_SPIN_POS_Y
#define SEC_SPIN_WIDTH	HOUR_SPIN_WIDTH
#define SEC_SPIN_HEIGHT HOUR_SPIN_HEIGHT
#define SEC_SPIN_BUTTON_SIZE HOUR_SPIN_BUTTON_SIZE
#define SEC_SPIN_MAX_VALUE 59

#define BUTTON_SAVETIME_POS_X (HOUR_SPIN_POS_X + 4)
#define BUTTON_SAVETIME_POS_Y (MIN_SPIN_POS_Y + HOUR_SPIN_HEIGHT + 4)
#define BUTTON_SAVETIME_WIDTH (HOUR_SPIN_WIDTH+MIN_SPIN_WIDTH+SEC_SPIN_WIDTH+64)
#define BUTTON_SAVETIME_HEIGHT (SWIPE_ITEM_SIZE_TIME-HOUR_SPIN_HEIGHT-HOUR_SPIN_POS_Y-4-4)

#define HOUR_TEXT_POS_X	HOUR_SPIN_POS_X + HOUR_SPIN_WIDTH+16
#define HOUR_TEXT_POS_Y HOUR_SPIN_POS_Y + 4
#define MIN_TEXT_POS_X	MIN_SPIN_POS_X + MIN_SPIN_WIDTH+16
#define MIN_TEXT_POS_Y	MIN_SPIN_POS_Y + 4

#define CALENDAR_HEADER_SIZE 28
#define CALENDAR_CELL_X 50
#define CALENDAR_CELL_Y 40
#define CALENDAR_POS_X 10
#define CALENDAR_POS_Y 10
#define CALENDAR_WIDTH 7*CALENDAR_CELL_X
#define CALENDAR_HEIGHT 7*CALENDAR_CELL_Y

#define BUTTON_OK_POS_X (CALENDAR_WIDTH + 20)
#define BUTTON_OK_POS_Y	50
#define BUTTON_OK_WIDTH 100
#define BUTTON_OK_HEIGHT 50

#define BUTTON_CANCEL_POS_X BUTTON_OK_POS_X
#define BUTTON_CANCEL_POS_Y	(BUTTON_OK_POS_Y+BUTTON_OK_HEIGHT+10)
#define BUTTON_CANCEL_WIDTH BUTTON_OK_WIDTH
#define BUTTON_CANCEL_HEIGHT BUTTON_OK_HEIGHT

#define LANG_LIST_POS_X 150
#define LANG_LIST_POS_Y 4
#define LANG_LIST_WIDTH 200
#define LANG_LIST_HEIGHT 40

#define BUTTON_SAVELANG_POS_X (LANG_LIST_POS_X)
#define BUTTON_SAVELANG_POS_Y 28
#define BUTTON_SAVELANG_WIDTH (LANG_LIST_WIDTH)
#define BUTTON_SAVELANG_HEIGHT 24
/**@}*/

/*************************************************************************************************/
/**
 * @enum Settings Items
 */
typedef enum
{
    _SETTINGS_GENERAL = 0, /** @< General Items Separator */
    _SETTINGS_BRIGHTNESS, /** @< Brightness configuration item */
    _SETTINGS_TIME, /** @< Time configuration item */
    _SETTINGS_DATE, /** @< Data configuration item */
    _SETTINGS_BATTERY, /** @< Battery Status item */
    _SETTINGS_INPUT, /** @< Input Items Separator */
    _SETTINGS_TOUCHCAL, /** @< Touch Calibration configuration item */
    _SETTINGS_LANGUAGE /** @< Language Calibration configuration item */
} eSettingsMenu;

/*************************************************************************************************/
/**
 * @brief creates setting window and add it to the gui
 * @return void.
 */
void vSettings_AddSettingsScreen(void);

/*************************************************************************************************/
/**
 * @brief show all items and windows of the settings window
 * @return void.
 */
void vSettings_ShowSettings(void);

/*************************************************************************************************/
/**
 * @brief hide all items and windows of the settings window
 * @return void.
 */
void vSettings_HideSettings(void);

/*************************************************************************************************/
/**
 * @brief Check if the settings is in touch calibration state
 * @return true if in touch calibrartion state.
 */
bool bSettings_isInTouchCalibration(void);

/*************************************************************************************************/
/**
 * @brief Retrieves the Settings Window emWin handler.
 * @return emWin Window handler.
 */
WM_HWIN hSettings_Win(void);
#endif /* USER_SETTINGSSCREEN_H_ */
