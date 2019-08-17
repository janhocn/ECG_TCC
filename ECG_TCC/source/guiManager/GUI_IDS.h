/**
 ******************************************************************************
 * @file    GUI_IDS.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    2 December 2017
 * @brief   Manage all emWin GUI IDs
 ******************************************************************************
 */

#ifndef USER_RESOURCES_GUI_IDS_H_
#define USER_RESOURCES_GUI_IDS_H_

#include "GUI.h"

/*************************************************************************************************/
/**
 * @enum Parameters to ithe GUI ID of each element in the project
 */
typedef enum
{
    WINDOW_SETTINGS_ID = GUI_ID_USER + 0x01, /** @< Setting Window ID */
    SWIPE_ID, /** @< Swipe Setting Window ID */
    BRIGHTNESS_SLIDER_ID, /** @< Brightness Slider Setting Window ID */
    HOUR_SPIN_ID, /** @< Hour Spin Setting Window ID */
    MIN_SPIN_ID, /** @< Minute Spin Setting Window ID */
    SEC_SPIN_ID, /** @< Second Spin Setting Window ID */
    HOUR_TEXT_ID, /** @< Hour Text Setting Window ID */
    MIN_TEXT_ID, /** @< Minute Text Setting Window ID */
    SEC_TEXT_ID, /** @< Second Text Setting Window ID */
    CALENDAR_ID, /** @< Calendar Setting Window ID */
    BUTTON_OK_ID, /** @< Calendar Button OK Setting Window ID */
    BUTTON_CANCEL_ID, /** @< Calendar Button Cancel Setting Window ID */
    BUTTON_SAVE_TIME_ID, /** @< Save Time Button Setting Window ID */
    BUTTON_SAVE_LANG_ID, /** @< Save Language Button Setting Window ID */
    LANGUAGE_LIST_ID, /** @< Language List Setting Window ID */
    WINDOW_TOUCHCAL_ID, /** @< Touch Calibration Window Setting Window ID */
    ECG_GRAPH_ID, /** @< Ecg Graph ID */
    ECG_BPM_ID, /** @< Ecg BPM Text ID */
    ECG_RR_ID, /** @< Ecg RR Text ID */
    ECG_HR_ID, /** @< Ecg HR Unit Text ID */
    ECG_HR_VALUE_ID, /** @< Ecg HR Text ID */
    ECG_HEART_ID, /** @< Ecg Heart Icon ID */
    ECG_LOGICON_ID, /** @< Ecg Log Icon ID */
    ECG_LOG_CHECKBOX_ID, /** @< Ecg start log checkbox ID */
    ECG_LOG_TEXT_ID, /** @< Ecg start log text ID */
    SPO2_SPO2_VALUE_ID, /** @< SPO2 Value Text ID */
    SPO2_SPO2_ID, /** @< SPO2 %SPO2 Text ID */
    SPO2_ICON_SPO2_ID, /** @< SPO2 Icon ID */
    SPO2_HR_ID, /** @< SPO2 BPM Text ID */
    SPO2_HR_VALUE_ID, /** @< SPO2 HR Value ID */
    SPO2_GRAPH_ID, /** @< SpO2 Graph ID */
    SPO2_LOGICON_ID, /** @< SPO2 Log Icon ID */
    SPO2_LOG_CHECKBOX_ID, /** @< SPO2 start log checkbox ID */
    SPO2_LOG_TEXT_ID, /** @< SPO2 start log text ID */
    HOME_ICONVIEW_ID, /** @< Home Iconview ID */
    HOME_BUTTON_ID, /** @< Home Button ID */
    STATUS_BAR_HOUR_ID, /** @< Status Bar Time Text ID */
    STATUS_BAR_DATE_ID, /** @< Status Bar Date Text ID */
    STATUS_BAR_BAT_ID, /** @< Status Bar Battery Progress Bar ID */
    STATUS_BAR_BAT_CHARGING_ICON, /** @< Status Bar Battery Charging Icon ID */
    STATUS_BAR_BAT_LOGGING_ICON, /** @< Status Bar Logging Icon ID */
    MACKENZIE_ICON_ID /** @< Mackenzie icon on Home window */
} eUserIDs;

/*************************************************************************************************/
#endif /* USER_RESOURCES_GUI_IDS_H_ */
