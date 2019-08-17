/**
 ******************************************************************************
 * @file    StatusBar.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 September 2017
 * @brief   Module to Manage the status bar
 ******************************************************************************
 */
#ifndef USER_STATUSBAR_H_
#define USER_STATUSBAR_H_

#include <stdint.h>
#include <stdbool.h>
#include "LCDConf.h"
#include "GUI.h"
#include "WM.h"


/**
 * \defgroup Status Bar Dimensions
 * @{
 */
#define STATUS_BAR_POS_X  0
#define STATUS_BAR_POS_Y  0
#define STATUS_BAR_HEIGHT 20
#define STATUS_BAR_WIDTH XSIZE_PHYS

#define BATTERY_WIDTH 30
#define BATTERY_HEIGHT 15
#define BATTERY_POS_X (STATUS_BAR_WIDTH - BATTERY_WIDTH - 8)
#define BATTERY_POS_Y 2

#define BATTERY_TOP_WIDTH 2
#define BATTERY_TOP_HEIGHT (BATTERY_HEIGHT - 4 -4)
#define BATTERY_TOP_POS_X (BATTERY_POS_X + BATTERY_WIDTH)
#define BATTERY_TOP_POS_Y (BATTERY_POS_Y+4)
#define BATTERY_TOP_POS_X2 (BATTERY_TOP_POS_X + BATTERY_TOP_WIDTH)
#define BATTERY_TOP_POS_Y2 (BATTERY_TOP_POS_Y + BATTERY_TOP_HEIGHT)

#define HOUR_WIDTH 80
#define HOUR_HEIGHT 30
#define HOUR_POS_X 2
#define HOUR_POS_Y 0

#define DATE_WIDTH 80
#define DATE_HEIGHT 30
#define DATE_POS_X (HOUR_WIDTH + 20)
#define DATE_POS_Y 0
/**@}*/

/*************************************************************************************************/
/**
 * @struct Date Time Struct
 */
typedef struct _DateTime_t
{
	uint8_t sec; /** @< Seconds */
	uint8_t min; /** @< Minutes */
	uint8_t hour; /** @< Hour */
	uint8_t day; /** @< Day */
	uint8_t month; /** @< Month */
	uint16_t year; /** @< Year */
} DateTime_t;

/*************************************************************************************************/
/**
 * @brief Initializes the status bar and link it to the screen
 * @param hWinParent emWin parent Window
 * @return void.
 */
void vStatusBar_AddStatusBar(WM_HWIN hWinParent);

/*************************************************************************************************/
/**
 * @brief Sets the battery Icon percentage
 * @param value battery Icon Percentage (0-100)
 * @return void.
 */
void vStatusBar_setBatteryPercentage(uint8_t value);

/*************************************************************************************************/
/**
 * @brief Show the charging Icon
 * @param isCharging true to show, false to hide
 * @return void.
 */
void vStatusBar_setCharging(bool isCharging);

/*************************************************************************************************/
/**
 * @brief Show the battery icon
 * @param isBatteryDetected true to show, false to hide
 * @return void.
 */
void vStatusBar_setBatteryDetected(bool isBatteryDetected);

/*************************************************************************************************/
/**
 * @brief Show the logging Icon
 * @param isLogging true to show, false to hide
 * @return void.
 */
void vStatusBar_setLogging(bool isLogging);
/*************************************************************************************************/
/**
 * @brief Set the Clock time values
 * @param sec Seconds
 * @param min Minutes
 * @param hour Hour (24h format)
 * @return void
 */
void vStatusBar_SetTime(uint8_t sec, uint8_t min, uint8_t hour);

/*************************************************************************************************/
/**
 * @brief Set the Clock date values
 * @param day day of the month
 * @param month month the year
 * @param year 2XXX year
 * @return void
 */
void vStatusBar_SetDate(uint8_t day, uint8_t month, uint16_t year);

/*************************************************************************************************/
/**
 * @brief Retrives the status bar window handler
 * @return enWin Window handler
 */
WM_HWIN hStatusBar_Win(void);
#endif /* USER_STATUSBAR_H_ */
