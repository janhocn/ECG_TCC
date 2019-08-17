/**
 ******************************************************************************
 * @file    RTC.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 September 2017
 * @brief   Internal RTC Module
 ******************************************************************************
 */


#ifndef USER_RTC_H_
#define USER_RTC_H_

#include "stdint.h"
#include "stdbool.h"

/*************************************************************************************************/
/**
 * @brief Initializes the RTC
 * @return void
 */
void vRTC_Init(void) ;

/*************************************************************************************************/
/**
 * @brief Returns the RTC values
 * @param sec Pointer to Seconds
 * @param min Pointer to Minutes
 * @param hour Pointer to Hour (24h format)
 * @param day Pointer to day of the month
 * @param month Pointer to month the year
 * @param year Pointer to 2XXX year
 * @return void
 */
void vRTC_GetDateTime(uint8_t *sec, uint8_t *min, uint8_t *hour, uint8_t *day, uint8_t *month, uint16_t *year);

/*************************************************************************************************/
/**
 * @brief Set the RTC date values
 * @param day day of the month
 * @param month month the year
 * @param year 2XXX year
 * @return void
 */
void vRTC_SetDate(uint8_t day, uint8_t month, uint16_t year);

/*************************************************************************************************/
/**
 * @brief Set the RTC time values
 * @param sec Seconds
 * @param min Minutes
 * @param hour Hour (24h format)
 * @return void
 */
void vRTC_SetTime(uint8_t sec, uint8_t min, uint8_t hour);

/*************************************************************************************************/
/**
 * @brief ISR for Seconds interrupt Handler
 * @attention This function notifies the SO to perform a second change in the GUI.
 */
void vRTC_IRQ_RTC(void);
#endif /* USER_RTC_H_ */
