/**
 ******************************************************************************
 * @file    Home.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    3 July 2017
 * @brief   Module to manage Home Window
 ******************************************************************************
 */
#ifndef USER_HOME_H_
#define USER_HOME_H_

#include <stddef.h>
#include "stdint.h"
#include "ICONVIEW.h"
#include "TEXT.h"

/*************************************************************************************************/
/**
 * @brief Creates the Home Window.
 * @return void.
 */
void vHOME_AddHomeScreen(void);

/*************************************************************************************************/
/**
 * @brief Show the home button.
 * @return void.
 */
void vHOME_ShowButton(void);

/*************************************************************************************************/
/**
 * @brief Hide the home button.
 * @return void.
 */
void vHOME_HideButton(void);

/*************************************************************************************************/
/**
 * @brief Retrieves the Home window emWin handler.
 * @return emWin Window handler.
 */
WM_HWIN hHOME_Win(void);

/*************************************************************************************************/
/**
 * @brief Retrieves the Home Button widget emWin handler.
 * @return emWin Window handler.
 */
WM_HWIN hHOME_ButtonWin(void);
#endif /* USER_HOME_H_ */
