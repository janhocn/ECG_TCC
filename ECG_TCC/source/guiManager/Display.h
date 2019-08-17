/**
 ******************************************************************************
 * @file    Display.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 July 2017
 * @brief   Module to Manage the display states
 ******************************************************************************
 */

#ifndef USER_DISPLAY_H_
#define USER_DISPLAY_H_

/*************************************************************************************************/
/**
 * @enum Parameters to inform the state of the windows
 */
typedef enum
{
    _WINDOW_HOME = -1, /** @< Home window State */
    _WINDOW_ECGGraph, /** @< ECG Graph window State */
    _WINDOW_SPO2Graph, /** @< SPO2 Graph window State */
    _WINDOW_SETTINGS, /** @< Settings window State */
    _WINDOW_ABOUT, /** @< About window State */
    _WINDOW_HELP, /** @< Help window State */
    _WINDOW_DEBUG, /** @< Debug window State */
} stateDisplayTabs;

/*************************************************************************************************/
/**
 * @brief Initializes all display routines including Emwin API
 * @return void.
 */
void vDisplay_GUI_Init(void);

/*************************************************************************************************/
/**
 * @brief Initializes and creates all application widows
 * @return void.
 */
void vDisplay_WindowsInit(void);

/*************************************************************************************************/
/**
 * @brief Refresh the GUI.
 * @return void.
 */
void vDisplay_RefreshDisplay(void);

/*************************************************************************************************/
/**
 * @brief Retrieves the actual window state
 * @return window state (see stateDisplayTabs).
 */
stateDisplayTabs vDisplay_GetState(void);

/*************************************************************************************************/
/**
 * @brief Sets the actual window state
 * @param st Window State (see stateDisplayTabs).
 * @return void
 */
void vDisplay_SetWindow(stateDisplayTabs st);

/*************************************************************************************************/
/**
 * @brief Shows a message box
 * @param msg Message to show
 * @param title title string
 * @return void
 */
void vDisplay_ShowMessageBox(char * msg,char * title);

/*************************************************************************************************/
/**
 * @brief Draws the main background image.
 * @return void
 */
void vDisplay_DrawBackground(void);
/*************************************************************************************************/
#endif /* USER_DISPLAY_H_ */
