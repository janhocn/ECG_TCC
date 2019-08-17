/**
 ******************************************************************************
 * @file    GraphSPO2.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    19 February 2018
 * @brief   Module to use the SPO2 graph window.
 ******************************************************************************
 */

#ifndef USER_GRAPHSPO2_H_
#define USER_GRAPHSPO2_H_

#include <stdint.h>
#include "WM.h"
#include "LCDConf.h"
#include "GUI.h"
#include "IMAGE.h"
#include "TEXT.h"
#include "CHECKBOX.h"
#include "GraphWidget.h"
/*************************************************************************************************/
/**
 * @struct Structure containing all elements in this module, including emWin Handlers and the
 *         sGraph_t structure.
 */
typedef struct _sGraphSPO2Pack_t
{
    WM_HWIN hWindowGraphSPO2; /** @< emWin Handler to the window */
    TEXT_Handle hTextSPO2Text; /** @< emWin Handler to the string "% SPO2" */
    TEXT_Handle hTextSPO2Value; /** @< emWin Handler to the %SPO2 value */
    TEXT_Handle hTextHRText; /** @< emWin Handler to the string "HR" */
    TEXT_Handle hTextHRValue; /** @< emWin Handler to the HR value */
    TEXT_Handle hLogText; /** @< emWin Handler to the string "LOG" */
    CHECKBOX_Handle hLOGCheckbox; /** @< emWin Handler to the start log check box*/
    IMAGE_Handle hIconLog; /** @< emWin Handler to the log image */
    IMAGE_Handle hIconSPO2; /** @< emWin Handler to the SPO2 Icon */
    sGraph_t GraphSPO2Widget; /** @< structure to the graph widget (user) */
} sGraphSPO2Pack_t;

/*************************************************************************************************/
/**
 * @enum Enumeration to distinguish data curves.
 */
typedef enum _eGraphSPO2Data {
    _E_DATA_SPO2_RED_RAW = 0,  /** @< Index of the Raw RED LED data */
    _E_DATA_SPO2_IR_RAW,  /** @< Index of the Raw IR LED data */
    _E_DATA_SPO2_FILTERED,  /** @< Index of the filtered ecg data */
    _E_DATA_SPO2_NUMBER /** @< Max number of data curves */
}eGraphSPO2Data;

/*************************************************************************************************/
/**
 * @brief Creates the window
 * @return void.
 */
void vGraphSPO2_AddGraphSPO2Screen(void);

/*************************************************************************************************/
/**
 * @brief Adds a value to a data curve.
 * @param value Value to be plotted.
 * @param Data Number of the data curve.
 * @attention to properly exhibit the data the function vGraphSPO2_PostData need to be called after
 *            adding the desired data to all data curves
 * @return void.
 */
void vGraphSPO2_AddDataMainGraph(uint16_t value, eGraphSPO2Data Data);

/*************************************************************************************************/
/**
 * @brief Clear the data curves (all) and return the sweep bar to the beginning
 * @return void.
 */
void vGraphSPO2_ClearMainGraph(void);

/*************************************************************************************************/
/**
 * @brief Plot the data curves into the graph.
 * @attention This function should be called after vGraph_AddData
 * @return void.
 */
void vGraphSPO2_PostData(void);

/*************************************************************************************************/
/**
 * @brief Uncheck Log checkbox.
 * @return void.
 */
void GraphSPO2_UncheckLog(void);


/*************************************************************************************************/
/**
 * @brief Set the SPO2 value on the screen.
 * @param value SPO2 value
 * @return void.
 */
void vGraphSPO2_setSPO2Value(uint8_t value);

/*************************************************************************************************/
/**
 * @brief Set the heart rate value on the screen.
 * @param value Heart Rate value
 * @return void.
 */
void vGraphSPO2_setHeartRateValue(uint16_t value);


/*************************************************************************************************/
/**
 * @brief Retrieves the window handler
 * @return emWin Handler (type WM_HWIN) to the window.
 */
WM_HWIN hGraphSPO2_getWindow(void);
/*************************************************************************************************/
#endif /* USER_GRAPHSPO2_H_ */
