/**
 ******************************************************************************
 * @file    GraphECG.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    19 February 2018
 * @brief   Module to use the ECG graph window.
 ******************************************************************************
 */

#ifndef USER_GRAPHECG_H_
#define USER_GRAPHECG_H_

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
typedef struct _sGraphECGPack_t
{
    WM_HWIN hWindowGraphECG; /** @< emWin Handler to the window */
    TEXT_Handle hTextRRInterval; /** @< emWin Handler to the string "RR:" */
    TEXT_Handle hTextHeartRateText; /** @< emWin Handler to the string "HR" */
    TEXT_Handle hTextHeartRateValue; /** @< emWin Handler to the heart rate value */
    TEXT_Handle hTextBPMUnit; /** @< emWin Handler to the string "BPM" */
    TEXT_Handle hLogText; /** @< emWin Handler to the string "LOG" */
    CHECKBOX_Handle hLOGCheckbox; /** @< emWin Handler to the start log check box*/
    IMAGE_Handle hIconLog; /** @< emWin Handler to the log image */
    IMAGE_Handle hIconHeart; /** @< emWin Handler to the heart image */
    sGraph_t GraphECGWidget; /** @< structure to the graph widget (user) */
    bool toggleTextHR; /** @< var to control the color toggling of the HR text */
} sGraphECGPack_t;

/*************************************************************************************************/
/**
 * @enum Enumeration to distinguish data curves.
 */
typedef enum _eGraphECGData {
    _E_DATA_ECG_RAW = 0,  /** @< Index of the Raw ecg data */
    _E_DATA_ECG_FILTERED,  /** @< Index of the filtered ecg data */
    _E_DATA_NUMBER /** @< Max number of data curves */
}eGraphECGData;

/*************************************************************************************************/
/**
 * @brief Creates the window
 * @return void.
 */
void vGraphECG_AddGraphECGScreen(void);

/*************************************************************************************************/
/**
 * @brief Adds a value to a data curve.
 * @param value Value to be plotted.
 * @param Data Number of the data curve.
 * @attention to properly exhibit the data the function vGraphECG_PostData need to be called after
 *            adding the desired data to all data curves
 * @return void.
 */
void vGraphECG_AddDataMainGraph(uint16_t value, eGraphECGData Data);

/*************************************************************************************************/
/**
 * @brief Clear the data curves (all) and return the sweep bar to the beginning
 * @return void.
 */
void vGraphECG_ClearMainGraph(void);

/*************************************************************************************************/
/**
 * @brief Plot the data curves into the graph.
 * @attention This function should be called after vGraph_AddData
 * @return void.
 */
void vGraphECG_PostData(void);

/*************************************************************************************************/
/**
 * @brief Uncheck Log checkbox.
 * @return void.
 */
void GraphECG_UncheckLog(void);

/*************************************************************************************************/
/**
 * @brief Set the RR Interval value on the screen.
 * @param value RR Interval in ms
 * @return void.
 */
void vGraphECG_setRRInterval(uint16_t value);

/*************************************************************************************************/
/**
 * @brief Set the heart rate value on the screen.
 * @param value Heart Rate value
 * @return void.
 */
void vGraphECG_setHeartRateValue(uint16_t value);

/*************************************************************************************************/
/**
 * @brief Toggle the color of the HR text (red and white).
 * @return void.
 */
void vGraphECG_toggleHRTextColor(void);

/*************************************************************************************************/
/**
 * @brief Retrieves the window handler
 * @return emWin Handler (type WM_HWIN) to the window.
 */
WM_HWIN hGraphECG_getWindow(void);
/*************************************************************************************************/
#endif /* USER_GRAPHECG_H_ */
