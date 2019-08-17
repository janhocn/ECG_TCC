/**
 ******************************************************************************
 * @file    logTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    05 May 2018
 * @brief   Module to manage logging routines
 ******************************************************************************
 */

#ifndef TASKS_LOGTASK_H_
#define TASKS_LOGTASK_H_

#include "arm_math.h"

/*************************************************************************************************/
/**
 * @def Maximum number of each log file
 */
#define MAX_LOG_NUMBER 50

/*************************************************************************************************/
/**
 * @def Maximum time Log
 */
#define LOG_TIME 35000

/*************************************************************************************************/
/**
 * @def Maximum number of samples for SPO2 Data (for IR and for Red LEDs)
 */
#define LOG_NUMSAMPLES_SPO2 50


/*************************************************************************************************/
/**
 * @def Maximum number of samples for ECG Data
 */
#define LOG_NUMSAMPLES_ECG 50

/*************************************************************************************************/
/**
 * @var data structure containing the necessary buffers for logging.
 */
typedef struct _logData
{
    struct
    {
        struct
        {
            char string[10]; /** 8 Bytes for 32bit hexa representation + ';' + \0 */
        } bufferRed[LOG_NUMSAMPLES_SPO2]; /** @< Buffer to hold data for Red LED from SPO2 */

        struct
        {
            char string[10]; /** 8 Bytes for 32bit hexa representation + ';' + \0 */
        } bufferIR[LOG_NUMSAMPLES_SPO2]; /** @< Buffer to hold data for IR LED from SPO2 */

    } buffferSPO2;

    struct
    {
        char string[5]; /** 4 Bytes for 16bit hexa representation + ';' + \0 */
    } bufferECG[LOG_NUMSAMPLES_ECG]; /** @< Buffer to hold data for ECG RAW Data */
} logData_t;

/*************************************************************************************************/
/**
 * @enum Data type to be logged
 */
typedef enum _logType
{
    _LOG_ECG_DATA, /** @< To log ecg raw data */
    _LOG_SPO2_DATA, /** @< To log spo2 (Red and IR LED) data */
    _LOG_STOP /** @< logging is stopped (waiting) */
} logType_t;

/*************************************************************************************************/
/**
 * @brief Initialzie the module and task
 * @return void.
 */
void vLogTaskInit(void);

/*************************************************************************************************/
/**
 * @brief Stop logging, any kind of data
 * @return void.
 */
void vLogTask_StopLog(void);

/*************************************************************************************************/
/**
 * @brief Start logging, specific data
 * @param type Type of data to be logged
 * @return void.
 * @attention If a log was already started, it is not possible to override it, meaning that only
 *            one log can be performed at time
 */
void vLogTask_StartLog(logType_t type);

/*************************************************************************************************/
/**
 * @brief Send the data to be logged
 * @param pData pointer to the data
 * @return void.
 * @attention The pData should contain the number of points corresponding the maximum defined here
 *            e.g A buffer for ecg raw data should contain only LOG_NUMSAMPLES_ECG float32_t samples
 */
void vLogTask_PostData(void* pData);

/*************************************************************************************************/
/**
 * @brief Returns the type of data being logged, if any
 * @return data type.
 */
logType_t xLogTask_isLogging(void);

#endif /* TASKS_LOGTASK_H_ */
