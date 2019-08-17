/**
 ******************************************************************************
 * @file    logTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    05 May 2018
 * @brief   Module to manage logging routines
 ******************************************************************************
 */
#include <logTask.h>
#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user.h"
#include "task_manager.h"
#include "FileSystem.h"
#include "arm_math.h"
#include "StatusBar.h"
#include "GraphECG.h"
#include "GraphSPO2.h"
#include "MESSAGEBOX.h"
#include "Messages.h"
#include "Display.h"

static volatile logType_t logging = _LOG_STOP;
static TaskMngTaskHandle_t xLogTaskHandle = NULL;
static TaskMngStreamBufferHandle_t xLogTaskBufferECG = NULL;
static TaskMngStreamBufferHandle_t xLogTaskBufferSPO2 = NULL;
static volatile FIL fileDesciptorECG;
static volatile FIL fileDesciptorRed;
static volatile FIL fileDesciptorIR;
static volatile logData_t data;
static bool fdClosed = true;
static uint32_t u32LogStartTime = 0;
/*************************************************************************************************/
/**
 * @brief Task to log files
 * @param pvParameters Pointer to user parameters (refer to FreeRTOS API).
 * @return void.
 */
static void vLogTask(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief Creates files numbering it
 * @param fp pointer to file descriptor handler
 * @param path filename
 * @param pathLen filename length
 * @return FatFS result
 */
FRESULT _vLogTask_OpenFile(FIL* fp, TCHAR* path, DWORD pathLen);

/*************************************************************************************************/
static void vLogTask(void *pvParameters)
{
    uint32_t ui32BlockCounter = 0;
    uint32_t bytesWriten = 0;
    volatile FRESULT result;

    xLogTaskBufferECG = xTaskManager_StreamBufferCreate(1 * sizeof(data.bufferECG), 1 * sizeof(data.bufferECG));
    configASSERT(xLogTaskBufferECG != NULL);
    xLogTaskBufferSPO2 = xTaskManager_StreamBufferCreate(1 * sizeof(data.buffferSPO2), 1 * sizeof(data.buffferSPO2));
    configASSERT(xLogTaskBufferSPO2 != NULL);

    for (;;)
    {
        switch (logging)
        {
            case _LOG_ECG_DATA:
                ui32BlockCounter = xTaskManager_StreamBufferReceive(xLogTaskBufferECG, (void *) &data.bufferECG[0],
                        sizeof(data.bufferECG), 500);
                if (ui32BlockCounter == sizeof(data.bufferECG))
                {
                    if (!fdClosed)
                    {
                        result = stFileSystem_WriteFile((FIL *) &fileDesciptorECG, (const void *) &data.bufferECG[0],
                                sizeof(data.bufferECG), (UINT *) &bytesWriten);
                        if (result == FR_OK)
                        {
                            if (xTaskManager_getTickCount() - u32LogStartTime > LOG_TIME)
                            {
                                vLogTask_StopLog();
                            }
                        }
                    }
                }
                break;
            case _LOG_SPO2_DATA:
                ui32BlockCounter = xTaskManager_StreamBufferReceive(xLogTaskBufferSPO2, (void *) &data.buffferSPO2,
                        sizeof(data.buffferSPO2), 500);
                if (ui32BlockCounter == sizeof(data.buffferSPO2))
                {
                    if (!fdClosed)
                    {
                        result = stFileSystem_WriteFile((FIL *) &fileDesciptorRed,
                                (const void *) &data.buffferSPO2.bufferRed[0], sizeof(data.buffferSPO2.bufferRed),
                                (UINT *) &bytesWriten);

                        vTaskDelay(1);
                        if (result == FR_OK)
                        {
                            result = stFileSystem_WriteFile((FIL *) &fileDesciptorIR,
                                    (const void *) &data.buffferSPO2.bufferIR[0], sizeof(data.buffferSPO2.bufferIR),
                                    (UINT *) &bytesWriten);
                            if (result == FR_OK)
                            {
                                if (xTaskManager_getTickCount() - u32LogStartTime > LOG_TIME)
                                {
                                    vLogTask_StopLog();
                                }
                            }
                        }
                    }
                }
                break;
            case _LOG_STOP:
            default:
                ui32TaskManager_TaskNotifyTake(pdTRUE, portMAX_DELAY);
                u32LogStartTime = xTaskManager_getTickCount();
                break;
        }
    }
    vTaskManager_TaskDelete(NULL);
}

/*************************************************************************************************/
void vLogTask_PostData(void* pData)
{
    uint8_t numBytes = 0;
    uint32_t length = 0;

    switch (logging)
    {
        case _LOG_ECG_DATA:
            numBytes = 5;
            length = LOG_NUMSAMPLES_ECG * numBytes;
            if (xLogTaskBufferECG)
            {
                xTaskManager_StreamBufferSend(xLogTaskBufferECG, (const void *) pData, length, 200);
            }
            else
            {
                logging = _LOG_STOP;
            }
            break;
        case _LOG_SPO2_DATA:
            numBytes = 10;
            length = 2 * LOG_NUMSAMPLES_SPO2 * numBytes;     //2 times for Red and IR
            if (xLogTaskBufferSPO2)
            {
                xTaskManager_StreamBufferSend(xLogTaskBufferSPO2, (const void *) pData, length, 200);
            }
            else
            {
                logging = _LOG_STOP;
            }
            break;
        case _LOG_STOP:
        default:
            numBytes = 0;
            length = 0;
            return;
    }
}

/*************************************************************************************************/
logType_t xLogTask_isLogging(void)
{
    return logging;
}
/*************************************************************************************************/
FRESULT _vLogTask_OpenFile(FIL* fp, TCHAR* path, DWORD pathLen)
{
    FILINFO fno;
    uint16_t number = 0;
    char numberStr[2] = { 0 };
    while (f_stat(path, &fno) == FR_OK)
    {
        memcpy(numberStr, &path[pathLen - 6], 2);
        number = (numberStr[0] - 0x30) * 10 + (numberStr[1] - 0x30);
        number++;
        if (number > MAX_LOG_NUMBER)
        {
            break;
        }
        numberStr[0] = (number / 10) + 0x30;
        numberStr[1] = (number % 10) + 0x30;
        memcpy(&path[pathLen - 6], numberStr, 2);
    }
    return stFileSystem_OpenFile(fp, path, "w");
}

/*************************************************************************************************/
void vLogTask_StartLog(logType_t type)
{
    FRESULT result = FR_OK;
    TCHAR bufferName[32] = { 0 };
    if (logging == _LOG_STOP)
    {
        switch (type)
        {
            case _LOG_ECG_DATA:
                memcpy(bufferName, __LOG_ECG_RAW_FILE, sizeof(__LOG_ECG_RAW_FILE));
                result = _vLogTask_OpenFile((FIL*) &fileDesciptorECG, bufferName, strlen(__LOG_ECG_RAW_FILE));
                break;

            case _LOG_SPO2_DATA:
                memcpy(bufferName, __LOG_ECG_SPO2_RED_FILE, sizeof(__LOG_ECG_SPO2_RED_FILE));
                result = _vLogTask_OpenFile((FIL*) &fileDesciptorRed, bufferName, strlen(__LOG_ECG_SPO2_RED_FILE));
                vTaskManager_Delay(5);
                if (result == FR_OK)
                {
                    memset(bufferName, 0, sizeof(bufferName));
                    memcpy(bufferName, __LOG_ECG_SPO2_IR_FILE, sizeof(__LOG_ECG_SPO2_IR_FILE));
                    result = _vLogTask_OpenFile((FIL*) &fileDesciptorIR, bufferName, strlen(__LOG_ECG_SPO2_IR_FILE));
                }
                break;

            default:
                return;
        }
        if (result == FR_OK)
        {
            fdClosed = false;
            logging = type;
            vStatusBar_setLogging(true);

            xTaskManager_TaskNotifyGive(xLogTaskHandle);
        }
        else
        {
            u32LogStartTime = 0;
            fdClosed = true;
            logging = _LOG_STOP;
        }
    }
}

/*************************************************************************************************/
void vLogTask_StopLog(void)
{

    switch (logging)
    {
        case _LOG_ECG_DATA:
            fdClosed = true;
            stFileSystem_CloseFile((FIL *) &fileDesciptorECG);
            GraphECG_UncheckLog();
            if (vDisplay_GetState() == _WINDOW_ECGGraph)
            {
                vDisplay_ShowMessageBox((char *) xMessage_getECGGraph()->LogFinished, "LOG");
            }
            break;

        case _LOG_SPO2_DATA:
            fdClosed = true;
            stFileSystem_CloseFile((FIL *) &fileDesciptorRed);
            stFileSystem_CloseFile((FIL *) &fileDesciptorIR);
            GraphSPO2_UncheckLog();
            if (vDisplay_GetState() == _WINDOW_SPO2Graph)
            {
                vDisplay_ShowMessageBox((char *) xMessage_getSPO2Graph()->LogFinished, "LOG");
            }
            break;

        case _LOG_STOP:
        default:
            return;
    }

    //Reset Buffers
    if (xLogTaskBufferECG)
    {
        xTaskManager_StreamBufferReset(xLogTaskBufferECG);
    }
    if (xLogTaskBufferSPO2)
    {
        xTaskManager_StreamBufferReset(xLogTaskBufferSPO2);
    }
    u32LogStartTime = 0;
    logging = _LOG_STOP;
    vStatusBar_setLogging(false);
}

/*************************************************************************************************/
void vLogTaskInit(void)
{
    xTaskManager_TaskCreate(vLogTask, "vLogTask", configMINIMAL_STACK_SIZE + 1000,
    NULL, LogTask_PRIORITY, &xLogTaskHandle);

}
