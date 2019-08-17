/**
 ******************************************************************************
 * @file    spo2AcqTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    01 March 2017
 * @brief   Module to manage the spo2 acquisition tasks
 ******************************************************************************
 */

#include "qrsDetector.h"
#include <stdio.h>
#include <math.h>
#include "string.h"
#include "user.h"
#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "task_manager.h"
#include "displayRefreshTask.h"
#include "GraphSPO2.h"
#include "GraphCommon.h"
#include "Display.h"
#include "pin_mux.h"
#include "spo2AcqTask.h"
#include "logTask.h"
#include "Common.h"
#include "FileSystem.h"
#include "SPO2.h"
/*CMSIS includes */
#include "arm_math.h"
#include "InterruptManager.h"
#include "MaximAlgorithm.h"
#include "fsl_port.h"

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         1.4
#define PULSE_MAX_THRESHOLD         3.0
#define PULSE_GO_DOWN_THRESHOLD     1
#define PULSE_BPM_SAMPLE_SIZE       5 //Moving average size
#define MEAN_FILTER_SIZE            10

/* SaO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES    4

/*************************************************************************************************/
/**
 * @enum Heart rate detection states
 */
typedef enum PulseStateMachine
{
    PULSE_IDLE, PULSE_TRACE_UP, PULSE_TRACE_DOWN
} PulseStateMachine;

/*************************************************************************************************/
/**
 * @struct Structure with data result from pulse detection
 */
typedef struct _sPulseDetectResult
{
    float32_t BPM; /** @< Heart rate value */
    uint32_t lastBeatThreshold; /** @< Last beat threshold calculated*/
} sPulseDetectResult_t;

/*************************************************************************************************/
/**
 * @struct Structure containning the data buffers to pass to the graph
 */
typedef struct _spo2GraphData
{
    uint16_t uiData1[SAMPLES_SPO2_PER_BLOCK];
    uint16_t uiData2[SAMPLES_SPO2_PER_BLOCK];
    uint16_t uiData3[SAMPLES_SPO2_PER_BLOCK];
    uint16_t uiData4[SAMPLES_SPO2_PER_BLOCK];
    uint16_t ui16Counter;
} spo2GraphData_t;

/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
typedef struct _SPO2Members
{
    uint32_t countSamplesToInit; /** @< To performa first delay before start graph drawing */

    struct
    {
        TaskMngTaskHandle_t xProcessDataTask; /** @< FreeRTOS Task Handler*/
        TaskMngTaskHandle_t xDrawOnGraphTask; /** @< FreeRTOS Task Handler*/
    } Tasks;
    struct
    {
        TaskMngStreamBufferHandle_t xGraphData; /** @< FreeRTOS Stream Buffer Handler*/
    } Buffers;
    struct
    {
        TaskMngSemaphoreHandle_t xADCRdySemaphore; /** @< FreeRTOS Semphore Handler*/
    } Semaphores;
    struct
    {
        struct
        {
            char string[10];
        } bufferRed[LOG_NUMSAMPLES_SPO2]; /** @< Buffer to hold data for Red LED from SPO2 */

        struct
        {
            char string[10];
        } bufferIR[LOG_NUMSAMPLES_SPO2]; /** @< Buffer to hold data for IR LED from SPO2 */
    } logBuffferSPO2;
} sSPO2Members_t;

/** @brief Pack with all members of this module*/
static sSPO2Members_t SPO2Pack;

/*************************************************************************************************/
/**
 * @brief Task to read and process the LEDs data from the sensor
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vSPO2ProcessTask(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief Task to draw the processed data on the screen
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vSPO2DrawOnGraphTask(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief Detects a pulse according to the mean value of IR sensor, and calculate
 *        the BPM value.
 * @param filterValue mean IR value.
 * @param result pointer the sPulseDetectResult_t structure with BPM and last threshold.
 * @return true if a pulse was detected, false otherwise.
 * @attention ported from https://morf.lv/implementing-pulse-oximeter-using-max30100
 */
static bool bSPO2DetectPulse(float32_t filterValue, sPulseDetectResult_t * result);

static void _vSPO2AcqTask_TurnOnSPO2(void);
static void _vSPO2AcqTask_TurnOffSPO2(void);
/*************************************************************************************************/
static bool bSPO2DetectPulse(float32_t filterValue, sPulseDetectResult_t * result)
{
    static float32_t prev_sensor_value = 0;
    static uint8_t values_went_down = 0;
    static uint32_t currentBeat = 0;
    static uint32_t lastBeat = 0;
    static uint32_t lastBeatThreshold = 0;
    static PulseStateMachine currentPulseDetectorState = PULSE_IDLE;
    static float32_t currentBPM = 0;
    static float32_t valuesBPM[PULSE_BPM_SAMPLE_SIZE] = { 0 };
    static float32_t valuesBPMSum = 0;
    static uint8_t valuesBPMCount = 0;
    static uint8_t bpmIndex = 0;

    if (filterValue > PULSE_MAX_THRESHOLD)
    {
        currentPulseDetectorState = PULSE_IDLE;
        prev_sensor_value = 0;
        lastBeat = 0;
        currentBeat = 0;
        values_went_down = 0;
        lastBeatThreshold = 0;
        result->lastBeatThreshold = lastBeatThreshold;
        result->BPM = currentBPM;
        return false;
    }

    switch (currentPulseDetectorState)
    {
        case PULSE_IDLE:
            if (filterValue >= PULSE_MIN_THRESHOLD)
            {
                currentPulseDetectorState = PULSE_TRACE_UP;
                values_went_down = 0;
            }
            break;

        case PULSE_TRACE_UP:
            if (filterValue > prev_sensor_value)
            {
                currentBeat = xTaskManager_getTickCount();
                lastBeatThreshold = filterValue;
            }
            else
            {
                //Peak reached
                uint32_t beatDuration = currentBeat - lastBeat;
                lastBeat = currentBeat;

                float rawBPM = 0;
                if (beatDuration > 0)
                    rawBPM = 60000.0 / (float) beatDuration;

                valuesBPM[bpmIndex] = rawBPM;
                valuesBPMSum = 0;
                for (int i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++)
                {
                    valuesBPMSum += valuesBPM[i];
                }

                bpmIndex++;
                bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

                if (valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
                    valuesBPMCount++;

                currentBPM = valuesBPMSum / valuesBPMCount;

                currentPulseDetectorState = PULSE_TRACE_DOWN;

                result->lastBeatThreshold = lastBeatThreshold;
                result->BPM = currentBPM;
                return true;
            }
            break;

        case PULSE_TRACE_DOWN:
            if (filterValue < prev_sensor_value)
            {
                values_went_down++;
            }

            if (filterValue < PULSE_MIN_THRESHOLD)
            {
                currentPulseDetectorState = PULSE_IDLE;
            }
            break;
    }

    result->lastBeatThreshold = lastBeatThreshold;
    result->BPM = currentBPM;
    prev_sensor_value = filterValue;
    return false;
}

/*************************************************************************************************/
static void vSPO2ProcessTask(void *pvParameters)
{
    int8_t ch_spo2_valid_maxim;
    int32_t n_heart_rate_maxim;
    int8_t ch_hr_valid_maxim;

    uint32_t aun_ir_buffer[BUFFER_SIZE];     //infrared LED sensor data
    uint32_t aun_red_buffer[BUFFER_SIZE];     //red LED sensor data
    uint8_t decimateFS = 0;
    volatile uint8_t n_buffer_count = 0;
    //////////////////////////////////////////////////////////////////////////////////////
    int32_t redValue = 0;
    int32_t irValue = 0;
    float32_t voltage = 0.0;
    float32_t irACValueSqSum = 0.0;
    float32_t redACValueSqSum = 0.0;
    static float32_t valueToGraph = 0.0;
    static float32_t dcFilteredRed = 0.0;
    static float32_t dcFilteredIR = 0.0;
    static float32_t dcRed = 0.0;
    static float32_t dcIR = 0.0;
    static spo2GraphData_t GraphData;
    static const float32_t ACREDGain = 1000.0;
    static const float32_t ACIRDGain = 1000.0;
    static expAvgFilter_t RedAVGFilter;
    static expAvgFilter_t IRAVGFilter;
    meanDiffFilter_t meanDiffRed;
    meanDiffFilter_t meanDiffIR;
    sPulseDetectResult_t pulseDetect;
    uint16_t samplesRecorded = 0;
    uint16_t pulsesDetected = 0;
    float32_t spo2Value;
    static TaskMngTickType_t xTimeToZeroBPM = 0;
    static uint16_t logDataCounter = 0;

    /****************Enable interrupt on PORTC**********************/
    EnableIRQ(PORTC_IRQn);
    NVIC_SetPriority(PORTC_IRQn, PORTC_INT_PRIO);
    /********************************************************************/

    SPO2Pack.Semaphores.xADCRdySemaphore = xTaskManager_SemaphoreCreate();
    configASSERT(SPO2Pack.Semaphores.xADCRdySemaphore != NULL);

    vCommon_ExpAvgInit(&RedAVGFilter, 0.2, 1.2);
    vCommon_ExpAvgInit(&IRAVGFilter, 0.2, 1.2);

    bCommon_initMeanDiff(&meanDiffRed, MEAN_FILTER_SIZE);
    bCommon_initMeanDiff(&meanDiffIR, MEAN_FILTER_SIZE);

    GraphData.ui16Counter = 0;
    for (;;)
    {
        xTaskManager_SemaphoreTake(SPO2Pack.Semaphores.xADCRdySemaphore, portMAX_DELAY);

        //Process Data Here
        PORT_SetPinInterruptConfig(GPIO_AFE4400_ADCRDY_PORT, GPIO_AFE4400_ADCRDY_PIN, kPORT_InterruptOrDMADisabled);
        if (sSPO2_ReadRedLED((uint32_t *) &redValue) == kStatus_Success)
        {
            //Conversion from 24 bits to float
            voltage = (float32_t) (redValue * (0.00000057220458984375));

            /** Remove DC componente */
            vCommon_dcRemoval(voltage, dcRed, 0.95, &dcFilteredRed, &dcRed);

            /** Smooth singnal */
            dcFilteredRed = fCommon_ExpAvg(&RedAVGFilter, dcFilteredRed);

            redACValueSqSum += (dcFilteredRed * dcFilteredRed);

            /** Adjusting the graph on top */
            valueToGraph = dcFilteredRed * ACREDGain;
            valueToGraph += 3.6;     //Add DC to attach the curve in the middle
            valueToGraph = (uint16_t) ((valueToGraph / (2.4)) * (VTotalGridPX / 2));
            valueToGraph = (valueToGraph > VTotalGridPX) ? VTotalGridPX : valueToGraph;
            GraphData.uiData1[GraphData.ui16Counter] = (VTotalGridPX - (uint32_t) valueToGraph) + VTotalGridPX / 2;

            if (sSPO2_ReadIRLED((uint32_t *) &irValue) == kStatus_Success)
            {
                if (decimateFS >= (PRF / FS))
                {
                    aun_red_buffer[n_buffer_count] = (uint32_t) (redValue >> 4);
                    aun_ir_buffer[n_buffer_count] = (uint32_t) (irValue >> 4);
                    n_buffer_count++;
                    decimateFS = 0;
                }
                decimateFS++;
                if (n_buffer_count >= BUFFER_SIZE)
                {
                    n_buffer_count = 0;
                    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &spo2Value,
                            &ch_spo2_valid_maxim, &n_heart_rate_maxim, &ch_hr_valid_maxim);

                    vGraphSPO2_setSPO2Value((uint8_t) spo2Value);
                }

                //Conversion from 24 bits to float
                voltage = (float32_t) (irValue * (0.00000057220458984375));

                if (xLogTask_isLogging() == _LOG_SPO2_DATA)
                {
                    sprintf((char *) SPO2Pack.logBuffferSPO2.bufferIR[logDataCounter].string, "%08X;",
                            (unsigned int) irValue);
                    sprintf((char *) SPO2Pack.logBuffferSPO2.bufferRed[logDataCounter].string, "%08X;",
                            (unsigned int) redValue);
                    logDataCounter++;
                    if (logDataCounter >= LOG_NUMSAMPLES_SPO2)
                    {
                        logDataCounter = 0;
                        vLogTask_PostData((void *) &SPO2Pack.logBuffferSPO2);
                    }
                }
                else
                {
                    logDataCounter = 0;
                }
                /** Remove DC componente */
                vCommon_dcRemoval(voltage, dcIR, 0.95, &dcFilteredIR, &dcIR);

                /** Smooth singnal */
                dcFilteredIR = fCommon_ExpAvg(&IRAVGFilter, dcFilteredIR);

                irACValueSqSum += (dcFilteredIR * dcFilteredIR);

                valueToGraph = dcFilteredIR * ACIRDGain;

                /** Adjusting the graph on bottom */
                valueToGraph += 1.2;     //Add DC to attach the curve in the middle
                valueToGraph = (uint16_t) ((valueToGraph / (2.4)) * (VTotalGridPX / 2));
                valueToGraph = (valueToGraph > VTotalGridPX / 2) ? VTotalGridPX / 2 : valueToGraph;
                GraphData.uiData2[GraphData.ui16Counter] = VTotalGridPX / 2 - (uint32_t) valueToGraph;

                /** Detecting Peaks */
                valueToGraph = dcFilteredIR * 1000.0;
                fCommon_meanDiff(&meanDiffIR, valueToGraph);
                /** Adjusting the graph on bottom */
                meanDiffIR.mean += 1.2;
                valueToGraph = (uint16_t) ((meanDiffIR.mean / (2.4)) * (VTotalGridPX / 2));
                valueToGraph = (valueToGraph > VTotalGridPX / 2) ? VTotalGridPX / 2 : valueToGraph;
                GraphData.uiData4[GraphData.ui16Counter] = (uint32_t) valueToGraph;

                /** Calculating BPM and SPO2 */
                samplesRecorded++;
                if (bSPO2DetectPulse(meanDiffIR.mean, &pulseDetect) && samplesRecorded > 0)
                {
                    pulsesDetected++;

                    if (pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
                    {
                        irACValueSqSum = 0;
                        redACValueSqSum = 0;
                        samplesRecorded = 0;
                    }

                    xTimeToZeroBPM = xTaskManager_getTickCount();     //start Time
                    vGraphSPO2_setHeartRateValue((uint16_t) pulseDetect.BPM);
                }

                if (xTaskManager_getTickCount() - xTimeToZeroBPM > 10000)
                {
                    //after 10 seconds if we do not have any pulse, then zero the display
                    pulseDetect.BPM = 0;
                    vGraphSPO2_setHeartRateValue((uint8_t) 0);
                    vGraphSPO2_setSPO2Value((uint8_t) 0);
                }
            }
        }

        PORT_SetPinInterruptConfig(GPIO_AFE4400_ADCRDY_PORT, GPIO_AFE4400_ADCRDY_PIN, GPIO_AFE4400_ADCRDY_INT);

        GraphData.ui16Counter++;

        if (GraphData.ui16Counter >= SAMPLES_SPO2_PER_BLOCK)
        {
            if (SPO2Pack.Buffers.xGraphData)
            {
                xTaskManager_StreamBufferSend(SPO2Pack.Buffers.xGraphData, (const void *) &GraphData,
                        sizeof(spo2GraphData_t), 0);
            }
            GraphData.ui16Counter = 0;
        }
    }

    vTaskDelete(NULL);
}
/*************************************************************************************************/
static void vSPO2DrawOnGraphTask(void *pvParameters)
{
    uint16_t uiIndex = 0;
    uint16_t ui16BlockCounter;
    static spo2GraphData_t GraphData;
    SPO2Pack.Buffers.xGraphData = xTaskManager_StreamBufferCreate(2 * sizeof(spo2GraphData_t), sizeof(spo2GraphData_t));
    configASSERT(SPO2Pack.Buffers.xGraphData != NULL);

    for (;;)
    {
        ui16BlockCounter = (xTaskManager_StreamBufferReceive(SPO2Pack.Buffers.xGraphData, (void *) &GraphData,
                sizeof(spo2GraphData_t), 500));
        if (ui16BlockCounter)
        {
            for (uiIndex = 0; uiIndex < GraphData.ui16Counter; uiIndex++)
            {
                switch (vDisplay_GetState())
                {
                    case _WINDOW_SPO2Graph:
                        if (SPO2Pack.countSamplesToInit < 80)
                        {
                            SPO2Pack.countSamplesToInit++;
                            break;
                        }
                        vGraphSPO2_AddDataMainGraph(GraphData.uiData1[uiIndex], _E_DATA_SPO2_RED_RAW);
                        vGraphSPO2_AddDataMainGraph(GraphData.uiData2[uiIndex], _E_DATA_SPO2_IR_RAW);
//                        vGraphSPO2_AddDataMainGraph(GraphData.uiData4[uiIndex], _E_DATA_SPO2_FILTERED);
                        vGraphSPO2_PostData();
                        xTaskManager_TaskNotifyGive(xUpdateDisplayTaskGetHandler());
                        break;

                    default:
                        break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void vSPO2Task_IRQ_DataReady(uint32_t status)
{
    if (status == (1 << GPIO_AFE4400_ADCRDY_PIN))
    {
        /* Clear external interrupt flag. */
        GPIO_ClearPinsInterruptFlags(GPIO_AFE4400_ADCRDY_GPIO, 1 << GPIO_AFE4400_ADCRDY_PIN);

        if (SPO2Pack.Semaphores.xADCRdySemaphore)
        {
            xTaskManager_SemaphoreGive(SPO2Pack.Semaphores.xADCRdySemaphore);
        }
    }
}
/*************************************************************************************************/
void vSPO2AcqTaskInit(void)
{
    memset(&SPO2Pack, 0, sizeof(sSPO2Members_t));

    xTaskCreate(vSPO2ProcessTask, "vSPO2ProcessTask",
    configMINIMAL_STACK_SIZE + 600,
    NULL, SpO2ProcessTask_PRIORITY, &SPO2Pack.Tasks.xProcessDataTask);
    configASSERT(SPO2Pack.Tasks.xProcessDataTask != NULL);

    xTaskCreate(vSPO2DrawOnGraphTask, "vSPO2DrawOnGraphTask",
    configMINIMAL_STACK_SIZE + 500,
    NULL, SpO2DrawOnGraphTask_PRIORITY, &SPO2Pack.Tasks.xDrawOnGraphTask);
    configASSERT(SPO2Pack.Tasks.xDrawOnGraphTask != NULL);

}
/*************************************************************************************************/
void vSPO2AcqTaskResume(void)
{
    _vSPO2AcqTask_TurnOnSPO2();
    if (SPO2Pack.Tasks.xProcessDataTask)
        vTaskManager_TaskResume(SPO2Pack.Tasks.xProcessDataTask);
    if (SPO2Pack.Tasks.xDrawOnGraphTask)
        vTaskManager_TaskResume(SPO2Pack.Tasks.xDrawOnGraphTask);
    vSPO2_Init();
    PORT_SetPinInterruptConfig(GPIO_AFE4400_ADCRDY_PORT, GPIO_AFE4400_ADCRDY_PIN, GPIO_AFE4400_ADCRDY_INT);
}
/*************************************************************************************************/
void vSPO2AcqTaskSuspend(void)
{
    _vSPO2AcqTask_TurnOffSPO2();
    sSPO2_PowerControl(_E_POWER_OFF);
    PORT_SetPinInterruptConfig(GPIO_AFE4400_ADCRDY_PORT, GPIO_AFE4400_ADCRDY_PIN, kPORT_InterruptOrDMADisabled);
    if (SPO2Pack.Tasks.xProcessDataTask)
        vTaskManager_TaskSuspend(SPO2Pack.Tasks.xProcessDataTask);
    if (SPO2Pack.Tasks.xDrawOnGraphTask)
        vTaskManager_TaskSuspend(SPO2Pack.Tasks.xDrawOnGraphTask);
}

/*************************************************************************************************/
static void _vSPO2AcqTask_TurnOnSPO2(void)
{
    GPIO_SetPinsOutput(GPIO_AFE4400_POWER_EN_GPIO, 1U << GPIO_AFE4400_POWER_EN_PIN);
    vTaskManager_Delay(1);
}

/*************************************************************************************************/
static void _vSPO2AcqTask_TurnOffSPO2(void)
{
    GPIO_ClearPinsOutput(GPIO_AFE4400_POWER_EN_GPIO, 1U << GPIO_AFE4400_POWER_EN_PIN);
    vTaskManager_Delay(1);
}
/*************************************************************************************************/
