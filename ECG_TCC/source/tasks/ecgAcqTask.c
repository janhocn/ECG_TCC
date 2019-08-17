/**
 ******************************************************************************
 * @file    ecgAcqTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 August 2017
 * @brief   Module to manage the ecg acquisition tasks
 ******************************************************************************
 */
#include <stdio.h>
#include "string.h"
#include "pin_mux.h"
#include "user.h"
#include "qrsDetector.h"
#include "prioAssigner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "task_manager.h"
#include "ecgAcqTask.h"
#include "Common.h"
#include "FileSystem.h"
#include "kalman_filter.h"
#include "displayRefreshTask.h"
#include "GraphECG.h"
#include "GraphSPO2.h"
#include "GraphCommon.h"
#include "Display.h"
#include "InterruptManager.h"
#include "logTask.h"
/*CMSIS includes */
#include "arm_math.h"
/*Filter Coefficients*/
#include "fdacoeffs_lp15_fir.h"
#include "fdacoeffs_lp150_eliptic.h"
#include "fdacoefs_notch60_Eliptic.h"
#include "fdacoefs_notch120_Eliptic.h"
#include "fdacoefs_diffFir.h"
#include "fdacoeffs_hp5_fir.h"


/*************************************************************************************************/
/**
 * @brief Task to draw the processed data on the screen
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vEcgDrawOnGraphTask(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief Task to process the ecg data (filtering and conditioning)
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vEcgProcessDataTask(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief Task print the average Heart Rate on the screen
 * @param pvParameters Pointer to user paramenters (refer to FreeRTOS API).
 * @return void.
 */
static void vEcgPrintHR(void *pvParameters);

/*************************************************************************************************/
/**
 * @brief Fucntion to configure and initialize the ADC channel.
 * @return void.
 */
static void _vEcgACQTaskADCInit(void);

/*************************************************************************************************/
/**
 * @brief Fucntion to initialize the filters
 * @return void.
 */
static void _vEcgACQTaskFilterInit(void);

/*************************************************************************************************/
/**
 * @brief Fucntion to initialize the PIT timer to trigger the ADC conversion
 * @return void.
 */
static void _vEcgACQTaskPITInit(void);

/*************************************************************************************************/
/**
 * @brief Performs the QRS detection filters as in the pan tompkins algorithm
 *        In->High Pass->Low Pass->Differentiate->Squaring->Integration->Out
 * @param in pointer to the input data to be filtered (size 1)
 * @param out pointer to the filtered output data (size 1)
 * @return void.
 */
static void _vEcgAcqTask_FilterForQRS(float32_t* in, float32_t* out);

static void _vEcgAcqTask_TurnOffECG(void);
static void _vEcgAcqTask_TurnOnECG(void);
/*************************************************************************************************/
/**
 * @def Max resolution ADC value
 */
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
#define ADC_MAX (float32_t)65535.0
#else
#define ADC_MAX (float32_t)4095.0
#endif

/*************************************************************************************************/
/**
 * @enum Enumeration with data block sizes
 */
enum
{
    SAMPLES_BLOCK_ECG = (uint16_t) (SAMPLES_PER_BLOCK * SAMPLES_PER_PIXEL ), /** @< Size of the block to proccess ECG Data */
    SAMPLES_PER_QRS = 100, /** @< Size of the block with the HR value to get the average */
};

/*************************************************************************************************/
/**
 * @enum Enumeration with the FIR state size for QRS filters
 */
enum
{
    STATEDIFF_SIZE = (1 + TAPS_DIFF - 1), /** @< State size for the differentiation filter */
    STATELPDIFF_SIZE = (1 + TAPS_LP15_FIR - 1), /** @< State size for the high-pass filter */
    STATEHPDIFF_SIZE = (1 + TAPS_HP5_FIR - 1), /** @< State size for the low-pass filter */
    STATEHPDC_SIZE = (1 + TAPS_HP3_FIR - 1), /** @< State size for the low-pass filter */
};

/*************************************************************************************************/
/**
 * @struct Structure containning the data buffers to pass to the graph
 */
typedef struct _sGraphData
{
    uint16_t uiDataArr1[SAMPLES_PER_BLOCK]; /** @< Block data to graph data 1 */
    uint16_t uiDataArr2[SAMPLES_PER_BLOCK]; /** @< Block data to graph data 2 */
    uint16_t xCounter; /** @< Time index counter */
} sGraphData;

/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
typedef struct _sECGMembers
{
    q15_t q15ECGHR; /** @< Hold the Average HR value */
    bool isADCInitialized; /** @< Indicates that the module has been initialized*/
    uint32_t countSamplesToInit; /** @< To performa first delay before start graph drawing */

    struct
    {
        adc16_config_t adc16ConfigStruct; /** @< KDS ADC configuration structure*/
        adc16_channel_config_t adc16ChannelConfigStruct; /** @< KDS ADC channel configuration structure*/
    } ADC;

    struct
    {
        TaskMngTaskHandle_t xProcessDataTask; /** @< FreeRTOS Task Handler*/
        TaskMngTaskHandle_t xPrintHRTask; /** @< FreeRTOS Task Handler*/
        TaskMngTaskHandle_t xDrawOnGraphTask; /** @< FreeRTOS Task Handler*/
    } Tasks;

    struct
    {
        float32_t coeffsMovingAverage[50]; /** @< Buffer to hold the moving average coefficients*/
        struct
        {
            arm_biquad_cascade_df2T_instance_f32 iir2LowPass140Hz; /** @< Instance of IIR biquad 140Hz Low Pass (For ECG)*/
            arm_biquad_cascade_df2T_instance_f32 iir2Notch60Hz; /** @< Instance of IIR biquad 60Hz Notch (For ECG)*/
            arm_biquad_cascade_df2T_instance_f32 iir2Notch120Hz; /** @< Instance of IIR biquad 120Hz Notch (For ECG)*/
            arm_fir_instance_f32 firLowPassForDiff; /** @< Instance of FIR 15Hz Low Pass (For QRS Detection)*/
            arm_fir_instance_f32 firHighPassForDiff; /** @< Instance of FIR 5Hz High Pass (For QRS Detection)*/
            arm_fir_instance_f32 firDifferentiator; /** @< Instance of 8 points derivative (For QRS Detection)*/
            arm_fir_instance_f32 firHighPassForDC; /** @< Instance of FIR 3Hz High Pass (For QRS Detection)*/
            arm_fir_instance_f32 firMovingAVG; /** @< Instance of 150ms window moving average (For QRS Detection)*/

        } Instances;

        struct
        {
            kalman_state kalmanState; /** @< STATE Instance for kalman filter (To smooth the ECG)*/
            float32_t *firStateMovingAVG; /** @< STATE of 150ms window moving average (For QRS Detection)*/
            float32_t firStateDiff[STATEDIFF_SIZE]; /** @< STATE of 8 points derivative (For QRS Detection)*/
            float32_t firStateHPDiff[STATEHPDIFF_SIZE]; /** @< STATE of FIR 5Hz High Pass (For QRS Detection)*/
            float32_t firStateLPDiff[STATELPDIFF_SIZE]; /** @< STATE of FIR 15Hz Low Pass (For QRS Detection)*/
//            float32_t firStateHPDC[STATEHPDC_SIZE]; /** @< STATE of FIR 3Hz High Pass (For DC Block)*/
            float32_t faIIRstateLPElip140Hz[2 * SECTIONS_ELIP150]; /** @< STATE of IIR biquad 140Hz Low Pass (For ECG)*/
            float32_t faIIR2stateNElip60Hz[2 * SECTIONS_ELIPNOTCH60]; /** @< STATE of IIR biquad 60Hz Notch (For ECG)*/
            float32_t faIIR2stateNElip120Hz[2 * SECTIONS_ELIPNOTCH120]; /** @< STATE of IIR biquad 120Hz Notch (For ECG)*/
        } States;
    } Filters;

    struct
    {
        TaskMngStreamBufferHandle_t xGraphData; /** @< FreeRTOS Stream Buffer Handler*/
        TaskMngStreamBufferHandle_t xECGStreamBuffer; /** @< FreeRTOS Stream Buffer Handler*/
        TaskMngStreamBufferHandle_t xECGQRSStreamBuffer; /** @< FreeRTOS Stream Buffer Handler*/

        struct
        {
            char string[5];
        } logBufferECG[LOG_NUMSAMPLES_ECG]; /** @< Buffer to hold data for ECG RAW Data */
    } Buffers;

    FIL fileECG;
} sECGMembers_t;

/*************************************************************************************************/
/**
 * @var Pack with all members of this module
 */
static sECGMembers_t ECGpack;

/*************************************************************************************************/
static void vEcgProcessDataTask(void *pvParameters)
{
    float32_t meanValue;
    float32_t fECGRaw[SAMPLES_BLOCK_ECG] = { 0 };
    float32_t fECGFiltered[SAMPLES_BLOCK_ECG] = { 0 };
    float32_t fECGBaseline[SAMPLES_BLOCK_ECG] = { 0 };
    uint16_t ui16BlockCounter;
    sGraphData GraphData;
    float32_t ecgGain = 2.0;
    static float32_t result = 0.0;
    static float32_t dcECG = 0.0;

    ECGpack.Buffers.xECGStreamBuffer = xTaskManager_StreamBufferCreate(SAMPLES_BLOCK_ECG * sizeof(float32_t),
            SAMPLES_BLOCK_ECG * sizeof(float32_t));

    configASSERT(ECGpack.Buffers.xECGStreamBuffer != NULL);

    for (;;)
    {
        ui16BlockCounter = xTaskManager_StreamBufferReceive(ECGpack.Buffers.xECGStreamBuffer, (void *) fECGRaw,
                SAMPLES_BLOCK_ECG * sizeof(float32_t), 500);
        ui16BlockCounter /= sizeof(float32_t);

        if (ui16BlockCounter == SAMPLES_BLOCK_ECG)
        {
            memset(&GraphData, 0, sizeof(sGraphData));

            /**************************** Filtering ECG RAW **********************************************/
            arm_biquad_cascade_df2T_f32(&ECGpack.Filters.Instances.iir2LowPass140Hz, fECGRaw, fECGBaseline,
                    ui16BlockCounter);
            ///////////////////////////////////////////////////////////////////////////////////////////////
            arm_biquad_cascade_df2T_f32(&ECGpack.Filters.Instances.iir2Notch60Hz, fECGBaseline, fECGFiltered,
                    ui16BlockCounter);
            ///////////////////////////////////////////////////////////////////////////////////////////////
            arm_biquad_cascade_df2T_f32(&ECGpack.Filters.Instances.iir2Notch120Hz, fECGFiltered, fECGBaseline,
                    ui16BlockCounter);
            ///////////////////////////////////////////////////////////////////////////////////////////////
            //Removing Baseline
//            arm_fir_f32(&ECGpack.Filters.Instances.firHighPassForDC, fECGBaseline, fECGFiltered, ui16BlockCounter);
            for (uint16_t i = 0; i < ui16BlockCounter; i++)
            {
//                kalman_update(&ECGpack.Filters.States.kalmanState, fECGFiltered[i]);
//                fECGBaseline[i] = ECGpack.Filters.States.kalmanState.x;
//                fECGFiltered[i] = fECGFiltered[i] - fECGBaseline[i] + (ADC_MAX / 2.0);
                vCommon_dcRemoval(fECGFiltered[i], dcECG, 0.998, &result, &dcECG);
                fECGBaseline[i] = dcECG;
                fECGFiltered[i] = ecgGain*result + (ADC_MAX / 2.0);
//                fECGFiltered[i] = ecgGain*fECGFiltered[i] + (ADC_MAX / 2.0);
            }

            switch (vDisplay_GetState())
            {
                case _WINDOW_ECGGraph:
                    for (uint16_t i = 0; i < ui16BlockCounter; i += SAMPLES_PER_PIXEL)
                    {
                        if (GraphData.xCounter < XMAXDisplay)
                        {
#if USE_PROTOTYPE
                            arm_mean_f32((float32_t *) &fECGRaw[i], SAMPLES_PER_PIXEL, (float32_t *) &meanValue);
                            GraphData.uiDataArr1[GraphData.xCounter] = (uint16_t)((meanValue / ADC_MAX)
                                    * (VTotalGridPX));
                            arm_mean_f32((float32_t *) &fECGFiltered[i], SAMPLES_PER_PIXEL, (float32_t *) &meanValue);
                            GraphData.uiDataArr2[GraphData.xCounter] = (uint16_t)((meanValue / ADC_MAX)
                                    * (VTotalGridPX));
#else
//                            arm_mean_f32((float32_t *) &fECGRaw[i], SAMPLES_PER_PIXEL, (float32_t *) &meanValue);
//                            GraphData.uiDataArr1[GraphData.xCounter] = (uint16_t) ((meanValue / ADC_MAX )
//                                    * (VTotalGridPX));
                            arm_mean_f32((float32_t *) &fECGFiltered[i], SAMPLES_PER_PIXEL, (float32_t *) &meanValue);
                            GraphData.uiDataArr2[GraphData.xCounter] = VTotalGridPX
                                    - (uint16_t) ((meanValue / ADC_MAX ) * (VTotalGridPX));
#endif
                        }
                        GraphData.xCounter++;
                    }

                    if (ECGpack.Buffers.xGraphData)
                    {
                        if (GraphData.xCounter > XMAXDisplay)
                        {
                            GraphData.xCounter = XMAXDisplay;
                        }
                        xTaskManager_StreamBufferSend(ECGpack.Buffers.xGraphData, (const void *) &GraphData,
                                sizeof(sGraphData), 200);
                    }
                    break;

                default:
                    break;
            }

        }
    }

    vTaskDelete(NULL);
}

/*************************************************************************************************/
void _vEcgAcqTask_FilterForQRS(float32_t* in, float32_t* out)
{
    float32_t fECGScratchPad;

    /******************************* Detecting QRS Complex *******************************************/
    /**Band Pass Filtering for removing any DC and High Frequency noises*/
    arm_fir_f32(&ECGpack.Filters.Instances.firLowPassForDiff, in, out, 1);
    arm_fir_f32(&ECGpack.Filters.Instances.firHighPassForDiff, out, &fECGScratchPad, 1);
    /*************************************************************************************************/
    /**Do the derivative of the filtered ECG signal*/
    arm_fir_f32(&ECGpack.Filters.Instances.firDifferentiator, &fECGScratchPad, out, 1);
    /*************************************************************************************************/
    /**Taking the squared value to highlight only positive values*/
    arm_mult_f32(out, out, &fECGScratchPad, 1);
    /*************************************************************************************************/
    /**Smooth the derivative of the filtered ECG signal (Moving Integral)*/
    arm_fir_f32(&ECGpack.Filters.Instances.firMovingAVG, &fECGScratchPad, out, 1);
}

/*************************************************************************************************/
static void vEcgDrawOnGraphTask(void *pvParameters)
{
    uint16_t uiIndex = 0;
    uint16_t ui16BlockCounter;
    sGraphData GraphData;
    ECGpack.Buffers.xGraphData = xTaskManager_StreamBufferCreate(2 * sizeof(sGraphData), sizeof(sGraphData));
    configASSERT(ECGpack.Buffers.xGraphData != NULL);

    for (;;)
    {
        ui16BlockCounter = (xTaskManager_StreamBufferReceive(ECGpack.Buffers.xGraphData, (void *) &GraphData,
                sizeof(sGraphData), 500));
        if (ui16BlockCounter)
        {

            for (uiIndex = 0; uiIndex < GraphData.xCounter; uiIndex++)
            {
                switch (vDisplay_GetState())
                {
                    case _WINDOW_ECGGraph:
                        if (ECGpack.countSamplesToInit < 80)
                        {
                            ECGpack.countSamplesToInit++;
                            break;
                        }

                        vGraphECG_AddDataMainGraph(GraphData.uiDataArr2[uiIndex], _E_DATA_ECG_FILTERED);
                        vGraphECG_AddDataMainGraph(GraphData.uiDataArr1[uiIndex], _E_DATA_ECG_RAW);
                        vGraphECG_PostData();
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

/*************************************************************************************************/
static void vEcgPrintHR(void *pvParameters)
{
    q15_t fECGScratchRawQRS[SAMPLES_PER_QRS] = { 0 };
    uint16_t ui16QRSBlockCounter = 0;
    ECGpack.Buffers.xECGQRSStreamBuffer = xTaskManager_StreamBufferCreate(sizeof(fECGScratchRawQRS),
            sizeof(fECGScratchRawQRS));
    configASSERT(ECGpack.Buffers.xECGQRSStreamBuffer != NULL);

    for (;;)
    {
        ui16QRSBlockCounter = (xTaskManager_StreamBufferReceive(ECGpack.Buffers.xECGQRSStreamBuffer,
                (void *) fECGScratchRawQRS, sizeof(fECGScratchRawQRS), 1500));
        ui16QRSBlockCounter /= sizeof(q15_t);
        if (ui16QRSBlockCounter == SAMPLES_PER_QRS)
        {
            arm_mean_q15(fECGScratchRawQRS, ui16QRSBlockCounter, &ECGpack.q15ECGHR);
            vGraphECG_setHeartRateValue((uint16_t) ECGpack.q15ECGHR);
            vGraphECG_setRRInterval((uint16_t) i32QRSDetecto_getRRInterval(5.1));
        }
    }

    vTaskDelete(NULL);
}

/*************************************************************************************************/
__INLINE void vEcgAcqTask_IRQ_AnalogRead(void)
{
    static uint16_t counterSamples = 0;
    static uint16_t counterSamplesQRS = 0;
    static uint16_t counterSamplesLOG = 0;
    static uint16_t downSamplingTimeQRS = 0;
    uint16_t u16ECGRawData = 0;
    float32_t fECGRawData = 0;
    float32_t fECGFiltQRS = 0;
    bool detected = false;
    static uint16_t countRR = 0;
    static float32_t bufferSendRaw[SAMPLES_BLOCK_ECG] = { 0 };
    static q15_t bufferHRSend[SAMPLES_PER_QRS] = { 0 };
    /****************************************************************************/

    /**Read ECG sample from ADC channel*/
    fECGRawData = (float32_t) ADC16_GetChannelConversionValue(GPIO_ECG_IN_ADC_BASE_ADDR, 0U);

    /** Convert data to logging*/
    u16ECGRawData = (uint32_t) fECGRawData;

    /** Buffer to be processed */
    bufferSendRaw[counterSamples] = fECGRawData;

    /**Downsampling the rate for QRS Detection*/
    downSamplingTimeQRS++;
    if (downSamplingTimeQRS == adcNumSamplesQRS)
    {
        downSamplingTimeQRS = 0;
        /** Scaling the data because the QRS filtering output is too high */
        /** The value 1000 was chosen based on MATLAB simulations */
        fECGRawData /= 1000;
        _vEcgAcqTask_FilterForQRS(&fECGRawData, &fECGFiltQRS);
        detected = bQRSDetector_Detect((int32_t) fECGFiltQRS);
        if (detected)
        {
            countRR = 0;
            vGraphECG_toggleHRTextColor();
        }
        else
        {
            countRR++;
        }

        /** if in four secons any R peak is detected, then lets assume zero BPM */
        if (countRR >= (uint16_t) (4000 / adcQRSSamplePeriodms))
        {
            /**Reset the levels and thresholds*/
            vQRSDetector_resetDetection();
            countRR = 0;
            bufferHRSend[counterSamplesQRS] = 0;
        }
        else
        {
            /** accumulate a heart rate buffer for taking the average */
            bufferHRSend[counterSamplesQRS] = (q15_t) i32QRSDetecto_calculatePulse(5.1E-3);
        }
        counterSamplesQRS++;
    }

    if (counterSamplesQRS == SAMPLES_PER_QRS)
    {
        counterSamplesQRS = 0;
        if (ECGpack.Buffers.xECGQRSStreamBuffer)
        {
            xTaskManager_StreamBufferSend(ECGpack.Buffers.xECGQRSStreamBuffer, (const void *) bufferHRSend,
                    SAMPLES_PER_QRS * sizeof(q15_t), 0);
        }
    }

    if (xLogTask_isLogging() == _LOG_ECG_DATA)
    {
        sprintf((char *) ECGpack.Buffers.logBufferECG[counterSamplesLOG].string, "%04X;", u16ECGRawData);
        counterSamplesLOG++;
        if (counterSamplesLOG >= LOG_NUMSAMPLES_ECG)
        {
            counterSamplesLOG = 0;

            vLogTask_PostData((void *) &ECGpack.Buffers.logBufferECG[0]);
        }
    }
    else
    {
        counterSamplesLOG = 0;
    }

    counterSamples++;
    if (counterSamples == SAMPLES_BLOCK_ECG)
    {
        counterSamples = 0;
        if (ECGpack.Buffers.xECGQRSStreamBuffer)
        {
            xTaskManager_StreamBufferSend(ECGpack.Buffers.xECGStreamBuffer, (const void *) bufferSendRaw,
                    SAMPLES_BLOCK_ECG * sizeof(float32_t), 0);
        }
    }
}

/*************************************************************************************************/
static void _vEcgACQTaskFilterInit(void)
{
    uint16_t NMovingAVG;
    /**Smoothing filter to remove baseline wander*/
    kalman_init(&ECGpack.Filters.States.kalmanState, 0.03, 5000, 1, 32767);

    /**Moving window of timeWindowMovingAVG seconds*/
    NMovingAVG = (float32_t) (timeWindowMovingAVG * (1 / (adcQRSSamplePeriodms / 1000.0f)));
    arm_fill_f32((float32_t) (1.0 / NMovingAVG), ECGpack.Filters.coeffsMovingAverage, NMovingAVG);

    ECGpack.Filters.States.firStateMovingAVG = (float32_t *) pvPortMalloc(NMovingAVG * sizeof(float32_t));
    configASSERT(ECGpack.Filters.States.firStateMovingAVG != NULL);

    /**Moving Average filter (used for QRS detection) */
    arm_fir_init_f32(&ECGpack.Filters.Instances.firMovingAVG, NMovingAVG,
            (float32_t*) ECGpack.Filters.coeffsMovingAverage, ECGpack.Filters.States.firStateMovingAVG, 1);
    /**Derivative filter (used for QRS detection) */
    arm_fir_init_f32(&ECGpack.Filters.Instances.firDifferentiator, TAPS_DIFF, (float32_t*) baDifferentiator_FirCoeffs,
            ECGpack.Filters.States.firStateDiff, 1);
    /**High-Pass filter (used for QRS detection) */
    arm_fir_init_f32(&ECGpack.Filters.Instances.firHighPassForDiff, TAPS_HP5_FIR, (float32_t*) baHP5Fir,
            ECGpack.Filters.States.firStateHPDiff, 1);
    /**Low-Pass filter (used for QRS detection) */
    arm_fir_init_f32(&ECGpack.Filters.Instances.firLowPassForDiff, TAPS_LP15_FIR, (float32_t*) baLP15Fir,
            ECGpack.Filters.States.firStateLPDiff, 1);

    /**High-Pass filter (used for DC Block) */
//    arm_fir_init_f32(&ECGpack.Filters.Instances.firHighPassForDC, TAPS_HP3_FIR, (float32_t*) baHP3Fir,
//            ECGpack.Filters.States.firStateHPDC, 1);
    /**60Hz Notch filter (used for main signal acquisition) */
    arm_biquad_cascade_df2T_init_f32(&ECGpack.Filters.Instances.iir2Notch60Hz, SECTIONS_ELIPNOTCH60,
            (float32_t*) baElipticNotch60Coeffs, ECGpack.Filters.States.faIIR2stateNElip60Hz);
    /**120Hz Notch filter (used for main signal acquisition) */
    arm_biquad_cascade_df2T_init_f32(&ECGpack.Filters.Instances.iir2Notch120Hz, SECTIONS_ELIPNOTCH120,
            (float32_t*) baElipticNotch120Coeffs, ECGpack.Filters.States.faIIR2stateNElip120Hz);
    /**150Hz Low Pass filter (used for main signal acquisition) */
    arm_biquad_cascade_df2T_init_f32(&ECGpack.Filters.Instances.iir2LowPass140Hz, SECTIONS_ELIP150,
            (float32_t*) baEliptic150Coeffs, ECGpack.Filters.States.faIIRstateLPElip140Hz);
}

/*************************************************************************************************/
static void _vEcgACQTaskADCInit(void)
{
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    uint16_t offsetValue = 0; /*!< Offset error from correction value. */
#endif

    ADC16_GetDefaultConfig(&ECGpack.ADC.adc16ConfigStruct);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    ECGpack.ADC.adc16ConfigStruct.resolution = kADC16_Resolution16Bit;
#else
    ECGpack.ADC.adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
#endif
    /** enabled hardware trigger  */
    ECGpack.ADC.adc16ConfigStruct.enableContinuousConversion = false;
    ECGpack.ADC.adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;

    ECGpack.ADC.adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
    ECGpack.ADC.adc16ConfigStruct.enableLowPower = true;
#if ((defined BOARD_ADC_USE_ALT_VREF) && BOARD_ADC_USE_ALT_VREF)
    ECGpack.ADC.adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(GPIO_ECG_IN_ADC_BASE_ADDR, &ECGpack.ADC.adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(GPIO_ECG_IN_ADC_BASE_ADDR, true);

    ADC16_SetHardwareAverage(GPIO_ECG_IN_ADC_BASE_ADDR, kADC16_HardwareAverageDisabled);

#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    /** Auto calibration */
    ADC16_DoAutoCalibration(GPIO_ECG_IN_ADC_BASE_ADDR);
    offsetValue = GPIO_ECG_IN_ADC_BASE_ADDR->OFS;
    ADC16_SetOffsetValue(GPIO_ECG_IN_ADC_BASE_ADDR, offsetValue);
#endif

    ECGpack.ADC.adc16ChannelConfigStruct.channelNumber = GPIO_ECG_IN_ADC_CHANNEL;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    ECGpack.ADC.adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif
    ECGpack.ADC.adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;
    /** Configure channel 0 */
    ADC16_SetChannelConfig(GPIO_ECG_IN_ADC_BASE_ADDR, GPIO_ECG_IN_ADC_CHANNEL_GROUP,
            &ECGpack.ADC.adc16ChannelConfigStruct);

#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) && (FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
    ADC16_EnableDMA(GPIO_ECG_IN_ADC_BASE_ADDR, false);
#endif

    NVIC_ClearPendingIRQ(GPIO_ECG_IN_ADC_IRQ);
    NVIC_SetPriority(GPIO_ECG_IN_ADC_IRQ, ADC_ECG_INT_PRIO);
    NVIC_DisableIRQ(GPIO_ECG_IN_ADC_IRQ);
    __DSB();
    __ISB();

    _vEcgACQTaskPITInit();
    ECGpack.isADCInitialized = true;
}

/*************************************************************************************************/
static void _vEcgACQTaskPITInit(void)
{
#if USE_LPTMR_AS_TIMER
    lptmr_config_t lptmrConfig;

    LPTMR_GetDefaultConfig(&lptmrConfig);

    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(adcSamplePeriodus, TIMER_SOURCE_CLOCK));

    LPTMR_StartTimer(LPTMR0);

    /* Configure SIM for ADC hw trigger source selection */
    SIM->SOPT7 |= 0x0000008EU;
#else

    pit_config_t pitConfig;

    PIT_GetDefaultConfig(&pitConfig);

    PIT_Init(PIT, &pitConfig);

    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(adcSamplePeriodus, TIMER_SOURCE_CLOCK));

    PIT_StartTimer(PIT, kPIT_Chnl_0);

    /** Configure SIM for ADC hw trigger source selection */
    SIM->SOPT7 |= 0x00000084U;
#endif
}

/*************************************************************************************************/
void vEcgAcqTaskInit(void)
{
    memset(&ECGpack, 0, sizeof(sECGMembers_t));

    ECGACQ_InitPins();

    _vEcgACQTaskFilterInit();
    _vEcgACQTaskADCInit();

    xTaskCreate(vEcgProcessDataTask, "vEcgProcessDataTask",
    configMINIMAL_STACK_SIZE + 1000,
    NULL, EcgProcessTask_PRIORITY, &ECGpack.Tasks.xProcessDataTask);

    configASSERT(ECGpack.Tasks.xProcessDataTask != NULL);

    xTaskCreate(vEcgPrintHR, "vEcgPrintHR",
    configMINIMAL_STACK_SIZE + 250,
    NULL, EcgPrintHRTask_PRIORITY, &ECGpack.Tasks.xPrintHRTask);

    configASSERT(ECGpack.Tasks.xPrintHRTask != NULL);

    xTaskCreate(vEcgDrawOnGraphTask, "vEcgDrawOnGraphTask",
    configMINIMAL_STACK_SIZE + 300,
    NULL, EcgDrawOnGraphTask_PRIORITY, &ECGpack.Tasks.xDrawOnGraphTask);

    configASSERT(ECGpack.Tasks.xDrawOnGraphTask != NULL);
}

/*************************************************************************************************/
void vEcgAcqTaskResume(void)
{

    if (ECGpack.Tasks.xProcessDataTask)
        vTaskResume(ECGpack.Tasks.xProcessDataTask);
    if (ECGpack.Tasks.xPrintHRTask)
        vTaskResume(ECGpack.Tasks.xPrintHRTask);
    if (ECGpack.Tasks.xDrawOnGraphTask)
        vTaskResume(ECGpack.Tasks.xDrawOnGraphTask);
    _vEcgAcqTask_TurnOnECG();
    if (ECGpack.isADCInitialized)
    {
        ECGpack.countSamplesToInit = 0;
        PIT_StartTimer(PIT, kPIT_Chnl_0);
        EnableIRQ(GPIO_ECG_IN_ADC_IRQ);
    }
}

/*************************************************************************************************/
void vEcgAcqTaskSuspend(void)
{
//    vLogTask_StopLog();
    if (ECGpack.isADCInitialized)
    {
        ECGpack.countSamplesToInit = 0;
        DisableIRQ(GPIO_ECG_IN_ADC_IRQ);
        __DSB();
        __ISB();
        PIT_StopTimer(PIT, kPIT_Chnl_0);
    }
    _vEcgAcqTask_TurnOffECG();
    if (ECGpack.Tasks.xProcessDataTask)
        vTaskSuspend(ECGpack.Tasks.xProcessDataTask);
    if (ECGpack.Tasks.xPrintHRTask)
        vTaskSuspend(ECGpack.Tasks.xPrintHRTask);
    if (ECGpack.Tasks.xDrawOnGraphTask)
        vTaskSuspend(ECGpack.Tasks.xDrawOnGraphTask);
}

/*************************************************************************************************/
static void _vEcgAcqTask_TurnOnECG(void)
{
    GPIO_SetPinsOutput(GPIO_ECG_POWER_EN_GPIO, 1U << GPIO_ECG_POWER_EN_PIN);
    vTaskManager_Delay(1);
}

/*************************************************************************************************/
static void _vEcgAcqTask_TurnOffECG(void)
{
    GPIO_ClearPinsOutput(GPIO_ECG_POWER_EN_GPIO, 1U << GPIO_ECG_POWER_EN_PIN);
    vTaskManager_Delay(1);
}
/*************************************************************************************************/
