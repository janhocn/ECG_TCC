#include "qrsDetector.h"
#include "user.h"
#include "GUI.h"
#include "bufferPeak.h"
#include "peak.h"
#include "buffer.h"

int32_t SPKF = 0;
int32_t NPKF = 0;
uint32_t THRESHOLD1 = 0;
uint32_t THRESHOLD2 = 0;

int32_t RR_AVG1 = 0;
int32_t RR_AVG2 = 0;
int32_t RR_LOW = 0;
int32_t RR_HIGH = 0;
int32_t RR_MISS = -1;

int32_t SLOPEUP = 0;
int32_t RR = 0;
int32_t NUM_MISS = 0;

buff rawIn;

buffPeak peaks;
buffPeak rpeaks;

buff recentRR;
buff recentRR_OK;

Peak_t guiPeak;

static bool bresetDetection = true;

static int32_t __calculateRR(Peak_t peak);
static int32_t __calculateRRAVG1(void);
static int32_t __calculateRRAVG2(void);
static Peak_t __findPeakSearchback(int32_t * result);
static void __updateThresholds(void);
static bool __searchBack(void);
static void __updateNewRPeak(Peak_t peak);
static bool __foundRPeak(Peak_t peak);
static void __updateNoRPeak(Peak_t peak);
static void __updateGUI(Peak_t peak);

bool bQRSDetector_Detect(int32_t value)
{
    bool RRdetected = false;
    static uint32_t time = 0;
    static uint8_t counter = 0;
    int32_t current = 0;
    int32_t last = 0;

    if (bresetDetection)
    {
        bresetDetection = false;
        counter = 0;

        SPKF = 18000;
        NPKF = 2500;
        THRESHOLD1 = 14000;
        THRESHOLD2 = 7200;

        RR_AVG1 = 0;
        RR_AVG2 = 0;
        RR_LOW = 0;
        RR_HIGH = 0;
        RR_MISS = -1;

        SLOPEUP = 0;
        RR = 0;
        NUM_MISS = 0;

        // Initialise buffer sizes
        initBuffer(15, &rawIn);
        memset(rawIn.data, 0, rawIn.size);

        // Initialise peaks buffer
        initBufferPeak(400, &peaks);
        memset(peaks.data, 0, peaks.size);

        initBufferPeak(14, &rpeaks);
        memset(rpeaks.data, 0, rpeaks.size);

        // Initialise RR buffers
        initBuffer(8, &recentRR);
        memset(recentRR.data, 0, recentRR.size);
        initBuffer(8, &recentRR_OK);
        memset(recentRR_OK.data, 0, recentRR_OK.size);
    }

    insertToBuffer(value, &rawIn);

    if (counter < 2)
    {
        counter++;
    }
    else
    {
        current = getHeadBuffer(&rawIn);
        last = getPreviousBuffer(1, &rawIn);

        if (current < last && SLOPEUP)
        {
            // Maxima found
            Peak_t peak = { last, time - 1 };
            insertToBufferPeak(peak, &peaks);
            if (peak.value > THRESHOLD1)
            {
                // R peak found!
                RRdetected = __foundRPeak(peak);
            }
            else
            {
                // No R Peak found!
                __updateNoRPeak(peak);
            }

            SLOPEUP = 0;
        }
        else if (current > last)
        {
            SLOPEUP = 1;
        }
    }
    time++;

/*    if (time >= 20000)
    {
        cleanupBuffer(&rawIn);
        cleanupBufferPeak(&peaks);
        cleanupBufferPeak(&rpeaks);

        cleanupBuffer(&recentRR);
        cleanupBuffer(&recentRR_OK);

        bresetDetection = true;
    }*/
    return RRdetected;
}

void vQRSDetector_resetDetection(void)
{
    bresetDetection = true;

    cleanupBuffer(&rawIn);
    cleanupBufferPeak(&peaks);
    cleanupBufferPeak(&rpeaks);

    cleanupBuffer(&recentRR);
    cleanupBuffer(&recentRR_OK);
}

static void __updateThresholds(void)
{
    THRESHOLD1 = NPKF + (SPKF - NPKF) / 4;
    THRESHOLD2 = THRESHOLD1 / 2;
}

static void __updateLowHighMiss(int32_t AVG)
{
    RR_LOW = 92 * AVG / 100;
    RR_HIGH = 116 * AVG / 100;
    RR_MISS = 166 * AVG / 100;
}

static int32_t __calculateRRAVG1(void)
{
    return getAvgBuffer(&recentRR);
}

static int32_t __calculateRRAVG2(void)
{
    return getAvgBuffer(&recentRR_OK);
}

static void __updateNewRPeak(Peak_t peak)
{
    insertToBufferPeak(peak, &rpeaks);
    insertToBuffer(RR, &recentRR);
    insertToBuffer(RR, &recentRR_OK);
    SPKF = peak.value / 8 + 7 * SPKF / 8;
    RR_AVG1 = __calculateRRAVG1();
    RR_AVG2 = __calculateRRAVG2();
    __updateLowHighMiss(RR_AVG2);
    __updateThresholds();

    __updateGUI(peak);
}

static Peak_t __findPeakSearchback(int32_t * result)
{
    uint32_t i;
    for (i = 1; i < peaks.size; i++)
    {
        Peak_t peak = getPreviousPeak(i, &peaks);
        if (peak.value > THRESHOLD2)
        {
            RR = __calculateRR(peak);
            *result = 1;
            return peak;
        }
    }

    *result = 0;
    Peak_t peak = { 0, 0 };
    return peak;
}

static bool __searchBack(void)
{
    int32_t result;
    Peak_t peak = __findPeakSearchback(&result);

    if (!result)
    {
        return false;
    }

    insertToBufferPeak(peak, &rpeaks);
    SPKF = peak.value / 4 + 3 * SPKF / 4;
    insertToBuffer(RR, &recentRR);
    RR_AVG1 = __calculateRRAVG1();
    __updateLowHighMiss(RR_AVG1);
    __updateThresholds();
    __updateGUI(peak);
    return true;

}

static int32_t __calculateRR(Peak_t peak)
{
    Peak_t last = getHeadPeak(&rpeaks);
    if (last.time == 0)
        return 0;
    return peak.time - last.time;
}

static bool __foundRPeak(Peak_t peak)
{
    RR = __calculateRR(peak);

    if (RR > RR_LOW && RR < RR_HIGH)
    {
        NUM_MISS = 0;
        __updateNewRPeak(peak);
        return true;
    }
    else
    {
        NUM_MISS++;
        if (RR > RR_MISS)
        {
            return __searchBack();
        }
        return false;
    }
}

static void __updateNoRPeak(Peak_t peak)
{
    NPKF = (peak.value / 8) + (7 * NPKF / 8);
    __updateThresholds();
}

static void __updateGUI(Peak_t peak)
{
    guiPeak.time = peak.time;
    guiPeak.value = peak.value;
}

void vQRSDetecto_showStatus(void)
{
    //    printf("RPeak - time = %d, value = %d\n", guiPeak.time, guiPeak.value);
    if (guiPeak.value < 2000)
    {
        GUI_DispStringAt("WARNING! RPeak value is too low\n", 0, 0);
    }
    if (NUM_MISS >= 5)
    {
        GUI_DispStringAt("WARNING! Five or more RPeaks missed RR intervals\n", 0, 0);
    }
    //    printf("Pulse is now: %d\n", calculatePulse());
}

int32_t i32QRSDetecto_calculatePulse(float32_t samplingPeriod)
{
    if (RR_AVG1 == 0)
        return 0;
    return 60 / (RR_AVG1 * samplingPeriod);
}

int32_t i32QRSDetecto_getRRInterval(float32_t samplingPeriod)
{
    return (RR_AVG1 * samplingPeriod);
}
