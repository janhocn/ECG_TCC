/**
 ******************************************************************************
 * @file    TouchScreen.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    7 de mar de 2018
 * @brief   This module conatins functions to manage touchscreen
 ******************************************************************************
 */

#include "TouchScreen.h"
#include "arm_math.h"
#include "AnalogTouch.h"
#include "fsl_port.h"
#include "MK64F12.h"
#include "pin_mux.h"
#include "prioAssigner.h"
#include <stdlib.h>
#include "TSC2007.h"

/*************************************************************************************************/
/**
 * @struct Maximum and minimum coordinate values from calibration
 */
typedef struct
{
    int Min; /** @< minimal value*/
    int Max; /** @< maximum value*/
} tMinMax;

/*************************************************************************************************/
/**
 * @struct Structure containning all the data necessary to work with the module
 */
static struct _sTouchScreenPack
{
#if USE_ANALOGTOUCH
    analogTouch_handle_t Atouch; /** @< Handler to the touch driver*/
#else
    tsc2007_handle_t tsc2007Pack; /** @< Handler to the touch driver i2c*/
#endif
    int32_t xPhys; /** @< x physical value in adc count*/
    int32_t yPhys; /** @< y physical value in adc count*/
    int32_t xMin; /** @< x minimal value*/
    int32_t xMax; /** @< x maximum value*/
    int32_t yMin; /** @< y minimal value*/
    int32_t yMax; /** @< y maximum value*/
    tMinMax xyMinMax[2]; /** @< Maximum and minimum coordinate values from calibration*/
} TouchScreenPack;

/*************************************************************************************************/
/**
 * @brief Converts the ADC value to X Logical position
 * @param adx x adc value
 * @return X logical position
 */
static int32_t _i32TouchScreen_AD2X(int32_t adx);

/*************************************************************************************************/
/**
 * @brief Converts the ADC value to Y Logical position
 * @param ady y adc value
 * @return Y logical position
 */
static int32_t _i32TouchScreen_AD2Y(int32_t ady);

/*************************************************************************************************/
/**
 * @brief Converts a logical position (pixels) to physical position (ADC) -> Calibration curve
 * @param l maximum possible logical position
 * @param l0 Initial Logical position
 * @param l1 Final logical position
 * @param p0 Initial physical position
 * @param p1 Final physical position
 * @return Converted position
 */
static int32_t _i32TouchScreen_Log2Phys(int32_t l, int32_t l0, int32_t l1, int32_t p0, int32_t p1);

/*************************************************************************************************/
/**
 * @brief Used to store the touch state for the emWin touch API
 * @param x x position
 * @param y y position
 * @return void
 */
static void _i32TouchScreen_StoreUnstable(int32_t x, int32_t y);

#if !USE_ANALOGTOUCH
/*************************************************************************************************/
/**
 * @brief Handles the i2c event signal
 * @param event Signal comming from i2c CMSIS IRQ handler
 * @return void
 */
static void _vTouchScreen_I2CSignalEvent(uint32_t event);
#endif
/*************************************************************************************************/
static int32_t _i32TouchScreen_AD2X(int32_t adx)
{
    int32_t r = adx - TouchScreenPack.xyMinMax[GUI_COORD_X].Min;
    r *= GUI_TOUCH_XSIZE - 1;
    return r / (TouchScreenPack.xyMinMax[GUI_COORD_X].Max - TouchScreenPack.xyMinMax[GUI_COORD_X].Min);
}

/*************************************************************************************************/
static int32_t _i32TouchScreen_AD2Y(int32_t ady)
{
    int32_t r = ady - TouchScreenPack.xyMinMax[GUI_COORD_Y].Min;
    r *= GUI_TOUCH_YSIZE - 1;
    return r / (TouchScreenPack.xyMinMax[GUI_COORD_Y].Max - TouchScreenPack.xyMinMax[GUI_COORD_Y].Min);
}

/*************************************************************************************************/
static int32_t _i32TouchScreen_Log2Phys(int32_t l, int32_t l0, int32_t l1, int32_t p0, int32_t p1)
{
    return p0 + ((p1 - p0) * (l - l0)) / (l1 - l0);
}

/*************************************************************************************************/
static void _i32TouchScreen_StoreUnstable(int32_t x, int32_t y)
{
    static int32_t _xLast = -1;
    static int32_t _yLast = -1;
    int xOut, yOut;

    if ((x != -1) && (y != -1) && (_xLast != -1) && (_yLast != -1))
    {
        xOut = _xLast;
        yOut = _yLast;
    }
    else
    {
        xOut = -1;
        yOut = -1;
    }
    _xLast = x;
    _yLast = y;
    GUI_TOUCH_StoreUnstable(xOut, yOut);
}

/*************************************************************************************************/
int32_t i32TouchScreen_GetxPhys(void)
{
    return TouchScreenPack.xPhys;
}

/*************************************************************************************************/
int32_t i32TouchScreen_GetyPhys(void)
{
    return TouchScreenPack.yPhys;
}

/*************************************************************************************************/
void vTouchScreen_SetDefaultCalibration(void)
{
    TouchScreenPack.xyMinMax[GUI_COORD_X].Min = GUI_TOUCH_AD_LEFT;
    TouchScreenPack.xyMinMax[GUI_COORD_X].Max = GUI_TOUCH_AD_RIGHT;
    TouchScreenPack.xyMinMax[GUI_COORD_Y].Min = GUI_TOUCH_AD_TOP;
    TouchScreenPack.xyMinMax[GUI_COORD_Y].Max = GUI_TOUCH_AD_BOTTOM;
}

/*************************************************************************************************/
bool bTouchScreen_Calibrate(int32_t Coord, int32_t Log0, int32_t Log1, int32_t Phys0, int32_t Phys1)
{
    int32_t l0 = 0;
    int32_t l1 = (Coord == GUI_COORD_X) ? GUI_TOUCH_XSIZE - 1 : GUI_TOUCH_YSIZE - 1;
    if (labs(Phys0 - Phys1) < 20)
    {
        return 1;
    }
    if (labs(Log0 - Log1) < 20)
    {
        return 1;
    }
    TouchScreenPack.xyMinMax[Coord].Min = _i32TouchScreen_Log2Phys(l0, Log0, Log1, Phys0, Phys1);
    TouchScreenPack.xyMinMax[Coord].Max = _i32TouchScreen_Log2Phys(l1, Log0, Log1, Phys0, Phys1);
    return 0;
}

/*************************************************************************************************/
void vTouchScreen_GetCalData(int32_t Coord, int32_t* pMin, int32_t* pMax)
{
    *pMin = TouchScreenPack.xyMinMax[Coord].Min;
    *pMax = TouchScreenPack.xyMinMax[Coord].Max;
}

/*************************************************************************************************/
void vTouchScreen_Exec(void)
{
#if USE_ANALOGTOUCH
    static uint8_t ReadState;
    int x, y;
    bool Pressed;
    /* calculate Min / Max values */
    if (TouchScreenPack.xyMinMax[GUI_COORD_X].Min < TouchScreenPack.xyMinMax[GUI_COORD_X].Max)
    {
        TouchScreenPack.xMin = TouchScreenPack.xyMinMax[GUI_COORD_X].Min;
        TouchScreenPack.xMax = TouchScreenPack.xyMinMax[GUI_COORD_X].Max;
    }
    else
    {
        TouchScreenPack.xMax = TouchScreenPack.xyMinMax[GUI_COORD_X].Min;
        TouchScreenPack.xMin = TouchScreenPack.xyMinMax[GUI_COORD_X].Max;
    }
    if (TouchScreenPack.xyMinMax[GUI_COORD_Y].Min < TouchScreenPack.xyMinMax[GUI_COORD_Y].Max)
    {
        TouchScreenPack.yMin = TouchScreenPack.xyMinMax[GUI_COORD_Y].Min;
        TouchScreenPack.yMax = TouchScreenPack.xyMinMax[GUI_COORD_Y].Max;
    }
    else
    {
        TouchScreenPack.yMax = TouchScreenPack.xyMinMax[GUI_COORD_Y].Min;
        TouchScreenPack.yMin = TouchScreenPack.xyMinMax[GUI_COORD_Y].Max;
    }

    /* Execute the state machine which reads the touch */
    switch (ReadState)
    {
        case 0:
        TouchScreenPack.yPhys = ui16AnalogTouch_MeasureY(&TouchScreenPack.Atouch);
        vAnalogTouch_ActivateY(&TouchScreenPack.Atouch); /* Prepare X- measurement */
        ReadState++;
        break;
        default:
        TouchScreenPack.xPhys = ui16AnalogTouch_MeasureX(&TouchScreenPack.Atouch);
        vAnalogTouch_ActivateX(&TouchScreenPack.Atouch); /* Prepare Y- measurement */
        /* Convert values into logical values */
        x = TouchScreenPack.xPhys;
        y = TouchScreenPack.yPhys;

        Pressed = bAnalogTouch_isPressed(&TouchScreenPack.Atouch);
        if ((x < TouchScreenPack.xMin) || (x > TouchScreenPack.xMax) || (y < TouchScreenPack.yMin) || (y > TouchScreenPack.yMax) || !Pressed)
        {
            _i32TouchScreen_StoreUnstable(-1, -1);
        }
        else
        {
            x = _i32TouchScreen_AD2X(x);
            y = _i32TouchScreen_AD2Y(y);
            _i32TouchScreen_StoreUnstable(x, y);
        }
        /* Reset state machine */
        ReadState = 0;
        break;
    }
#else
    tsc2007_data_set_t TSCdataSet;
    volatile int32_t x = -1;
    volatile int32_t y = -1;
    volatile float32_t rt = -1;

    if (bTSC2007_WaitTouchDetected(&TouchScreenPack.tsc2007Pack, 20))
    {
        /* calculate Min / Max values */
        if (TouchScreenPack.xyMinMax[GUI_COORD_X].Min < TouchScreenPack.xyMinMax[GUI_COORD_X].Max)
        {
            TouchScreenPack.xMin = TouchScreenPack.xyMinMax[GUI_COORD_X].Min;
            TouchScreenPack.xMax = TouchScreenPack.xyMinMax[GUI_COORD_X].Max;
        }
        else
        {
            TouchScreenPack.xMax = TouchScreenPack.xyMinMax[GUI_COORD_X].Min;
            TouchScreenPack.xMin = TouchScreenPack.xyMinMax[GUI_COORD_X].Max;
        }
        if (TouchScreenPack.xyMinMax[GUI_COORD_Y].Min < TouchScreenPack.xyMinMax[GUI_COORD_Y].Max)
        {
            TouchScreenPack.yMin = TouchScreenPack.xyMinMax[GUI_COORD_Y].Min;
            TouchScreenPack.yMax = TouchScreenPack.xyMinMax[GUI_COORD_Y].Max;
        }
        else
        {
            TouchScreenPack.yMax = TouchScreenPack.xyMinMax[GUI_COORD_Y].Min;
            TouchScreenPack.yMin = TouchScreenPack.xyMinMax[GUI_COORD_Y].Max;
        }

        vTSC2007_ClearPENStatus(&TouchScreenPack.tsc2007Pack);
        if (sTSC2007_readValues(&TouchScreenPack.tsc2007Pack, &TSCdataSet) == kStatus_Success)
        {
            x = _i32TouchScreen_AD2X((int32_t) TSCdataSet.values.Y);
            y = _i32TouchScreen_AD2Y((int32_t) TSCdataSet.values.X);

            rt = (float32_t) (TOUCH_PLATE_RESISTANCE);
            rt *= ((float32_t) (TSCdataSet.values.X / 4096.0));
            rt *= ((float32_t) (TSCdataSet.values.Z2 / (float32_t) TSCdataSet.values.Z1) - 1.0);
        }

        if (x > MAX_12BIT || y > MAX_12BIT || rt > 1300.0 || rt == -1)
        {
            x = -1;
            y = -1;
        }

        PORT_SetPinInterruptConfig(GPIO_TOUCH_PEN_IRQ_PORT, GPIO_TOUCH_PEN_IRQ_PIN, kPORT_InterruptFallingEdge);
    }
    _i32TouchScreen_StoreUnstable(x, y);
#endif
}

#if USE_ANALOGTOUCH
/*************************************************************************************************/
void vTouchScreen_ADCIRQHandler(void)
{
    vAnalogTouch_ADCIRQHandler(&TouchScreenPack.Atouch);
}
#else
/*************************************************************************************************/
static void _vTouchScreen_I2CSignalEvent(uint32_t event)
{
    vTSC2007_EventHandler(&TouchScreenPack.tsc2007Pack, event);
}

/*************************************************************************************************/
void vTouchScreen_PENIRQHandler(uint32_t status)
{
    if (status == (1 << GPIO_TOUCH_PEN_IRQ_PIN))
    {
        PORT_SetPinInterruptConfig(GPIO_TOUCH_PEN_IRQ_PORT, GPIO_TOUCH_PEN_IRQ_PIN, kPORT_InterruptOrDMADisabled);

        GPIO_ClearPinsInterruptFlags(GPIO_TOUCH_PEN_IRQ_GPIO, 1 << GPIO_TOUCH_PEN_IRQ_PIN);

        vTSC2007_PENIRQHandler(&TouchScreenPack.tsc2007Pack);
    }
}
#endif
#if USE_ANALOGTOUCH
/*************************************************************************************************/
void vTouchScreen_ActivateX(void)
{
    vAnalogTouch_ActivateX(&TouchScreenPack.Atouch);
}

/*************************************************************************************************/
void vTouchScreen_ActivateY(void)
{
    vAnalogTouch_ActivateY(&TouchScreenPack.Atouch);
}

/*************************************************************************************************/
uint16_t ui16TouchScreen_MeasureY(void)
{
    return ui16AnalogTouch_MeasureY(&TouchScreenPack.Atouch);
}

/*************************************************************************************************/
uint16_t ui16TouchScreen_MeasureX(void)
{
    return ui16AnalogTouch_MeasureX(&TouchScreenPack.Atouch);
}
#endif
/*************************************************************************************************/
void vTouchScreen_Init(void)
{
#if USE_ANALOGTOUCH
    TouchScreenPack.Atouch.xSize = GUI_TOUCH_XSIZE;
    TouchScreenPack.Atouch.ySize = GUI_TOUCH_YSIZE;
    TouchScreenPack.Atouch.pressureThreshold = 5;
    TouchScreenPack.Atouch.pressureThresholdMax = 3000;
    TouchScreenPack.Atouch._rxplate = 298;     //Measured with multimeter !!!

    TouchScreenPack.Atouch.Analog.channelNumberXm = GPIO_ATOUCH_ADC_XM_CHANNEL;
    TouchScreenPack.Atouch.Analog.channelNumberYp = GPIO_ATOUCH_ADC_YP_CHANNEL;
    TouchScreenPack.Atouch.Analog.adcChannelGroup = GPIO_ATOUCH_ADC_CHANNEL_GROUP;
    TouchScreenPack.Atouch.Analog.ADCBase = GPIO_ATOUCH_BASE_ADDR;
    TouchScreenPack.Atouch.Analog.resolution = 10;

    TouchScreenPack.Atouch.Digital.GPIOYp = GPIO_ATOUCH_YP_GPIO;
    TouchScreenPack.Atouch.Digital.pinYp = GPIO_ATOUCH_YP_PIN;
    TouchScreenPack.Atouch.Digital.GPIOXp = GPIO_ATOUCH_XP_GPIO;
    TouchScreenPack.Atouch.Digital.pinXp = GPIO_ATOUCH_XP_PIN;
    TouchScreenPack.Atouch.Digital.GPIOYm = GPIO_ATOUCH_YM_GPIO;
    TouchScreenPack.Atouch.Digital.pinYm = GPIO_ATOUCH_YM_PIN;
    TouchScreenPack.Atouch.Digital.GPIOXm = GPIO_ATOUCH_XM_GPIO;
    TouchScreenPack.Atouch.Digital.pinXm = GPIO_ATOUCH_XM_PIN;

    vTouchScreen_SetDefaultCalibration();//Start with default configuration

    EnableIRQ(GPIO_ATOUCH_ADC_IRQ);
    NVIC_SetPriority(GPIO_ATOUCH_ADC_IRQ, ADC_TOUCH_PRIO);
    vAnalogTouch_Configure(&TouchScreenPack.Atouch);
#else
    memcpy((void * restrict) &TouchScreenPack.tsc2007Pack.i2c_driver, (const void * restrict) &Driver_I2C0,
            sizeof(ARM_DRIVER_I2C));
    TouchScreenPack.tsc2007Pack.I2CAddress = TSC2007_I2C_ADDRESS;
    TouchScreenPack.tsc2007Pack.PENGpio = GPIO_TOUCH_PEN_IRQ_GPIO;
    TouchScreenPack.tsc2007Pack.PENPin = GPIO_TOUCH_PEN_IRQ_PIN;
    TouchScreenPack.tsc2007Pack.PENIRQn = GPIO_TOUCH_PEN_IRQn;
    TouchScreenPack.tsc2007Pack.PENIRQpriority = PORTA_INT_PRIO;
    TouchScreenPack.tsc2007Pack.DMAIRQpriority = TSC2007_DMA_PRIO;
    TouchScreenPack.tsc2007Pack.DMAIRQn = I2C0_IRQn;     //TSC2007_I2C_DMA_IRQn;
    TouchScreenPack.tsc2007Pack.plateResistance = TOUCH_PLATE_RESISTANCE;
    /****************Enable interrupt on PORTA***************************/
    EnableIRQ(GPIO_TOUCH_PEN_IRQn);
    /********************************************************************/

    vTouchScreen_SetDefaultCalibration();
    TSC2007_InitPins();
    sTSC2007_Init(&TouchScreenPack.tsc2007Pack, _vTouchScreen_I2CSignalEvent);
#endif
    bTouchScreen_Calibrate(GUI_COORD_Y, 0, LCD_GetYSize() - 1, GUI_TOUCH_AD_TOP, GUI_TOUCH_AD_BOTTOM);
    bTouchScreen_Calibrate(GUI_COORD_X, 0, LCD_GetXSize() - 1, GUI_TOUCH_AD_LEFT, GUI_TOUCH_AD_RIGHT);
}
/*************************************************************************************************/
