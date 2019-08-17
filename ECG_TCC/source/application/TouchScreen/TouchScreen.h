/**
 ******************************************************************************
 * @file    TouchScreen.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    7 de mar de 2018
 * @brief   This module conatins functions to manage touchscreen
 ******************************************************************************
 */
#ifndef APPLICATION_TOUCHSCREEN_TOUCHSCREEN_H_
#define APPLICATION_TOUCHSCREEN_TOUCHSCREEN_H_

#include "GUI.h"
#include "LCDConf.h"
#include <stdint.h>
#include <stdbool.h>
#include "pin_mux.h"

#if USE_PROTOTYPE == 1
#define USE_ANALOGTOUCH 1
#else
#define USE_ANALOGTOUCH 0
#endif

#define TOUCH_PLATE_RESISTANCE 298

#if !USE_ANALOGTOUCH
/*************************************************************************************************/
/**
 * @def TSC2007 I2C Address
 */
#define TSC2007_I2C_ADDRESS         0x48

/*************************************************************************************************/
/**
 * @def DMA I2C TSC2007 Channel IRQn
 */
#define TSC2007_I2C_DMA_IRQn DMA9_IRQn

#define GUI_TOUCH_AD_TOP 3814
#define GUI_TOUCH_AD_BOTTOM 264
#define GUI_TOUCH_AD_LEFT 281
#define GUI_TOUCH_AD_RIGHT 3919

#else
#define TOUCH_ADC_RESOL 10

#if TOUCH_ADC_RESOL == 10
#define GUI_TOUCH_AD_TOP 184
#define GUI_TOUCH_AD_BOTTOM 678
#define GUI_TOUCH_AD_LEFT 845
#define GUI_TOUCH_AD_RIGHT 135
#else
#error "Initial calibration points must be defined for other ADC resoltions"
#endif

#endif

#ifndef GUI_TOUCH_YSIZE
#define GUI_TOUCH_YSIZE YSIZE_PHYS
#endif

#ifndef GUI_TOUCH_XSIZE
#define GUI_TOUCH_XSIZE XSIZE_PHYS
#endif

/*************************************************************************************************/
/**
 * @brief Initializes the module
 * @return void
 */
void vTouchScreen_Init(void);

/*************************************************************************************************/
/**
 * @brief Rerturns the current x position
 * @return x position
 */
int32_t i32TouchScreen_GetxPhys(void);

/*************************************************************************************************/
/**
 * @brief Rerturns the current y position
 * @return y position
 */
int32_t i32TouchScreen_GetyPhys(void);

/*************************************************************************************************/
/**
 * @brief Measure the ADC value on X
 * @return ADC Value
 */
uint16_t ui16TouchScreen_MeasureX(void);

/*************************************************************************************************/
/**
 * @brief Measure the ADC value on Y
 * @return ADC Value
 */
uint16_t ui16TouchScreen_MeasureY(void);

/*************************************************************************************************/
/**
 * @brief Prepare the X line to be read
 * @return void
 */
void vTouchScreen_ActivateX(void);

/*************************************************************************************************/
/**
 * @brief Prepare the Y line to be read
 * @return void
 */
void vTouchScreen_ActivateY(void);

/*************************************************************************************************/
/**
 * @brief ADC Interrupt Handler
 * @return void
 */
void vTouchScreen_ADCIRQHandler(void);

/*************************************************************************************************/
/**
 * @brief Touch update task (should be executed in a time basis)
 * @return void
 */
void vTouchScreen_Exec(void);

/*************************************************************************************************/
/**
 * @brief Calibrates the touchscreen with given points
 * @param Coor (GUI_COORD_Y or GUI_COORD_X)
 * @param l0 Initial Logical position
 * @param l1 Final logical position
 * @param p0 Initial physical position
 * @param p1 Final physical position
 * @return 0 if success, 1 otherwise
 */
bool bTouchScreen_Calibrate(int32_t Coord, int32_t Log0, int32_t Log1, int32_t Phys0, int32_t Phys1);

/*************************************************************************************************/
/**
 * @brief Sets the calibration to its default value (measured and hardcoded)
 * @return void
 */
void vTouchScreen_SetDefaultCalibration(void);

/*************************************************************************************************/
/**
 * @brief Get the current calibration data (min and max) of a coordinate
 * @param Coord (GUI_COORD_Y or GUI_COORD_X)
 * @param pMin Pointer to the minimum calibration value
 * @param pMax Pointer to the maximum calibration value
 * @return void
 */
void vTouchScreen_GetCalData(int32_t Coord, int32_t* pMin, int32_t* pMax);

#if !USE_ANALOGTOUCH
/*************************************************************************************************/
/**
 * @brief PENIRQ pin IRQ handler
 * @param status true if the interrupt is for this handler (this pin)
 * @return void
 */
void vTouchScreen_PENIRQHandler(uint32_t status);
/*************************************************************************************************/
#endif

#endif /* APPLICATION_TOUCHSCREEN_TOUCHSCREEN_H_ */
