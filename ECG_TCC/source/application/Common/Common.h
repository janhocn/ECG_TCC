/**
 ******************************************************************************
 * @file    Common.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    6 March 2018
 * @brief   This file contains general functions used by other modules
 ******************************************************************************
 */
#ifndef APPLICATION_COMMON_COMMON_H_
#define APPLICATION_COMMON_COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary for the exponential average filter
 */
typedef struct _expAvgFilter_t
{
    float32_t prevAvg; /** @< Previous average value */
    float32_t w; /** @< Filter Coefficient */
} expAvgFilter_t;

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary for the mean filter
 */
typedef struct _meanDiffFilter_t
{
    float32_t* values; /** @< Buffer values */
    uint8_t index; /** @< Index to count the values */
    float32_t sum; /** @< Sum of the buffer */
    uint8_t count; /** @< Count the number of values in the buffer to perform the average */
    uint16_t size; /** @< Size of the filter */
    float32_t mean; /** @< Mean Result */
} meanDiffFilter_t;

/*************************************************************************************************/
/**
 * @brief Initializes a exponential average filter
 * @param expS Handler to the filter structure
 * @param w Filter Coefficient
 * @param initialAvg Choose an initial value for the calculations (can be 0)
 * @return void
 */
void vCommon_ExpAvgInit(expAvgFilter_t* expS, float32_t w, float32_t initialAvg);

/*************************************************************************************************/
/**
 * @brief Perform a exponential average filter
 * @param expS Handler to the filter structure
 * @param sample Sample to be filtered
 * @return Filtered Value
 */
float32_t fCommon_ExpAvg(expAvgFilter_t* expS, float32_t sample);

/*************************************************************************************************/
/**
 * @brief Initializes a mean median filter
 * @param filterValues Handler to the filter structure
 * @param size Size of the filter buffer
 * @return mean Value
 */
bool bCommon_initMeanDiff(meanDiffFilter_t* filterValues, uint16_t size);

/*************************************************************************************************/
/**
 * @brief Deinitializes a mean median filter
 * @param filterValues Handler to the filter structure
 * @return mean Value
 */
void vCommon_deinitMeanDiff(meanDiffFilter_t* filterValues);
/*************************************************************************************************/
/**
 * @brief Perform a mean median filter
 * @attention For more information access https://morf.lv/implementing-pulse-oximeter-using-max30100
 * @param filterValues Handler to the filter structure
 * @param M Value to be filtered
 * @return mean Value
 */
float32_t fCommon_meanDiff(meanDiffFilter_t* filterValues, float32_t M);

/*************************************************************************************************/
/**
 * @brief Converts the access mode w,r,a to FATFS pattern
 * Source: https://morf.lv/implementing-pulse-oximeter-using-max30100
 * w(t)=x(t)+ ∝ *w(t-1)
 * y(t)=w(t)-w(t-1)
 * y(t):       is the output of the filter
 * x(t):       current input/value
 * w(t):       intermediate value, acts like the history of the DC value
 * α:          is the response constant of the filter
 *                              If α = 1 then everything passes through
 *                              If α = 0 then nothing passes through
 *                              for DC removal you want the α as rather close to 1.
 * @attention For more information access http://sam-koblenski.blogspot.com.br/2015/11/everyday-dsp-for-programmers-dc-and.html
 * @param x Input Value
 * @param prev_w Previous DC value
 * @param alpha response constant of the filter
 * @param result Pointer to the filter output
 * @param w Pointer to the actual DC value
 * @return void
 */
void vCommon_dcRemoval(float32_t x, float32_t prev_w, float32_t alpha, float32_t *result, float32_t* w);

/*************************************************************************************************/

#endif /* APPLICATION_COMMON_COMMON_H_ */
