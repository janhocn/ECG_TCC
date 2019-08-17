/**
 ******************************************************************************
 * @file    Common.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    6 March 2018
 * @brief   This file contains general functions used by other modules
 ******************************************************************************
 */

#include "Common.h"
#include <stdlib.h>
#include "FreeRTOS.h"
/*************************************************************************************************/
void vCommon_ExpAvgInit(expAvgFilter_t* expS, float32_t w, float32_t initialAvg)
{
    expS->w = w;
    expS->prevAvg = initialAvg;
}

/*************************************************************************************************/
float32_t fCommon_ExpAvg(expAvgFilter_t* expS, float32_t sample)
{
    expS->prevAvg = expS->w * sample + (1 - expS->w) * expS->prevAvg;
    return expS->prevAvg;
}

/*************************************************************************************************/
void vCommon_deinitMeanDiff(meanDiffFilter_t* filterValues)
{
    vPortFree(filterValues->values);
}

/*************************************************************************************************/
bool bCommon_initMeanDiff(meanDiffFilter_t* filterValues, uint16_t size)
{
    filterValues->values = (float32_t*) malloc(size * sizeof(float32_t));

    if (filterValues->values == NULL)
    {
        return false;
    }

    memset(filterValues->values,0,size * sizeof(float32_t));

    filterValues->size = size;
    filterValues->count = 0;
    filterValues->index = 0;
    filterValues->sum = 0;
    filterValues->mean = 0;
    return true;
}

/*************************************************************************************************/
float32_t fCommon_meanDiff(meanDiffFilter_t* filterValues, float32_t M)
{
    if (filterValues->values == NULL)
    {
        return 0.0;
    }

    filterValues->sum -= filterValues->values[filterValues->index];
    filterValues->values[filterValues->index] = M;
    filterValues->sum += filterValues->values[filterValues->index];

    filterValues->index++;
    filterValues->index = filterValues->index % filterValues->size;

    if (filterValues->count < filterValues->size)
        filterValues->count++;

    filterValues->mean = filterValues->sum / filterValues->count;

    filterValues->mean -= M;
    return  filterValues->mean;
}

/*************************************************************************************************/
void vCommon_dcRemoval(float32_t x, float32_t prev_w, float32_t alpha, float32_t *result, float32_t* w)
{
    *w = x + alpha * prev_w;
    *result = *w - prev_w;
}

/*************************************************************************************************/
