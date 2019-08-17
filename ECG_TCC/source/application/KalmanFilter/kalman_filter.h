/*
 * kalman_filter.h
 *
 *  Created on: Feb 13, 2018
 *      Author: Diego
 */

#ifndef APPLICATION_KALMANFILTER_KALMAN_FILTER_H_
#define APPLICATION_KALMANFILTER_KALMAN_FILTER_H_

#include "arm_math.h"

typedef struct
{
    float32_t q;     //process noise covariance
    float32_t r;     //measurement noise covariance
    float32_t x;     //value
    float32_t p;     //estimation error covariance
    float32_t k;     //kalman gain
} kalman_state;

void kalman_init(kalman_state* state, float32_t q, float32_t r, float32_t p, float32_t intial_value);
void kalman_update(kalman_state* state, float32_t measurement);

#endif /* APPLICATION_KALMANFILTER_KALMAN_FILTER_H_ */
