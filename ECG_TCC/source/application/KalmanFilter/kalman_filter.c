/*
 * kalman_filter.c
 *
 *  Created on: Feb 13, 2018
 *      Author: Diego
 */

#include "kalman_filter.h"

void kalman_init(kalman_state* state, float32_t q, float32_t r, float32_t p, float32_t intial_value)
{
    state->q = q;
    state->r = r;
    state->p = p;
    state->x = intial_value;
}

void kalman_update(kalman_state* state, float32_t measurement)
{
    //prediction update
    state->p = state->p + state->q;
    //measurement update
    state->k = state->p / (state->p + state->r);
    state->x = state->x + state->k * (measurement - state->x);
    state->p = (1 - state->k) * state->p;
}
