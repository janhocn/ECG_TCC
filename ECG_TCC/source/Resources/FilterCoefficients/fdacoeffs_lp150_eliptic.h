/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.2 and the Signal Processing Toolbox 7.4.
 * Generated on: 05-Jan-2018 23:17:58
 */

/*
 * Discrete-Time IIR Filter (real)
 * -------------------------------
 * Filter Structure    : Direct-Form II Transposed, Second-Order Sections
 * Number of Sections  : 6
 * Stable              : Yes
 * Linear Phase        : No
 */

/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2017a\extern\include\tmwtypes.h 
 */
#include "arm_math.h"
#define SECTIONS_ELIP150 6*2 //6 Sectors for filter coeffs and 6 more for input gains
const float32_t baEliptic150Coeffs[SECTIONS_ELIP150 * (5)] =
{
    0.9345233076, 0, 0,
    0, 0,

    1, -1.917627841982, 1,
    1.913233283881, -0.9864372242791,

    0.8821635195535, 0, 0,
    0, 0,

    1, -1.909446340679, 1,
    1.897196366044, -0.9657179280532,

    0.7462498572145, 0, 0,
    0, 0,

    1, -1.884840938356, 1,
    1.863692186586, -0.9219195491629,

    0.4998978749172, 0, 0,
    0, 0,

    1, -1.788605051052, 1,
    1.80499339146, -0.8450199383008,

    0.2401005149547, 0, 0,
    0, 0,

    1, -0.8721666027076, 1,
    1.74389835851, -0.7649477213705,

    0.01526775257032, 0, 0,
    0, 0,

    1, -1.920290492271, 1,
    1.921449829203, -0.9964564969362
};