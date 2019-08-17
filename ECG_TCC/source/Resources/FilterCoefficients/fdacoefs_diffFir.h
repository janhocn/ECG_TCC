/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.2 and the Signal Processing Toolbox 7.4.
 * Generated on: 23-Jan-2018 19:49:17
 */

/*
 * Discrete-Time FIR Filter (real)
 * -------------------------------
 * Filter Structure  : Direct-Form FIR
 * Filter Length     : 201
 * Stable            : Yes
 * Linear Phase      : Yes (Type 3)
 */

/* General type conversion for MATLAB generated C-code  */
/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2017a\extern\include\tmwtypes.h 
 */

//int_c = (5-1)/(Fs*1/40);
//b = interp1(1:5,[1 2 0 -2 -1].*(1/8)*Fs,1:int_c:5);

#define TAPS_DIFF 5
const float32_t baDifferentiator_FirCoeffs[TAPS_DIFF] = {
        24.5098,   44.5098,   18.0392,  -21.9608,  -42.5490
};