#ifndef SENSOR_H
#define SENSOR_H

//includes
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "arm_math.h"

//public prototypes
bool bQRSDetector_Detect(int32_t value);
int32_t i32QRSDetecto_calculatePulse(float32_t samplingPeriod);
int32_t i32QRSDetecto_getRRInterval(float32_t samplingPeriod);
void vQRSDetecto_showStatus(void);
void vQRSDetector_resetDetection(void);
#endif
