/**
 ******************************************************************************
 * @file    BatteryCharger.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    18 April 2018
 * @brief   File System Module
 ******************************************************************************
 */
#ifndef APPLICATION_BATTERYCHARGER_BATTERYCHARGER_H_
#define APPLICATION_BATTERYCHARGER_BATTERYCHARGER_H_

#include "arm_math.h"
#include "fsl_common.h"
#include "BQ27441.h"

typedef struct _BatteryChargerInfo
{
    uint16_t stateOfCharge;
    uint16_t voltage;
    int16_t current;
    int16_t power;
    int16_t health;
    uint16_t fullCapacity;
    uint16_t designCapacity;
    uint16_t capacity;
    int16_t temperature;
} BatteryChargerInfo_t;

status_t stBatteryCharger_Init(void);
void vBatteryCharger_UpdateInfo(BatteryChargerInfo_t* info);
void bBatteryCharger_UpdateFlags(flags_t* flags);
uint16_t u16BatteryCharger_getFlags(void);

void vBatteryCharger_GPOUT_IRQHandler(uint32_t status);
#endif /* APPLICATION_BATTERYCHARGER_BATTERYCHARGER_H_ */
