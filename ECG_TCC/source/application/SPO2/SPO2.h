/**
 ******************************************************************************
 * @file    SPO2.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 November 2017
 * @brief   SPO2 Application module
 ******************************************************************************
 */

#ifndef APPLICATION_SPO2_SPO2_H_
#define APPLICATION_SPO2_SPO2_H_

#include "AFE4400.h"

typedef enum _eSPO2PowerControl {
    _E_POWER_ON,
    _E_POWER_OFF
}eSPO2PowerControl_t;
/*************************************************************************************************/
/**
 * @brief Initialize the SPO2 driver
 * @return void
 */
void vSPO2_Init(void);

/*************************************************************************************************/
/**
 * @brief Read the RAW Red LED ADC
 * @param readValue Value read
 * @return kStatus_Succes or kStatus_Fail
 */
status_t sSPO2_ReadRedLED(uint32_t * readValue);

/*************************************************************************************************/
/**
 * @brief Read the RAW IR LED ADC
 * @param readValue Value read
 * @return kStatus_Succes or kStatus_Fail
 */
status_t sSPO2_ReadIRLED(uint32_t * readValue);

/*************************************************************************************************/
/**
 * @brief Controls the power state of the IC
 * @param _pwc _E_POWER_ON or _E_POWER_OFF
 * @return kStatus_Succes or kStatus_Fail
 */
status_t sSPO2_PowerControl(eSPO2PowerControl_t _pwc);
/*************************************************************************************************/
#endif /* APPLICATION_SPO2_SPO2_H_ */
