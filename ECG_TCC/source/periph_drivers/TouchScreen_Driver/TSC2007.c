/**
 ******************************************************************************
 * @file    TSC2007.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 March 2018
 * @brief   Description: Driver to controle the TSC2007 touch controller driver.
 ******************************************************************************
 */

#include "fsl_device_registers.h"
#include "fsl_i2c_cmsis.h"
#include "fsl_i2c_edma.h"
#include "fsl_i2c.h"
#include "fsl_common.h"
#include "user.h"
#include "InterruptManager.h"
#include "Driver_I2C.h"
#include "TSC2007_Defs.h"
#include "TSC2007.h"
#include "prioAssigner.h"
#include "SystickUser.h"
#include "task_manager.h"

/*************************************************************************************************/
/**
 * @union Union to read a register from the TS2007.
 */
typedef union _tsc2007_register {
	uint16_t value; /** @< 16bit value*/
	struct {
		uint8_t low_byte; /** @< LSB part*/
		uint8_t high_byte; /** @< MSB part*/
	} bytes;
} tsc2007_register_t;

/*************************************************************************************************/
/**
 @brief Swaps two bytes of a 16bit word

 @param high_byte pointer to the MSB
 @param low_byte pointer to the LSB
 @return void
 */
static void _swap(uint8_t *high_byte, uint8_t *low_byte);

/*************************************************************************************************/
/**
 @brief Wait Master Complete Event on I2C

 @param handle TSC2007 driver handle to DSPI, pins and signals related parameters
 @return I2C event
 */
static uint32_t _u32TSC2007_WaitEvent(tsc2007_handle_t *handle);

/*************************************************************************************************/
/**
 @brief Reads data from a specific register.
 @param handle TSC2007 driver handle to DSPI, pins and signals related parameters
 @param cmd register to read.
 @param dataRead pointer to the data read.
 @return kStatus_Success if communication was successful.
 */
static status_t _sTSC2007_readTouchData(tsc2007_handle_t* handle, uint8_t cmd,
		uint16_t *dataRead);

/*************************************************************************************************/
status_t sTSC2007_Init(tsc2007_handle_t* handle, ARM_I2C_SignalEvent_t cb_event) {
	if (!handle) {
		return kStatus_InvalidArgument;
	}

	handle->xI2CSemaSignal = xTaskManager_SemaphoreCreate();
	configASSERT(handle->xI2CSemaSignal != NULL);

	handle->xPENSemaSignal = xTaskManager_SemaphoreCreate();
	configASSERT(handle->xPENSemaSignal != NULL);

    NVIC_SetPriority(handle->DMAIRQn, handle->DMAIRQpriority);
    NVIC_SetPriority(handle->PENIRQn, handle->PENIRQpriority);


	handle->i2c_driver.Initialize(cb_event);
	handle->i2c_driver.PowerControl(ARM_POWER_FULL);
	handle->i2c_driver.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST_PLUS);

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	if (sTSC2007_WriteByte(handle, PWRDOWN) != ARM_I2C_EVENT_TRANSFER_DONE) {
		return kStatus_Fail;
	}
	return kStatus_Success;
}

/*************************************************************************************************/
status_t sTSC2007_Deinit(tsc2007_handle_t* handle) {
	if (!handle) {
		return kStatus_InvalidArgument;
	}

	handle->i2c_driver.PowerControl(ARM_POWER_OFF);
	handle->i2c_driver.Uninitialize();

	return kStatus_Success;

}
/*************************************************************************************************/
void vTSC2007_PENIRQHandler(tsc2007_handle_t* handle) {
	if (!handle) {
		return;
	}

	handle->penDetected = true;

	xTaskManager_SemaphoreGive(handle->xPENSemaSignal);
}

/*************************************************************************************************/
bool bTSC2007_isTouchDetected(tsc2007_handle_t* handle) {
	if (!handle) {
		return false;
	}
	return handle->penDetected;
}

/*************************************************************************************************/
void vTSC2007_ClearPENStatus(tsc2007_handle_t* handle) {
	if (!handle) {
		return;
	}
	handle->penDetected = false;
}

/*************************************************************************************************/
bool bTSC2007_WaitTouchDetected(tsc2007_handle_t* handle, uint32_t timeout) {
	if (!handle) {
		return false;
	}

	if (xTaskManager_SemaphoreTake(handle->xPENSemaSignal, timeout) == pdTRUE) {
		return true;
	} else {
		return false;
	}
}

/*************************************************************************************************/
bool bTSC2007_getPENStatus(tsc2007_handle_t* handle) {
	if (!handle) {
		return true;
	}

	return (bool) GPIO_ReadPinInput(handle->PENGpio, handle->PENPin);
}

/*************************************************************************************************/
static status_t _sTSC2007_readTouchData(tsc2007_handle_t* handle, uint8_t cmd,
		uint16_t *dataRead) {
	tsc2007_register_t reg;
	if (!handle) {
		return kStatus_InvalidArgument;
	}

	if (sTSC2007_WriteByte(handle, cmd) != ARM_DRIVER_OK) {
		return kStatus_Fail;
	}

	if (sTSC2007_ReadMultiByte(handle, (uint8_t *) &reg.value,
			2) != ARM_DRIVER_OK) {
		return kStatus_Fail;
	}

	_swap(&reg.bytes.high_byte, &reg.bytes.low_byte);

	*dataRead = (reg.value >> 4);

	return kStatus_Success;
}

/*************************************************************************************************/
status_t sTSC2007_ReadMultiByte(tsc2007_handle_t* handle, uint8_t *data,
		uint8_t length) {

	status_t status = kStatus_Success;
	if (!handle) {
		return kStatus_InvalidArgument;
	}

	if (handle->i2c_driver.MasterReceive(handle->I2CAddress, data, length,
	false) != ARM_DRIVER_OK) {
		status = kStatus_Fail;
	} else if (_u32TSC2007_WaitEvent(handle) != ARM_I2C_EVENT_TRANSFER_DONE) {
		status = kStatus_Fail;
	}
	return status;
}

/*************************************************************************************************/
status_t sTSC2007_WriteByte(tsc2007_handle_t* handle, uint8_t value) {

	status_t status = kStatus_Success;
	uint8_t valueToSend = value;
	if (!handle) {
		return kStatus_InvalidArgument;
	}

	if (handle->i2c_driver.MasterTransmit(handle->I2CAddress, &valueToSend, 1,
	false) != ARM_DRIVER_OK) {
		status = kStatus_Fail;
	} else if (_u32TSC2007_WaitEvent(handle) != ARM_I2C_EVENT_TRANSFER_DONE) {
		status = kStatus_Fail;
	}
	return status;
}

/*************************************************************************************************/
void vTSC2007_EventHandler(tsc2007_handle_t *handle, uint32_t i2c_event) {
	if (!handle) {
		return;
	}

	handle->i2c_event = i2c_event;
	handle->i2c_event_received = true;
	xTaskManager_SemaphoreGive(handle->xI2CSemaSignal);
}

/*************************************************************************************************/
status_t sTSC2007_readValues(tsc2007_handle_t *handle,
		tsc2007_data_set_t *dataSet) {

	status_t status = kStatus_Success;
	if (!handle) {
		return kStatus_InvalidArgument;
	}

	/* y- still on; turn on only y+ (and ADC) */
	if (_sTSC2007_readTouchData(handle, READ_Y,
			&(dataSet->values.Y))!= kStatus_Success) {
		status = kStatus_Fail;
	}

	/* turn y- off, x+ on, then leave in lowpower */
	if (_sTSC2007_readTouchData(handle, READ_X,
			&(dataSet->values.X)) != kStatus_Success) {
		status = kStatus_Fail;
	}

	/* turn y+ off, x- on; we'll use formula #1 */
	if (_sTSC2007_readTouchData(handle, READ_Z1,
			&(dataSet->values.Z1))!= kStatus_Success) {
		status = kStatus_Fail;
	}

	if (_sTSC2007_readTouchData(handle, READ_Z2,
			&(dataSet->values.Z2))!= kStatus_Success) {
		status = kStatus_Fail;
	}

	/* Prepare for next touch reading - power down ADC, enable PENIRQ */
	if (sTSC2007_WriteByte(handle, PWRDOWN) != kStatus_Success) {
		status = kStatus_Fail;
	}

	return status;
}

/*************************************************************************************************/
static uint32_t _u32TSC2007_WaitEvent(tsc2007_handle_t *handle) {
	if (!handle) {
		return 0xFF;
	}

	xTaskManager_SemaphoreTake(handle->xI2CSemaSignal, portMAX_DELAY);

	handle->i2c_event_received = false;

	return handle->i2c_event;
}

/*************************************************************************************************/
static void _swap(uint8_t *high_byte, uint8_t *low_byte) {
	uint8_t temp;

	temp = *high_byte;
	*high_byte = *low_byte;
	*low_byte = temp;
}
/*************************************************************************************************/
