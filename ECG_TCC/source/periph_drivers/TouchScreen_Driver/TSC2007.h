/**
 ******************************************************************************
 * @file    TSC2007.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 March 2018
 * @brief   Description: Driver to controle the TSC2007 touch controller driver.
 ******************************************************************************
 */
#ifndef PERIPH_DRIVERS_DRIVER_TSC2007_H_
#define PERIPH_DRIVERS_DRIVER_TSC2007_H_

#include "TSC2007_Defs.h"
#include "fsl_i2c_cmsis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task_manager.h"

/*************************************************************************************************/
/**
 * @union Union to read the touch data.
 */
typedef union _tsc2007_data_set{
	uint8_t buffer[8];  /** @< Raw buffer with the data read*/
	struct {
		uint16_t X; /** @< X value*/
		uint16_t Y; /** @< Y value*/
		uint16_t Z1; /** @< Z1 value*/
		uint16_t Z2; /** @< Z2 value*/
	} values;
} tsc2007_data_set_t;

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to configure and use this driver
 */
typedef struct _tsc2007_handle {
	ARM_DRIVER_I2C i2c_driver; /** @< I2C CMSIS Driver*/
	volatile uint32_t i2c_event; /** @< Hold the event coming from DMA*/
	volatile bool i2c_event_received; /** @< Signals event received*/
	volatile bool penDetected; /** @< Signals touch detected */
	IRQn_Type PENIRQn; /** @< PEN IRQ*/
	uint32_t PENIRQpriority; /** @< PEN IRQ priority */
	IRQn_Type DMAIRQn; /** @< DMA Channel I2C IRQ*/
	uint32_t DMAIRQpriority; /** @< DMA Channel I2C IRQ priority*/
	GPIO_Type *PENGpio; /** @< PEN GPIO*/
	uint16_t PENPin; /** @< PEN Pin*/
	uint8_t I2CAddress; /** @< I2C address*/
	uint16_t plateResistance; /** @< Touch plate resistance */
	SemaphoreHandle_t xI2CSemaSignal; /** @< Semaphore to control the singals coming from DMA*/
	SemaphoreHandle_t xPENSemaSignal; /** @< Semaphore to control the singals from the PEN IRQ*/
} tsc2007_handle_t;

/*************************************************************************************************/
/**
 @brief Initializes I2C and verifies communication with the TSC2007.
 @brief Must be called before using any other function.

 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @param cb_event callback to I2C signal event
 @return kStatus_Success if communication was successful.
 */
status_t sTSC2007_Init(tsc2007_handle_t* handle, ARM_I2C_SignalEvent_t cb_event);

/*************************************************************************************************/
/**
 @brief Deinitializes DSPI communication.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sTSC2007_Deinit(tsc2007_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Hamdle to the PEN IRQ.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
void vTSC2007_PENIRQHandler(tsc2007_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Waits the PEN IRQ to happen by a RTOS semaphore.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @param timeout time to wait.
 @return true if the semaphore to be taken, false if timeout.
 */
bool bTSC2007_WaitTouchDetected(tsc2007_handle_t* handle, uint32_t timeout);

/*************************************************************************************************/
/**
 @brief Clear the touch detected status.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @return void
 */
void vTSC2007_ClearPENStatus(tsc2007_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Return the status of the IRQ;
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @return true if the IRQ have ocurred
 */
bool bTSC2007_isTouchDetected(tsc2007_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Return the current status of the pen pin.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @return true if pin is high, false if low
 */
bool bTSC2007_getPENStatus(tsc2007_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Read touch data (X,Y,Z)
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @param dataSet structure with the coordinate data.
 @return kStatus_Success if communication was successful.
 */
status_t sTSC2007_readValues(tsc2007_handle_t *handle, tsc2007_data_set_t *dataSet);

/*************************************************************************************************/
/**
 @brief Reads multiple data bytes from I2C.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @param data pointer to the data received.
 @param length number of bytes to receive
 @return kStatus_Success if communication was successful.
 */
status_t sTSC2007_ReadMultiByte(tsc2007_handle_t* handle, uint8_t *data, uint8_t length);

/*************************************************************************************************/
/**
 @brief Writes a byte to I2C.
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @param value byte to be written.
 @return kStatus_Success if communication was successful.
 */
status_t sTSC2007_WriteByte(tsc2007_handle_t* handle, uint8_t value);

/*************************************************************************************************/
/**
 @brief Handles the I2C event
 @param handle TSC2007 driver handle to I2C, pins and signals related parameters
 @param i2c_event I2C event
 @return void
 */
void vTSC2007_EventHandler(tsc2007_handle_t *handle, uint32_t i2c_event);
/*************************************************************************************************/

#endif
