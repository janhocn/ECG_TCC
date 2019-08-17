/**
 ******************************************************************************
 * @file    AFE4400.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    5 November 2017
 * @brief   Description: Definition of the AFE4400 library, which implements features of the AFE4400 Pulse Oximetry.
 * @attention !!!This code was ported from https://github.com/mogar/AFE4400/!!!
 ******************************************************************************
 */
#ifndef PERIPH_DRIVERS_SPO2_DRIVER_AFE4400_H_
#define PERIPH_DRIVERS_SPO2_DRIVER_AFE4400_H_

#include "AFE4400_Defs.h"
#include "fsl_dspi_cmsis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task_manager.h"

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to configure and use this driver
 */
typedef struct _afe4400_handle {
	ARM_DRIVER_SPI dspi_driver; /** @< SP� CMSIS Driver*/
	volatile uint32_t dspi_event;  /** @< Hold the event coming from DMA*/
	volatile bool dspi_event_received; /** @< Signals event received*/
	IRQn_Type DMATXIRQn; /** @< DMA TX SPI IRQ*/
	IRQn_Type DMARXIRQn; /** @< DMA RX SPI IRQ*/
	uint32_t DMAIRQpriority; /** @< DMA Channel SPI IRQ priority*/
    GPIO_Type *RSTGpio;
    uint16_t RSTPin;
    GPIO_Type *PDNGpio;
    uint16_t PDNPin;
    GPIO_Type *CSGpio;
    uint16_t CSPin;
    uint32_t baudrate; /** @< SPI baudrate */
	SemaphoreHandle_t xSemaSignal; /** @< Semaphore to control the singals coming from DMA*/
} afe4400_handle_t;

/**
 @brief Initializes DSPI and verifies communication with the AFE4400.
 @brief Also initialize the timing registers,
 @brief Must be called before using any other functions.

 @param handle AFE4400 driver handle to DSPI related parameters
 @param cb_event callback to DSPI signal event
 @return kStatus_Success if communication was successful.
 */
status_t sAFE4400_Init(afe4400_handle_t* handle, ARM_SPI_SignalEvent_t cb_event);
/**
 @brief Deinitializes DSPI communication.

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sAFE4400_Deinit(afe4400_handle_t* handle);
/**
 @brief Reads a register value (see AFE4400_Defs)

 @param handle AFE4400 driver handle to DSPI related parameters
 @param address Address of the register (available on eAFE4400Registers enum)
 @param data Pointer to the data read
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_readReg(afe4400_handle_t* handle, eAFE4400Registers address, uint32_t* data);
/**
 @brief Writes data to a a register value (see AFE4400_Defs)

 @param handle AFE4400 driver handle to DSPI related parameters
 @param address Address of the register (available on eAFE4400Registers enum)
 @param data Data to be written on the specified register
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_writeReg(afe4400_handle_t* handle, eAFE4400Registers address, uint32_t data);
/**
 @brief Reads the ADC value of the IR Sensor

 @param handle AFE4400 driver handle to DSPI related parameters
 @param irvalue ADC value
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_ReadIRValue(afe4400_handle_t *handle, uint32_t* irvalue);
/**
 @brief Reads the ADC value of the Red Sensor

 @param handle AFE4400 driver handle to DSPI related parameters
 @param redvalue ADC value
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_ReadRedValue(afe4400_handle_t *handle, uint32_t* redvalue);
/**
 @brief Performs a software reset on the AFE4400

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_SWReset(afe4400_handle_t *handle);
/**
 @brief Performs a hardware reset on the AFE4400

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_HWReset(afe4400_handle_t *handle);
/**
 @brief Turn the AFE4400 On

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_PowerOn(afe4400_handle_t *handle);
/**
 @brief Turn the AFE4400 Off

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_PowerDown(afe4400_handle_t *handle);
/**
 @brief Sets the IR and LED currents

 @param handle AFE4400 driver handle to DSPI related parameters
 @param led1_current IR sensor current
 @param led2_current RED LED current
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sAFE4400_setLEDCurrent(afe4400_handle_t *handle, uint32_t led1_current, uint32_t led2_current);

/**
 @brief Handles the SPI event
 @param handle AFE4400 driver handle to DSPI related parameters
 @param dsp_event SPI event
 @return void
 */
void vAFE4400_EventHandler(afe4400_handle_t *handle, uint32_t dspi_event);
#endif /* PERIPH_DRIVERS_SPO2_DRIVER_AFE4400_H_ */
