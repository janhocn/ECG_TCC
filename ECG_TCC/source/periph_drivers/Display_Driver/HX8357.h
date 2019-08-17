/**
 ******************************************************************************
 * @file    HX8357.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    6 November 2017
 * @brief   Display HX8357 Driver
 ******************************************************************************
 */

#ifndef PERIPH_DRIVERS_DISPLAY_DRIVER_HX8357_H_
#define PERIPH_DRIVERS_DISPLAY_DRIVER_HX8357_H_

#include "HX8357_Defs.h"
#include "fsl_dspi_cmsis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task_manager.h"

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to configure and use this driver
 */
typedef struct _hx8357_handle
{
    DMA_Type * DMABase; /** @< SP� DMA Address*/
    DMAMUX_Type * DMAMuxBase; /** @< SP� DMA Mux Address*/
    ARM_DRIVER_SPI dspi_driver; /** @< CMSIS SPI Driver*/
    IRQn_Type DMARXIRQn; /** @< DMA RX IRQ*/
    IRQn_Type DMATXIRQn; /** @< DMA TX IRQ*/
    uint32_t DMAIRQpriority; /** @< DMA Channel DSPI IRQ priority*/
    volatile uint32_t dspi_event; /** @< Hold the event coming from DMA*/
    volatile bool dspi_event_received; /** @< Signals event received*/
    GPIO_Type *DCGpio; /** @< GPIO Type for pin DC*/
    uint16_t DCPin; /** @< Pin DC*/
    GPIO_Type *CSGpio;
    uint16_t CSPin;
    GPIO_Type *RSTGpio; /** @< GPIO Type for pin RST*/
    uint16_t RSTPin; /** @< Pin RST*/
    GPIO_Type *BKLGpio; /** @< GPIO Type for pin Backlight*/
    uint16_t BKLPin; /** @< Pin Backlight (PWM)*/
    uint32_t baudrate; /** @< SPI baudrate */
    SemaphoreHandle_t xSemaTxDone; /** @< Semaphore to control the singals coming from DMA*/
} hx8357_handle_t;

/*************************************************************************************************/
/**
 @brief Initializes DSPI and verifies communication with the HX8357.
 @param ConfigHandle Handle containing the configuration parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sHX8357_Init(hx8357_handle_t* ConfigHandle);
/**
 @brief DeInitializes DSPI communication
 */
void vHX8357_Deinit(void);
/**
 @brief Turn the backlight on (Pin is configured at initialization)
 */
void vHX8357_TurnBacklightON(void);
/**
 @brief Turn the backlight off (Pin is configured at initialization)
 */
void vHX8357_TurnBacklightOFF(void);
/**
 @brief Software Reset of the Display
 */
void vHX8357_ResetDisplay(void);
/**
 @brief CS pin control
 @param NotActive - 1 to set the CS pin, 0 to clear
 */
void vHX8357_SetCS(uint8_t NotActive);
/**
 @brief Enter/Exit in stand by mode
 @param true if you want to put the display in stand by mode
 */
void vHX8357_StandBy(bool enter);
/**
 @brief Writes 8 bit data
 @param Data Data to be writen
 */
void vHX8357_pfWrite8_A0(uint8_t Data);
/**
 @brief Writes 8 bit command
 @param Data Command to be writen
 */
void vHX8357_pfWrite8_A1(uint8_t Data);
/**
 @brief Writes multiple data bytes
 @param pData pointer to the buffer to be wiriten
 @param NumItems Number of bytes in the buffer
 */
void vHX8357_pfWriteM8_A0(uint8_t *pData, int NumItems);
/**
 @brief Writes multiple command bytes
 @param pData pointer to the buffer to be wiriten
 @param NumItems Number of bytes in the buffer
 */
void vHX8357_pfWriteM8_A1(uint8_t *pData, int NumItems);
/**
 @brief Reads multiple data bytes
 @param pData pointer to the buffer to be read
 @param NumItems Number of bytes in the buffer
 */
void vHX8357_pfReadM8_A1(uint8_t *pData, int NumItems);
/**
 @brief Reads on byte data
 @return byte read
 */
uint8_t vHX8357_pfRead8_A1(void);
/**
 @brief Reads multiple data bytes
 @param pData pointer to the buffer to be read
 @param NumItems Number of bytes in the buffer
 */
void vHX8357_pfReadM8_A0(uint8_t *pData, int NumItems);
/**
 @brief Reads on byte data
 @return byte read
 */
uint8_t vHX8357_pfRead8_A0(void);
#endif /* PERIPH_DRIVERS_DISPLAY_DRIVER_HX8357_H_ */
