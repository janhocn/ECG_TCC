/**
 ******************************************************************************
 * @file    SpiFlash_Types.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    21 March 2018
 * @brief   Description: This header contains typed to handler
 *                       a SPI Flash driver
 ******************************************************************************
 */
#ifndef PERIPH_DRIVERS_SPIFLASH_DRIVER_SPIFLASH_TYPES_H_
#define PERIPH_DRIVERS_SPIFLASH_DRIVER_SPIFLASH_TYPES_H_

#include "W25QXX_Defs.h"
#include "fsl_dspi_cmsis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task_manager.h"


/*************************************************************************************************/
/**
 * @enum Partnumber list
 */
typedef enum _partNumber
{
    W25Q80 = 0,
    W25Q16,
    W25Q32,
    W25Q64,
    W25Q128,
    NUMBER_PARTS
} partNumber;

/*************************************************************************************************/
/**
 * @struct Structure to hold chip specific information
 */
typedef struct _pnListType
{
    partNumber pn; /** @< Partnumber */
    uint16_t id; /** @< part id */
    uint32_t bytes; /** @< total number of bytes */
    uint32_t pages; /** @< number of pages */
    uint32_t sectors; /** @< number of sectors per page */
    uint32_t blocks; /** @< number of blocks per sector */
    uint32_t bytesPerPage; /** number of bytes per page */
} pnListType;

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to configure and use this driver
 */
typedef struct _W25QXX_handle
{
    ARM_DRIVER_SPI dspi_driver; /** @< SP� CMSIS Driver */
    volatile uint32_t dspi_event; /** @< Hold the event coming from DMA */
    volatile bool dspi_event_received; /** @< Signals event received */
    volatile bool dspi_event_wait; /** @< Signals that is waiting and event */
    IRQn_Type DMATXIRQn; /** @< DMA TX SPI IRQ */
    IRQn_Type DMARXIRQn; /** @< DMA RX SPI IRQ */
    uint32_t DMAIRQPriority; /** @< DMA IRQ RXTX priority */
    GPIO_Type *RSTGpio; /** @< Reset GPIO */
    uint16_t RSTPin; /** @< Reset pin */
    GPIO_Type *WPGpio; /** @< Write protect GPIO */
    uint16_t WPPin; /** @< Write protect pin */
    GPIO_Type *CSGpio; /** @< Chip Select GPIO */
    uint16_t CSPin; /** @< Chip Select pin */
    uint32_t baudrate; /** @< SPI baudrate */
    pnListType partInfo; /** @< Structure containing the memory information */
    SemaphoreHandle_t xSemaSignal; /** @< Semaphore to control the singals coming from DMA*/
} SpiFlash_handle_t;

#endif /* PERIPH_DRIVERS_SPIFLASH_DRIVER_SPIFLASH_TYPES_H_ */
