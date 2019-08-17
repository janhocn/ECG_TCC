/**
 ******************************************************************************
 * @file    W25QXX.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    21 March 2018
 * @brief   Description: Module to control the W25QXX SPI Flash chips.
 ******************************************************************************
 */
#ifndef PERIPH_DRIVERS_SPIFLASH_DRIVER_W25QXX_H_
#define PERIPH_DRIVERS_SPIFLASH_DRIVER_W25QXX_H_

#include "W25QXX_Defs.h"
#include "SpiFlash_Types.h"

/*************************************************************************************************/
/**
 @brief Initializes DSPI and verifies communication with the W25QXX.
 @brief Must be called before using any other function.

 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param cb_event callback to DSPI signal event
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Init(SpiFlash_handle_t* handle, ARM_SPI_SignalEvent_t cb_event);

/*************************************************************************************************/
/**
 @brief Deinitializes DSPI communication.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Deinit(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Search the partnumber, it must be called at initialization.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_CheckPartNumber(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Put the flash in power down state.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_PowerDown(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Read the Status Register.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param SR Pointer to the status register value.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_readSR(SpiFlash_handle_t* handle, uint8_t* SR);

/*************************************************************************************************/
/**
 @brief Read the Partnumber ID.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param ID Pointer to the ID value.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_readPartID(SpiFlash_handle_t* handle, uint16_t* ID);

/*************************************************************************************************/
/**
 @brief Read the Manufacturer ID.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param MF Pointer to the Manufacturer ID value.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_readManufacturer(SpiFlash_handle_t* handle, uint8_t* MF);

/*************************************************************************************************/
/**
 @brief Read the Unique ID.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param ID Pointer to the Unique ID value.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_readUniqueID(SpiFlash_handle_t* handle, uint64_t* ID);

/*************************************************************************************************/
/**
 @brief Check the busy state of the chip.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param busy Pointer to the state response, (true if busy)
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_isBusy(SpiFlash_handle_t* handle, bool* busy);

/*************************************************************************************************/
/**
 @brief Wait for timeout ms the chip goes to the ready state
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param syncd Pointer to the state response, (true if ready, false if timeout (busy))
 @param timeout time in milliseconds to wait.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Sync(SpiFlash_handle_t* handle, bool* syncd, uint32_t timeout);

/*************************************************************************************************/
/**
 @brief Erase one sector of the memory.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param addr_start sector address
 @param timeout time in milliseconds to wait.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_EraseSector(SpiFlash_handle_t* handle, uint32_t addr_start);

/*************************************************************************************************/
/**
 @brief Erase a 32kB block of the memory.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param addr_start sector address
 @param timeout time in milliseconds to wait.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Erase32kBlock(SpiFlash_handle_t* handle, uint32_t addr_start);

/*************************************************************************************************/
/**
 @brief Erase a 64kB block of the memory.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param addr_start sector address
 @param timeout time in milliseconds to wait.
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Erase64kBlock(SpiFlash_handle_t* handle, uint32_t addr_start);

/*************************************************************************************************/
/**
 @brief Erase the entire memory.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_EraseAll(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Resume an erase operation.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_EraseResume(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Suspend an erase operation.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_EraseSuspend(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Read bytes from the memory.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param addr address to read
 @param buf pointer to the buffer to hold the readed block
 @param len number of bytes to read
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Read(SpiFlash_handle_t* handle, uint32_t addr, uint8_t *buf, uint16_t len);

/*************************************************************************************************/
/**
 @brief Write bytes to the memory.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param addr address to write
 @param buf pointer to the buffer with the values to write
 @param len number of bytes to write
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_Write(SpiFlash_handle_t* handle, uint32_t addr, uint8_t *buf, uint16_t len);

/*************************************************************************************************/
/**
 @brief Enable write operation.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_WriteEnable(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Disable write operation.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful.
 */
status_t sW25QXX_WriteDisable(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Handles the DSPI event
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @param i2c_event DSPI event
 @return void
 */
void vW25QXX_EventHandler(SpiFlash_handle_t *handle, uint32_t dspi_event);

/*************************************************************************************************/
/**
 @brief Verifies if the driver is waiting for a event to occur
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return true if waiting, false otherwise
 */
bool bW25QXX_isWaitingEvent(SpiFlash_handle_t *handle);

/*************************************************************************************************/
/**
 @brief Put the CS Line to Low to select the chip.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return void.
 */
void vW25QXX_selectChip(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Put the CS Line to High to unselect the chip.
 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return void
 */
void vW25QXX_deselectChip(SpiFlash_handle_t* handle);

/*************************************************************************************************/
/**
 @brief Performs a hardware reset on the Flash

 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sW25QXX_HWReset(SpiFlash_handle_t *handle);

/*************************************************************************************************/
/**
 @brief Enable the write protect pin

 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sW25QXX_SetWriteProtect(SpiFlash_handle_t *handle);

/*************************************************************************************************/
/**
 @brief Disable the write protect pin

 @param handle Spi flash driver handle to DSPI, pins and signals related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
status_t sW25QXX_ClearWriteProtect(SpiFlash_handle_t *handle);
/*************************************************************************************************/
#endif /* PERIPH_DRIVERS_SPIFLASH_DRIVER_W25QXX_H_ */
