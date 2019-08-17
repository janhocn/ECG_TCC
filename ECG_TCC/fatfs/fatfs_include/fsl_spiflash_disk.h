/**
 ******************************************************************************
 * @file    fsl_spiflash_disk.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    21 March 2018
 * @brief   Description:
 ******************************************************************************
 */

#ifndef FATFS_INCLUDE_FSL_SPIFLASH_DISK_H_
#define FATFS_INCLUDE_FSL_SPIFLASH_DISK_H_

#include "ff.h"
#include "diskio.h"
#include "SpiFlash_Types.h"

#define SPIFlashFatFs_DRIVE_NOT_MOUNTED ~0

/***************************************************************************/
/*!
 *  @brief Driver handle for configuration of the chip
 *
 *  The application should never directly access the fields in the structure.
 */
typedef SpiFlash_handle_t FlashChipConfig;

typedef struct SpiFlashFatFs_Driver
{
    status_t (*SPIFlash_Init)(FlashChipConfig*, ARM_SPI_SignalEvent_t);
    status_t (*SPIFlash_Deinit)(FlashChipConfig*);
    status_t (*SPIFlash_Read)(FlashChipConfig*, uint32_t, uint8_t *, uint16_t);
    status_t (*SPIFlash_Write)(FlashChipConfig*, uint32_t, uint8_t *, uint16_t);
    status_t (*SPIFlash_Sync)(FlashChipConfig*, bool*, uint32_t);
    void (*SPIFlash_EventHandler)(FlashChipConfig *, uint32_t);
    bool (*SPIFlash_isWaitingEvent)(FlashChipConfig *);
} SpiFlashFatFs_Driver;
/***************************************************************************/
/*!
 *  @brief SpiFlashFatFs Object
 *
 *  The application should never directly access the fields in the structure.
 */
typedef struct SpiFlashFatFs_Object
{
    SpiFlashFatFs_Driver* flashDriver;
    FlashChipConfig flashObj;
    UINT driveNumber;
    FATFS fileSystem;
    DSTATUS diskState; /*!< Disk status */
} SpiFlashFatFs_Object;

/***************************************************************************/
/*!
 *  @brief SpiFlashFatFs SPIFlashFatFs_Handle
 *
 *  Used to identify a SPIFlashFatFs device in the APIs
 */
typedef SpiFlashFatFs_Object *SPIFlashFatFs_Handle;

/*!
 * @name SPI Flash Disk Function
 * @{
 */

/*!
 * @brief Initializes SPI Flash disk.
 *
 * @param physicalDrive Physical drive number.
 * @retval STA_NOINIT Failed.
 * @retval RES_OK Success.
 */
DSTATUS spiflash_disk_initialize(BYTE physicalDrive);

/*!
 * Gets SD SPI Flash status
 *
 * @param physicalDrive Physical drive number.
 * @retval STA_NOINIT Failed.
 * @retval RES_OK Success.
 */
DSTATUS spiflash_disk_status(BYTE physicalDrive);

/*!
 * @brief Reads SPI Flash SPI disk.
 *
 * @param physicalDrive Physical drive number.
 * @param buffer The data buffer pointer to store read content.
 * @param sector The start sector number to be read.
 * @param count The sector count to be read.
 * @retval RES_PARERR Failed.
 * @retval RES_OK Success.
 */
DRESULT spiflash_disk_read(BYTE physicalDrive, BYTE *buf, DWORD sector, BYTE count);

/*!
 * @brief Writes SPI Flash disk.
 *
 * @param physicalDrive Physical drive number.
 * @param buffer The data buffer pointer to store write content.
 * @param sector The start sector number to be written.
 * @param count The sector count to be written.
 * @retval RES_PARERR Failed.
 * @retval RES_OK Success.
 */
DRESULT spiflash_disk_write(BYTE physicalDrive, const BYTE *buf, DWORD sector, BYTE count);

/*!
 * @brief SPI Flash disk IO operation.
 *
 * @param physicalDrive Physical drive number.
 * @param command The command to be set.
 * @param buffer The buffer to store command result.
 * @retval RES_PARERR Failed.
 * @retval RES_OK Success.
 */
DRESULT spiflash_disk_ioctl(BYTE physicalDrive, BYTE ctrl, void *buf);

/* @} */

/*!
 *  @brief  Function to initialize the SPIFlashFatFS module
 */
void SPIFlashFatFs_init(SpiFlashFatFs_Driver* driver);

/*!
 *  @brief  Function to open a  SPIFlashFatFs object
 */
SPIFlashFatFs_Handle SPIFlashFatFs_open(FlashChipConfig* flashHandle, SpiFlashFatFs_Driver* driver, UINT drv);

#endif /* FATFS_INCLUDE_FSL_SPIFLASH_DISK_H_ */
