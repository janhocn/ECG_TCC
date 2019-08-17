/**
 ******************************************************************************
 * @file    fsl_spiflash_disk.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    21 March 2018
 * @brief   Description:
 ******************************************************************************
 */
#include "fsl_spiflash_disk.h"


#define SPIPhysDriver2Index(phys)	(phys-SPIFLASHDISK)
/*
 * Array of SpiFlashFatFs_Object to determine the association of the FatFs
 * drive number _VOLUMES is defined in <ffconf.h>
 */
static SpiFlashFatFs_Object spiflashfatfsObjects[_VOLUMES_FLASHSPI];

static bool initialized = false;
/*************************************************************************************************/
/**
 * @brief checks the driver to verify if it is a spi flash driver
 * @param physicalDrive physical driver
 * @return true if the driver is valid
 */
static bool _SPIFlashFatFs_isDriver(UINT physicalDrive)
{
    int i;
    for (i = 0; i < _VOLUMES_FLASHSPI; i++)
    {
        if (physicalDrive == (SPIFLASHDISK+i))
        {
            return true;
        }
    }
    return false;
}
/*************************************************************************************************/
/**
 * @brief Handles the spi event signal
 * @param event Signal comming from spi CMSIS IRQ handler
 * @return void
 * @attention This callback searches for events on all handles in spiflashfatfsObjects array
 */
static void _SPIFlashFatFsSignalEvent(uint32_t event)
{
    for (int i = 0; i < _VOLUMES_FLASHSPI; i++)
    {
        if (spiflashfatfsObjects[i].flashDriver->SPIFlash_isWaitingEvent(&spiflashfatfsObjects[i].flashObj))
        {
            spiflashfatfsObjects[i].flashDriver->SPIFlash_EventHandler(&spiflashfatfsObjects[i].flashObj, event);
        }
    }
}

/**
 *  ======== spiflash_disk_initialize ========
 */
DSTATUS spiflash_disk_initialize(BYTE physicalDrive)
{
    status_t status;

    if (!_SPIFlashFatFs_isDriver(physicalDrive))
    {
        return STA_NOINIT;
    }

    SPIFlashFatFs_Handle handle = &(spiflashfatfsObjects[SPIPhysDriver2Index(physicalDrive)]);


    /* Don't need to keep the handle since we have the object already */
    status = handle->flashDriver->SPIFlash_Init(&(handle->flashObj), _SPIFlashFatFsSignalEvent);

    if (status != kStatus_Success)
    {
        handle->flashDriver->SPIFlash_Deinit(&(handle->flashObj));

        handle->driveNumber = SPIFlashFatFs_DRIVE_NOT_MOUNTED;

        handle->diskState = STA_NOINIT;
        return handle->diskState;
    }

    handle->diskState = 0;

    return 0;
}

/**
 *  ======== spiflash_disk_ioctrl ========
 */
DRESULT spiflash_disk_ioctl(BYTE physicalDrive, BYTE ctrl, void *buf)
{
    if (!_SPIFlashFatFs_isDriver(physicalDrive))
    {
        return RES_PARERR;
    }

    DRESULT res = RES_ERROR;
    SPIFlashFatFs_Handle handle = &(spiflashfatfsObjects[SPIPhysDriver2Index(physicalDrive)]);
    volatile bool busy = 0;

    if (handle->diskState & STA_NOINIT)
    {
        return (RES_NOTRDY);
    }

    switch (ctrl)
    {
        case GET_SECTOR_COUNT:
            /* Get number of pages on the disk (DWORD) */
            if (buf)
            {
                *(DWORD*) buf = (DWORD) handle->flashObj.partInfo.pages;
                res = RES_OK;
            }
            else
            {
                res = RES_PARERR;
            }
            break;

        case GET_SECTOR_SIZE:
            /* Get page size on the disk (WORD) */
            if (buf)
            {
                *(WORD*) buf = handle->flashObj.partInfo.bytesPerPage;
                res = RES_OK;
            }
            else
            {
                res = RES_PARERR;
            }
            break;

        case CTRL_SYNC:
            /* Make sure that data has been written */
            if (handle->flashDriver->SPIFlash_Sync(&(handle->flashObj),(bool *) &busy, 200) == kStatus_Success)
            {
                if (!busy)
                {
                    res = RES_OK;
                }
                else
                {
                    res = RES_NOTRDY;
                }
            }
            else
            {
                res = RES_ERROR;
            }
            break;

        default:
            res = RES_OK;
            break;
    }

    return (res);
}

/**
 *  ======== spiflash_disk_status ========
 */
DSTATUS spiflash_disk_status(BYTE physicalDrive)
{
    if (!_SPIFlashFatFs_isDriver(physicalDrive))
    {
        return STA_NOINIT;
    }

    SPIFlashFatFs_Handle handle = &(spiflashfatfsObjects[SPIPhysDriver2Index(physicalDrive)]);

    return (handle->diskState);
}

/**
 *  ======== spiflash_disk_write ========
 */
DRESULT spiflash_disk_write(BYTE physicalDrive, const BYTE *buf,
        DWORD sector, BYTE count)
{
    if (!_SPIFlashFatFs_isDriver(physicalDrive))
    {
        return RES_PARERR;
    }

    uint32_t size;

    SPIFlashFatFs_Handle handle = &(spiflashfatfsObjects[SPIPhysDriver2Index(physicalDrive)]);

    if (!count)
    {
        return (RES_PARERR);
    }
    if (handle->diskState & STA_NOINIT)
    {
        return (RES_NOTRDY);
    }
    if (handle->diskState & STA_PROTECT)
    {
        return (RES_WRPRT);
    }

    size = handle->flashObj.partInfo.bytesPerPage;
    /* Single block write */
    if (count == 1)
    {
        if (handle->flashDriver->SPIFlash_Write(&(handle->flashObj), (uint32_t) sector, (uint8_t *) buf, handle->flashObj.partInfo.bytesPerPage) == kStatus_Success)
        {
            count = 0;
        }
    }
    /* Multiple block write */
    else
    {
        /* WRITE_MULTIPLE_BLOCK */
        do
        {
            if (handle->flashDriver->SPIFlash_Write(&(handle->flashObj), (uint32_t) sector, (uint8_t *) buf, size) == kStatus_Success)
            {
                buf += size;
                sector++;
            }

        }
        while (--count);
    }

    return (count ? RES_ERROR : RES_OK);
}

/**
 *  ======== spiflash_disk_read ========
 */
DRESULT spiflash_disk_read(BYTE physicalDrive, BYTE *buf, DWORD sector, BYTE count)
{
    if (!_SPIFlashFatFs_isDriver(physicalDrive))
    {
        return RES_PARERR;
    }

    uint32_t size;
    SPIFlashFatFs_Handle handle = &(spiflashfatfsObjects[SPIPhysDriver2Index(physicalDrive)]);

    if (!count)
    {
        return (RES_PARERR);
    }

    if (handle->diskState & STA_NOINIT)
    {
        return (RES_NOTRDY);
    }

    size = handle->flashObj.partInfo.bytesPerPage;

    /* Single block read */
    if (count == 1)
    {
        if (handle->flashDriver->SPIFlash_Read(&(handle->flashObj), (uint32_t) sector, (uint8_t *) buf, size) == kStatus_Success)
        {
            count = 0;
        }

    }
    /* Multiple block read */
    else
    {
        do
        {
            if (handle->flashDriver->SPIFlash_Read(&(handle->flashObj), (uint32_t) sector, (uint8_t *) buf, size) == kStatus_Success)
            {
                buf += size;
                sector++;
            }
        }
        while (--count);
    }

    return (count ? RES_ERROR : RES_OK);
}

/**
 *  ======== SPIFlashFatFs_init ========
 */
void SPIFlashFatFs_init(SpiFlashFatFs_Driver* driver)
{
    int i;
    for (i = 0; i < _VOLUMES_FLASHSPI; i++)
    {
        spiflashfatfsObjects[i].diskState = STA_NOINIT;
        spiflashfatfsObjects[i].driveNumber = SPIFlashFatFs_DRIVE_NOT_MOUNTED;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_Init = driver->SPIFlash_Init;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_Deinit = driver->SPIFlash_Deinit;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_Read = driver->SPIFlash_Read;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_Write = driver->SPIFlash_Write;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_Sync = driver->SPIFlash_Sync;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_EventHandler = driver->SPIFlash_EventHandler;
        spiflashfatfsObjects[i].flashDriver->SPIFlash_isWaitingEvent = driver->SPIFlash_isWaitingEvent;
    }

    initialized = true;
}

/**
 *  ======== SPIFlashFatFs_open ========
 */
SPIFlashFatFs_Handle SPIFlashFatFs_open(FlashChipConfig* flashHandle, SpiFlashFatFs_Driver* driver, UINT physicalDrive)
{

    if (!_SPIFlashFatFs_isDriver(physicalDrive) || !initialized)
    {
        return (NULL);
    }

    SPIFlashFatFs_Handle handle = &(spiflashfatfsObjects[SPIPhysDriver2Index(physicalDrive)]);

    /* Mark as being used */
    handle->driveNumber = physicalDrive;

    memcpy(&(handle->flashObj), flashHandle, sizeof(FlashChipConfig));

    return (handle);
}

/*************************************************************************************************/
