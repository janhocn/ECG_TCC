/**
 ******************************************************************************
 * @file    W25QXX.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    21 March 2018
 * @brief   Description: Module to control the W25QXX SPI Flash chips.
 ******************************************************************************
 */

#include "W25QXX.h"
#include "fsl_device_registers.h"
#include "fsl_dspi.h"

#include "fsl_common.h"
#include "user.h"
#include "InterruptManager.h"
#include "prioAssigner.h"
#include "SystickUser.h"

//W25Q64 = 256_bytes_per_page * 16_pages_per_sector * 16_sectors_per_block * 128_blocks_per_chip
//= 256b*16*16*128 = 8Mbyte = 64MBits
static const pnListType pnList[] = {
        { W25Q80, 0x4014, 1048576, 4096, 256, 16, 256 },
        { W25Q16, 0x4015, 2097152, 8192, 512, 32, 256 },
        { W25Q32, 0x4016, 4194304, 16384, 1024, 64, 256 },
        { W25Q64, 0x4017, 8388608, 32768, 2048, 128, 256 },
        { W25Q128, 0x4018, 16777216, 65536, 4096, 256, 256 }
};

/**
 @brief Wait Master Complete Event on DSPI
 @param handle W25QXX driver handle to DSPI related parameters
 @return DSPI event
 */
static uint32_t _u32W25QXX_WaitEvent(SpiFlash_handle_t *handle);

/*************************************************************************************************/
status_t sW25QXX_Init(SpiFlash_handle_t* handle, ARM_SPI_SignalEvent_t cb_event)
{
    uint8_t dspi_buf[1] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    handle->xSemaSignal = xTaskManager_SemaphoreCreate();
    configASSERT(handle->xSemaSignal != NULL);

    handle->dspi_driver.Initialize(cb_event);
    handle->dspi_driver.PowerControl(ARM_POWER_FULL);
    handle->dspi_driver.Control( ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA1 | ARM_SPI_SS_MASTER_SW
            | ARM_SPI_DATA_BITS(8), handle->baudrate);

    NVIC_SetPriority(handle->DMATXIRQn, handle->DMAIRQPriority);
    NVIC_SetPriority(handle->DMARXIRQn, handle->DMAIRQPriority);

    GPIO_SetPinsOutput(handle->RSTGpio, 1U << handle->RSTPin);
    vSystickUser_Delay(50);

    dspi_buf[0] = RELEASE;
    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        return kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        return kStatus_Fail;
    }
    vW25QXX_deselectChip(handle);

    vSystickUser_Delay(1);

    if (sW25QXX_CheckPartNumber(handle) == kStatus_Success)
    {
        return kStatus_Success;
    }

    return kStatus_Fail;
}

/*************************************************************************************************/
status_t sW25QXX_Deinit(SpiFlash_handle_t* handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    handle->dspi_driver.PowerControl(ARM_POWER_OFF);
    handle->dspi_driver.Uninitialize();

    if (handle->xSemaSignal)
    {
        vTaskManager_SemaphoreDelete(handle->xSemaSignal);
    }
    return kStatus_Success;
}

/*************************************************************************************************/
void vW25QXX_selectChip(SpiFlash_handle_t* handle)
{
    GPIO_ClearPinsOutput(handle->CSGpio, 1U << handle->CSPin);
}

/*************************************************************************************************/
void vW25QXX_deselectChip(SpiFlash_handle_t* handle)
{
    GPIO_SetPinsOutput(handle->CSGpio, 1U << handle->CSPin);
}

/*************************************************************************************************/
status_t sW25QXX_readSR(SpiFlash_handle_t* handle, uint8_t* SR)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[4] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = R_SR1;
    dspi_buf[1] = 0xFF;

    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[2], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    vW25QXX_deselectChip(handle);
    vW25QXX_deselectChip(handle);

    dspi_buf[0] = R_SR2;
    dspi_buf[1] = 0xFF;
    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[3], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    vW25QXX_deselectChip(handle);

    *SR = (((uint16_t) dspi_buf[3]) << 8) | dspi_buf[2];
    return status;
}

/*************************************************************************************************/
status_t sW25QXX_readManufacturer(SpiFlash_handle_t* handle, uint8_t* MF)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[3] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = R_JEDEC_ID;
    dspi_buf[1] = 0x00;

    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[2], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);
    *MF = dspi_buf[2];
    return status;
}

/*************************************************************************************************/
status_t sW25QXX_readUniqueID(SpiFlash_handle_t* handle, uint64_t* ID)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[2] = { 0 };
    uint8_t *arrUID;
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    arrUID = (uint8_t*) ID;
    dspi_buf[0] = R_UNIQUE_ID;
    dspi_buf[1] = 0x00;

    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else
    {
        for (int i = 7; i >= 0; i--)
        {
            if (handle->dspi_driver.Transfer(&dspi_buf[1], &arrUID[i], 1) != ARM_DRIVER_OK)
            {
                status = kStatus_Fail;
                break;
            }
            else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
            {
                status = kStatus_Fail;
                break;
            }
        }

    }

    vW25QXX_deselectChip(handle);
    return status;
}

/*************************************************************************************************/
status_t sW25QXX_readPartID(SpiFlash_handle_t* handle, uint16_t* ID)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[4] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = R_JEDEC_ID;
    dspi_buf[1] = 0x00;

    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[2], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[3], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);
    *ID = (dspi_buf[2] << 8) | dspi_buf[3];
    return status;
}

/*************************************************************************************************/
status_t sW25QXX_isBusy(SpiFlash_handle_t* handle, bool* busy)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[3] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = R_SR1;
    dspi_buf[1] = 0xff;

    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[2], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    if (dspi_buf[2] & SR1_BUSY_MASK)
    {
        *busy = true;
    }
    else
    {
        *busy = false;
    }

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_Sync(SpiFlash_handle_t* handle, bool* syncd, uint32_t timeout)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[3] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = R_SR1;
    dspi_buf[1] = 0x00;

    vW25QXX_selectChip(handle);

    int16_t timeStart = u32SystickUser_getTick();
    do
    {
        if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[2], 1) != ARM_DRIVER_OK)
        {
            status = kStatus_Fail;
            break;
        }
        else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
        {
            status = kStatus_Fail;
            break;
        }

        if ((dspi_buf[2] & SR1_BUSY_MASK) == 0)
        {
            vW25QXX_deselectChip(handle);
            *syncd = true;

            return kStatus_Success;
        }
    }
    while (!bSystickUser_Wait(timeStart, timeout));
    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_WriteEnable(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[1] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = W_EN;
    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_WriteDisable(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[1] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = W_DE;
    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_Write(SpiFlash_handle_t* handle, uint32_t addr, uint8_t *buf, uint16_t len)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[5] = { 0 };
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    uint16_t n;
    uint16_t maxBytes = 256 - (addr % 256);  // force the first set of bytes to stay within the first page
    uint16_t offset = 0;

    while (len > 0)
    {
        n = (len <= maxBytes) ? len : maxBytes;
        vW25QXX_selectChip(handle);
        sW25QXX_WriteEnable(handle);
        dspi_buf[0] = PAGE_PGM;
        dspi_buf[1] = (uint8_t) (addr >> 16);
        dspi_buf[2] = (uint8_t) (addr >> 8);
        dspi_buf[3] = (uint8_t) (addr);
        if (handle->dspi_driver.Send(&dspi_buf[0], 4) != ARM_DRIVER_OK)
        {
            status = kStatus_Fail;
        }
        else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
        {
            status = kStatus_Fail;
        }
        else if (handle->dspi_driver.Send(&buf[offset], n) != ARM_DRIVER_OK)
        {
            status = kStatus_Fail;
        }
        else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
        {
            status = kStatus_Fail;
        }

        vW25QXX_deselectChip(handle);
        if (status == kStatus_Fail)
        {
            return status;
        }

        addr += n;  // adjust the addresses and remaining bytes by what we've just transferred.
        offset += n;
        len -= n;
        maxBytes = 256;   // now we can do up to 256 bytes per loop
    }

    if (status == kStatus_Fail)
    {
        return status;
    }

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_Read(SpiFlash_handle_t* handle, uint32_t addr, uint8_t *buf, uint16_t len)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[5] = { 0 };
    bool sync = false;

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    sW25QXX_Sync(handle, &sync, DEFAULT_TIMEOUT);
    if (!sync)
    {
        return kStatus_Fail;
    }

    dspi_buf[0] = READ;
    dspi_buf[1] = (uint8_t) (addr >> 16);
    dspi_buf[2] = (uint8_t) (addr >> 8);
    dspi_buf[3] = (uint8_t) addr;
    dspi_buf[4] = 0x00;

    vW25QXX_selectChip(handle);
    if (handle->dspi_driver.Send(&dspi_buf[0], 4) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[4], buf, len) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_EraseSector(SpiFlash_handle_t* handle, uint32_t addr_start)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[4] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (addr_start & 0x00000fff) //test if addr_start is 12bit-aligned
        return kStatus_Fail;

    dspi_buf[0] = SECTOR_E;
    dspi_buf[1] = (uint8_t) (addr_start >> 16);
    dspi_buf[2] = (uint8_t) (addr_start >> 8);
    dspi_buf[3] = (uint8_t) addr_start;

    sW25QXX_WriteEnable(handle);
    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 4) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_Erase32kBlock(SpiFlash_handle_t* handle, uint32_t addr_start)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[4] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (addr_start & 0x00007fff) //test if addr_start is 15bit-aligned
        return kStatus_Fail;

    dspi_buf[0] = BLK_E_32K;
    dspi_buf[1] = (uint8_t) (addr_start >> 16);
    dspi_buf[2] = (uint8_t) (addr_start >> 8);
    dspi_buf[3] = (uint8_t) addr_start;

    sW25QXX_WriteEnable(handle);
    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 4) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_Erase64kBlock(SpiFlash_handle_t* handle, uint32_t addr_start)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[4] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (addr_start & 0x0000ffff) //test if addr_start is 16bit-aligned
        return kStatus_Fail;

    dspi_buf[0] = BLK_E_64K;
    dspi_buf[1] = (uint8_t) (addr_start >> 16);
    dspi_buf[2] = (uint8_t) (addr_start >> 8);
    dspi_buf[3] = (uint8_t) addr_start;

    sW25QXX_WriteEnable(handle);
    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 4) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_EraseAll(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[1] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = CHIP_ERASE;

    sW25QXX_WriteEnable(handle);
    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_EraseSuspend(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[1] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = E_SUSPEND;

    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_EraseResume(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[1] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = E_RESUME;

    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_PowerDown(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[1] = { 0 };

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = PDWN;

    vW25QXX_selectChip(handle);

    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32W25QXX_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }

    vW25QXX_deselectChip(handle);

    vSystickUser_Delay(1);

    return status;
}

/*************************************************************************************************/
status_t sW25QXX_CheckPartNumber(SpiFlash_handle_t* handle)
{
    status_t status = kStatus_Success;
    uint8_t manuf;
    uint16_t id;

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    status = sW25QXX_readManufacturer(handle, &manuf);
    if (status == kStatus_Success)
    {
        status = sW25QXX_readPartID(handle, &id);
        if (status == kStatus_Success)
        {
            if (manuf != WINBOND_MANUF)
            {
                return kStatus_Fail;
            }

            for (int i = 0; i < NUMBER_PARTS; i++)
            {
                if (id == pnList[i].id)
                {
                    handle->partInfo.pn = pnList[i].pn;
                    handle->partInfo.id = pnList[i].id;
                    handle->partInfo.bytes = pnList[i].bytes;
                    handle->partInfo.pages = pnList[i].pages;
                    handle->partInfo.sectors = pnList[i].sectors;
                    handle->partInfo.blocks = pnList[i].blocks;
                    handle->partInfo.bytesPerPage = pnList[i].bytesPerPage;
                    return kStatus_Success;
                }
            }
        }
    }
    return kStatus_Fail;
}

/*************************************************************************************************/
void vW25QXX_EventHandler(SpiFlash_handle_t *handle, uint32_t dspi_event)
{
    if (!handle)
    {
        return;
    }

    handle->dspi_event = dspi_event;
    handle->dspi_event_received = true;
    handle->dspi_event_wait = false;
    xTaskManager_SemaphoreGive(handle->xSemaSignal);
}

/*************************************************************************************************/
bool bW25QXX_isWaitingEvent(SpiFlash_handle_t *handle)
{
    if (!handle)
    {
        return false;
    }
    return handle->dspi_event_wait;
}

/*************************************************************************************************/
status_t sW25QXX_HWReset(SpiFlash_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    GPIO_ClearPinsOutput(handle->RSTGpio, 1U << handle->RSTPin);
    vSystickUser_Delay(300);
    GPIO_SetPinsOutput(handle->RSTGpio, 1U << handle->RSTPin);
    vSystickUser_Delay(50);

    return kStatus_Success;
}


/*************************************************************************************************/
status_t sW25QXX_SetWriteProtect(SpiFlash_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    GPIO_ClearPinsOutput(handle->WPGpio, 1U << handle->WPPin);

    return kStatus_Success;
}

/*************************************************************************************************/
status_t sW25QXX_ClearWriteProtect(SpiFlash_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    GPIO_SetPinsOutput(handle->WPGpio, 1U << handle->WPPin);

    return kStatus_Success;
}
/*************************************************************************************************/
static uint32_t _u32W25QXX_WaitEvent(SpiFlash_handle_t *handle)
{
    uint32_t dspi_event;

    if (!handle)
    {
        return 0xFF;
    }

    handle->dspi_event_wait = true;
    if (xTaskManager_SemaphoreTake(handle->xSemaSignal, DEFAULT_TIMEOUT) == pdTRUE)
    {
        dspi_event = handle->dspi_event;
    }
    else
    {
        dspi_event = ARM_DRIVER_ERROR_TIMEOUT;
    }
    return dspi_event;

}
/*************************************************************************************************/
