/**
 ******************************************************************************
 * @file    FileSystem.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 January 2018
 * @brief   File System Module
 ******************************************************************************
 */
#include "FileSystem.h"

#include <stdio.h>
#include <string.h>
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "diskio.h"
#include "ff.h"
#include "fsl_sd.h"
#include "prioAssigner.h"
#include "fsl_sdmmc_host.h"
#include "fsl_sd_disk.h"
#include "GUI.h"
/*************************************************************************************************/
/**
 * @brief Converts the access mode w,r,a to FATFS pattern
 * @param Permissions to modify the file (write, read, append ...)
 * @return FATFS pattern access mode
 */
static BYTE __FileSytemMode(char* mode);

/*************************************************************************************************/
/*!
 * @brief wait card insert function.
 */
static status_t sdcardWaitCardInsert(void);

/*************************************************************************************************/
/*!
 * @brief This function will be called by GUI_BMP_Serialize to write the
 *        bytes to the file
 */
static void _vFileSystem_ImgWriteBytes(uint8_t Data, void * p);
/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to this module
 */
static struct _sFileSystem
{
    FATFS g_fileSystem; /** @< File system object */
    DIR directory; /** @< Directory object */
} sFileSystem;

const TCHAR driverNumberBuffer[3U] = { SDDISK + '0', ':', '/' };

/*! @brief SDMMC host detect card configuration */
static const sdmmchost_detect_card_t s_sdCardDetect = {
#ifndef BOARD_SD_DETECT_TYPE
        .cdType = kSDMMCHOST_DetectCardByGpioCD,
#else
        .cdType = BOARD_SD_DETECT_TYPE,
#endif
        .cdTimeOut_ms = (~0U), };
/*************************************************************************************************/
static BYTE __FileSytemMode(char* mode)
{
    uint8_t result;
    result = strcmp(mode, "r");
    if (!result)
        return FA_READ;

    result = strcmp(mode, "r+");
    if (!result)
        return FA_READ | FA_WRITE;

    result = strcmp(mode, "w");
    if (!result)
        return FA_CREATE_ALWAYS | FA_WRITE;

    result = strcmp(mode, "w+");
    if (!result)
        return FA_CREATE_ALWAYS | FA_WRITE | FA_READ;

    result = strcmp(mode, "a");
    if (!result)
        return FA_OPEN_APPEND | FA_WRITE;

    result = strcmp(mode, "a+");
    if (!result)
        return FA_OPEN_APPEND | FA_WRITE | FA_READ;

    result = strcmp(mode, "wx");
    if (!result)
        return FA_CREATE_NEW | FA_WRITE;

    result = strcmp(mode, "w+x");
    if (!result)
        return FA_CREATE_NEW | FA_WRITE | FA_READ;

    return 0x00;
}

/*************************************************************************************************/
status_t stFileSystem_Init(void)
{
    FRESULT error;
    FILINFO fno;
    FRESULT result;
    if (sdcardWaitCardInsert() != kStatus_Success)
    {
        return kStatus_SDCardNotInserted;
    }

    if (f_mount(&sFileSystem.g_fileSystem, driverNumberBuffer, 0U))
    {
        return kStatus_Fail;
    }

#if (_FS_RPATH >= 2U)
    error = f_chdrive((char const *) &driverNumberBuffer[0U]);
    if (error)
    {
        return kStatus_Fail;
    }
#endif

#if _USE_MKFS
/*
    if (f_mkfs(driverNumberBuffer, FM_ANY, 0U, work, sizeof work))
    {
        return kStatus_Fail;
    }
*/
#endif /* _USE_MKFS */

    result = f_stat(_T(__DIR_LOG), &fno);
    if (result == FR_NO_FILE)
    {
        if (f_mkdir(_T(__DIR_LOG)) == FR_OK)
        {
            result = f_stat(_T(__DIR_PRINT), &fno);
            if (result == FR_NO_FILE)
            {
                if (f_mkdir(_T(__DIR_PRINT)) != FR_OK)
                {
                    return kStatus_Fail;
                }
            }
        }
    }

    /* Open root directory */
    if (f_opendir(&sFileSystem.directory, "/"))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}

/*************************************************************************************************/
FRESULT stFileSystem_OpenFile(FIL* fp, const TCHAR* path, char* mode)
{
    return f_open(fp, _T(path), __FileSytemMode(mode));
}

/*************************************************************************************************/
FRESULT stFileSystem_WriteFile(FIL* fp, const void* buff, UINT btw, UINT* bw)
{
    return f_write(fp, (void*) buff, btw, bw);
}

/*************************************************************************************************/
FRESULT stFileSystem_ReadFile(FIL* fp, const void* buff, UINT btr, UINT* br)
{
    return f_read(fp, (void*) buff, btr, br);
}

/*************************************************************************************************/
status_t stFileSystem_CloseFile(FIL* fp)
{
    if (f_close(fp))
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/*************************************************************************************************/
FRESULT stFileSystem_DeleteFile(const TCHAR* path)
{
    if (f_unlink(_T(path)))
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/*************************************************************************************************/
static void _vFileSystem_ImgWriteBytes(uint8_t Data, void * p)
{
    UINT nWritten;
    stFileSystem_WriteFile((FIL*) p, &Data, 1, &nWritten);
}

/*************************************************************************************************/
status_t sFileSystem_PrintScreen(const TCHAR* path)
{
    FIL fileHandle;
    FRESULT result;

    result = stFileSystem_OpenFile(&fileHandle, path, "w");
    if (result != FR_OK)
    {
        return kStatus_Fail;
    }

    GUI_BMP_Serialize(_vFileSystem_ImgWriteBytes, &fileHandle);

    stFileSystem_CloseFile(&fileHandle);

    return kStatus_Success;
}

/*************************************************************************************************/
static status_t sdcardWaitCardInsert(void)
{
    /* Save host information. */
    g_sd.host.base = SD_HOST_BASEADDR;
    g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;

    NVIC_SetPriority(SD_HOST_IRQ, SDCARD_INT_PRIO);

    /* SD host init function */
    if (SD_HostInit(&g_sd) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    /* power off card */
    SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);
    /* wait card insert */
    if (SD_WaitCardDetectStatus(SD_HOST_BASEADDR, &s_sdCardDetect, true) == kStatus_Success)
    {
        /* power on the card */
        SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
    }
    else
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}
