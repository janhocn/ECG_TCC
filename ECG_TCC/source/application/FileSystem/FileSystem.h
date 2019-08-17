/**
 ******************************************************************************
 * @file    FileSystem.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    20 January 2018
 * @brief   File System Module
 ******************************************************************************
 */
#ifndef APPLICATION_FILESYSTEM_FILESYSTEM_H_
#define APPLICATION_FILESYSTEM_FILESYSTEM_H_

#include "ff.h"
#include "fsl_common.h"
/*!
 * @addtogroup File Strings
 * @attention All File names must be defined here
 * @{
 */
#define __DIR_LOG			    "/logs"
#define __DIR_PRINT			    "/prints"
#define __PRINT_ECG             "/prints/ECG.bmp"
#define __PRINT_SPO2            "/prints/SPO2.bmp"
#define __LOG_FILE              "/logs/log.txt"
#define __LOG_ERR_FILE          "/logs/logErr.txt"
#define __LOG_ECG_FILT_FILE	    "/logs/ECGFILT_00.txt"
#define __LOG_ECG_RAW_FILE	    "/logs/ECGRAW_00.txt"
#define __LOG_ECG_SPO2_RED_FILE	"/logs/SPO2RedRAW_00.txt"
#define __LOG_ECG_SPO2_IR_FILE	"/logs/SPO2IRRAW_00.txt"
/**@}*/

/*************************************************************************************************/
/**
 * @brief Initializes the SD Card and the file system
 * @return FAT FS FRESULT status (refer to FATFS Api)
 */
status_t stFileSystem_Init(void);

/*************************************************************************************************/
/**
 * @brief Opens/Creates a file
 * @param fp File descriptor handler
 * @param path String with the path and file name
 * @param mode Permissions to modify the file (write, read, append ...)
 * @return FAT FS FRESULT status (refer to FATFS Api)
 */
FRESULT stFileSystem_OpenFile(FIL* fp, const TCHAR* path, char* mode);

/*************************************************************************************************/
/**
 * @brief Closes a file
 * @param fp File descriptor handler
 * @return kStatus_Success on success
 */
status_t stFileSystem_CloseFile(FIL* fp);

/*************************************************************************************************/
/**
 * @brief Closes a file
 * @param path String with the path and file name
 * @return FAT FS FRESULT status (refer to FATFS Api)
 */
FRESULT stFileSystem_DeleteFile(const TCHAR* path);

/*************************************************************************************************/
/**
 * @brief Read a buffer from the file
 * @param fp File descriptor handler
 * @param buff Buffer destination
 * @param btr Number of bytes to read
 * @param br Number of bytes read
 * @return FAT FS FRESULT status (refer to FATFS Api)
 */
FRESULT stFileSystem_ReadFile(FIL* fp, const void* buff, UINT btr, UINT* br);

/*************************************************************************************************/
/**
 * @brief Write a buffer to the file
 * @param fp File descriptor handler
 * @param buff Buffer source
 * @param btr Number of bytes to write
 * @param br Number of bytes written
 * @return FAT FS FRESULT status (refer to FATFS Api)
 */
FRESULT stFileSystem_WriteFile(FIL* fp, const void* buff, UINT btw, UINT* bw);

/*************************************************************************************************/
/**
 * @brief Save a print screen of the current screenx'x'
 * @param path String with the path and file name
 * @return kStatus_Success on success
 */
status_t sFileSystem_PrintScreen( const TCHAR* path);
/*************************************************************************************************/

#endif /* APPLICATION_FILESYSTEM_FILESYSTEM_H_ */
