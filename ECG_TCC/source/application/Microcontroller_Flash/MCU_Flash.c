/**
 ******************************************************************************
 * @file    MCU_Flash.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    10 January 2018
 * @brief   MCU Flash Module
 ******************************************************************************
 */
#include "MCU_Flash.h"

#include "fsl_flash.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include <stdlib.h>
/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to this module
 */
static struct _sMCUFlash
{
    flash_config_t s_flashDriver; /** @< Flash driver Structure */
    bool bFalshInitialized; /** @< Indicates that the driver has been initialized */
    uint32_t spflashBlockBase; /** @< Flash Block Base Address */
    uint32_t spflashTotalSize; /** @< Flash Total Size in bytes */
    uint32_t spflashSectorSize; /** @< Flash Sector Size in bytes */
    uint32_t u32flashUserDataBaseAddr; /** @< Flash User region start address */
    uint32_t su32failAddr; /** @< Holds the failed address */
    uint32_t su32failDat; /** @< Holds the failed data */
    uint32_t su32writeBufferHelper[FSL_FEATURE_FLASH_PFLASH_RESOURCE_CMD_ADDRESS_ALIGMENT / 4]; /** @< Helps to write 8 byte aligned data*/
    uint8_t psu8ReadBuffer[FLASH_SECTOR_SIZE]; /** @< Pointer to buffern allocation for reading operations */
} sMCUFlash;

/*************************************************************************************************/
status_t stMCU_Flash_Init(void)
{
    status_t result; /* Return code from each flash driver function */

    /* Clean up Flash driver Structure*/
    memset(&sMCUFlash.s_flashDriver, 0, sizeof(flash_config_t));
    /* Setup flash driver structure for device and initialize variables. */
    result = FLASH_Init(&sMCUFlash.s_flashDriver);
    if (kStatus_FLASH_Success != result)
    {
        return kStatus_Fail;
    }

    sMCUFlash.bFalshInitialized = true;

    vMCU_Flash_getProperties(&sMCUFlash.spflashBlockBase, &sMCUFlash.spflashTotalSize, &sMCUFlash.spflashSectorSize);

#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
    /* Note: we should make sure that the sector shouldn't be swap indicator sector*/
    sMCUFlash.u32flashUserDataBaseAddr = sMCUFlash.spflashBlockBase
            + (sMCUFlash.spflashTotalSize - (sMCUFlash.spflashSectorSize * 2));
#else
    sMCUFlash.u32flashUserDataBaseAddr = sMCUFlash.spflashBlockBase + (sMCUFlash.spflashTotalSize - sMCUFlash.spflashSectorSize);
#endif

    return kStatus_FLASH_Success;
}

/*************************************************************************************************/
void vMCU_Flash_getProperties(uint32_t* pflashBlockBase, uint32_t* pflashTotalSize, uint32_t* pflashSectorSize)
{
    if (sMCUFlash.bFalshInitialized)
    {
        /* Get flash properties*/
        FLASH_GetProperty(&sMCUFlash.s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, pflashBlockBase);
        FLASH_GetProperty(&sMCUFlash.s_flashDriver, kFLASH_PropertyPflashTotalSize, pflashTotalSize);
        FLASH_GetProperty(&sMCUFlash.s_flashDriver, kFLASH_PropertyPflashSectorSize, pflashSectorSize);
    }
}

/*************************************************************************************************/
status_t stMCU_Flash_retrieveKnownData(eflashUserData data, const void * dataRead)
{
    if (!sMCUFlash.bFalshInitialized)
    {
        return kStatus_Fail;
    }

    switch (data)
    {
        default:
        case _FLASH_USER_LANGUAGE:
        {
            eLanguage LagRead;
            /* Verify programming by reading back from flash directly*/
            for (uint8_t i = 0; i < (FSL_FEATURE_FLASH_PFLASH_RESOURCE_CMD_ADDRESS_ALIGMENT / 4); i++)
            {
                sMCUFlash.su32writeBufferHelper[i] = *(volatile uint32_t *) ((sMCUFlash.u32flashUserDataBaseAddr
                        + FLASH_LANGUAGE_START_ADDR) + i * 4);
            }
            LagRead = (eLanguage) sMCUFlash.su32writeBufferHelper[0];
            *(eLanguage *) dataRead = LagRead;
        }
    }
    return kStatus_Success;
}

/*************************************************************************************************/
status_t stMCU_Flash_saveKnownData(eflashUserData data, void* value)
{
    status_t result;
    if (!sMCUFlash.bFalshInitialized)
    {
        return kStatus_Fail;
    }

//    sMCUFlash.psu8ReadBuffer = (uint8_t *) malloc(sMCUFlash.s_flashDriver.PFlashSectorSize);
//    if (sMCUFlash.psu8ReadBuffer == NULL)
//    {
//        return kStatus_Fail;
//    }

    /* Copies the entire sector to the backup buffer */
    memcpy(sMCUFlash.psu8ReadBuffer, (uint32_t *) sMCUFlash.u32flashUserDataBaseAddr,
            sMCUFlash.s_flashDriver.PFlashSectorSize);

    switch (data)
    {
        default:
        case _FLASH_USER_LANGUAGE:
        {
            eLanguage LagWrite;
            LagWrite = *(eLanguage *) value;

            sMCUFlash.psu8ReadBuffer[FLASH_LANGUAGE_START_ADDR] = (uint8_t) LagWrite;

            /* Erase the sector before programming */
            result = FLASH_Erase(&sMCUFlash.s_flashDriver, sMCUFlash.u32flashUserDataBaseAddr,
                    sMCUFlash.s_flashDriver.PFlashSectorSize, kFLASH_ApiEraseKey);
            if (kStatus_FLASH_Success != result)
            {
                goto _END;
            }

            result = stMCU_Flash_Program(sMCUFlash.u32flashUserDataBaseAddr, (uint32_t *) sMCUFlash.psu8ReadBuffer,
                    sMCUFlash.s_flashDriver.PFlashSectorSize);
            break;
        }
    }

    _END:
//    free(sMCUFlash.psu8ReadBuffer);
    return result;
}

/*************************************************************************************************/
uint32_t stMCU_Flash_ReadU32(uint32_t destAdrss)
{
    return *(volatile uint32_t *) (destAdrss);
}

/*************************************************************************************************/
status_t stMCU_Flash_Program(uint32_t destAdrss, uint32_t *buffer, uint32_t lengthInBytes)
{

    status_t result; /* Return code from each flash driver function */
    flash_security_state_t securityStatus = kFLASH_SecurityStateNotSecure; /* Return protection status */

    if (!sMCUFlash.bFalshInitialized)
    {
        return kStatus_Fail;
    }

    sMCUFlash.su32failAddr = 0;
    sMCUFlash.su32failDat = 0;
    /* Check security status. */
    result = FLASH_GetSecurityState(&sMCUFlash.s_flashDriver, &securityStatus);
    if (kStatus_FLASH_Success != result)
    {
        return kStatus_Fail;
    }

    /* Test pflash basic opeation only if flash is unsecure. */
    if (kFLASH_SecurityStateNotSecure == securityStatus)
    {

        /* Program user buffer into flash*/
        result = FLASH_Program(&sMCUFlash.s_flashDriver, destAdrss, buffer, lengthInBytes);
        if (kStatus_FLASH_Success != result)
        {
            return kStatus_Fail;
        }

        /* Verify programming by Program Check command with user margin levels */
        result = FLASH_VerifyProgram(&sMCUFlash.s_flashDriver, destAdrss, lengthInBytes, buffer, kFLASH_MarginValueUser,
                &sMCUFlash.su32failAddr, &sMCUFlash.su32failDat);
        if (kStatus_FLASH_Success != result)
        {
            return kStatus_Fail;
        }

#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT
        /* Clean the D-Cache before reading the flash data*/
        SCB_CleanInvalidateDCache();
#endif
    }
    return kStatus_Success;
}
/*************************************************************************************************/
