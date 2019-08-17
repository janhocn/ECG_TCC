/**
 ******************************************************************************
 * @file    SPO2.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 November 2017
 * @brief   SPO2 Application module
 ******************************************************************************
 */

#include "SPO2.h"

#include "fsl_gpio.h"
#include "fsl_device_registers.h"
#include "fsl_dspi.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "user.h"
#include "prioAssigner.h"

/*************************************************************************************************/
/**
 * @var AFE4400 driver handler
 */
static afe4400_handle_t afeHandle;

/*************************************************************************************************/
/**
 * @var signals driver corrected initialized
 */
static bool isDriverInitialized = false;

/*************************************************************************************************/
/**
 * @def CMSIS SPI Driver
 */
#define AFE4400_SPI_Driver Driver_SPI1

/*************************************************************************************************/
/**
 * @def SPI 1 IRQn
 */
#define AFE4400_DSPI_IRQn SPI1_IRQn

/*************************************************************************************************/
/**
 * @def DMA TX Channel IRQn
 */
#define AFE4400_DSPI_DMA_TX_IRQn DMA3_IRQn


/*************************************************************************************************/
/**
 * @def DMA RX Channel IRQn
 */
#define AFE4400_DSPI_DMA_RX_IRQn DMA4_IRQn

/**
 @def DSPI default transfer frequency for AFE4400
 */
#define AFE4400_TRANSFER_BAUDRATE 2000000U /*! Transfer baudrate - 2M */

/*************************************************************************************************/
/**
 * @brief Handles the spi event signal
 * @param event Signal coming from spi CMSIS IRQ handler
 * @return void
 */
static void _vSPO2_DSPIMasterSignalEvent(uint32_t event);

/*************************************************************************************************/
/**
 * @brief For test purposes only (Read all AFE4400 registers)
 * @return void
 */
void vSPO2_ReadAllRegs(void);

/*************************************************************************************************/
static void _vSPO2_DSPIMasterSignalEvent(uint32_t event)
{
    vAFE4400_EventHandler(&afeHandle,event);
}

/*************************************************************************************************/
void vSPO2_Init(void)
{
    status_t err;
    memcpy((void * restrict) &afeHandle.dspi_driver, (const void * restrict) &AFE4400_SPI_Driver, sizeof(ARM_DRIVER_SPI));
    afeHandle.DMATXIRQn = AFE4400_DSPI_DMA_TX_IRQn;
    afeHandle.DMARXIRQn = AFE4400_DSPI_DMA_RX_IRQn;
    afeHandle.DMAIRQpriority = AFE4400_SPI_INT_PRIO;
    afeHandle.RSTGpio = GPIO_AFE4400_RST_GPIO;
    afeHandle.RSTPin = GPIO_AFE4400_RST_PIN;
    afeHandle.PDNGpio = GPIO_AFE4400_PDN_GPIO;
    afeHandle.PDNPin = GPIO_AFE4400_PDN_PIN;
    afeHandle.CSGpio = GPIO_AFE4400_CS_GPIO;
    afeHandle.CSPin = GPIO_AFE4400_CS_PIN;
    afeHandle.baudrate = AFE4400_TRANSFER_BAUDRATE;

    err = sAFE4400_Init(&afeHandle, _vSPO2_DSPIMasterSignalEvent);

    if (err == kStatus_Success)
    {
        isDriverInitialized = true;
    }
}

/*************************************************************************************************/
status_t sSPO2_ReadRedLED(uint32_t * readValue)
{
    status_t err;
    if (!isDriverInitialized)
    {
        return kStatus_Fail;
    }

    err = sAFE4400_ReadRedValue(&afeHandle, readValue);
    return err;
}

/*************************************************************************************************/
status_t sSPO2_ReadIRLED(uint32_t * readValue)
{
    status_t err;
    if (!isDriverInitialized)
    {
        return kStatus_Fail;
    }

    err = sAFE4400_ReadIRValue(&afeHandle, readValue);
    return err;
}

/*************************************************************************************************/
status_t sSPO2_PowerControl(eSPO2PowerControl_t _pwc)
{
    status_t err = kStatus_Fail;
    if (!isDriverInitialized)
    {
        return kStatus_Fail;
    }
    switch(_pwc)
    {
        case _E_POWER_ON:
            err = sAFE4400_PowerOn(&afeHandle);
            break;
        case _E_POWER_OFF:
        default:
            err = sAFE4400_PowerDown(&afeHandle);
            break;
    }
    return err;
}

/*************************************************************************************************/
void vSPO2_ReadAllRegs(void)
{
    eAFE4400Registers Regs_i;
    uint32_t AFE44xxeg_buf[50];
    for (Regs_i = 0; Regs_i < 50; Regs_i++)
    {
        sAFE4400_readReg(&afeHandle, Regs_i, &AFE44xxeg_buf[Regs_i]);
    }
}
/*************************************************************************************************/
