/**
 ******************************************************************************
 * @file    AFE4400.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    5 November 2017
 * @brief   Description: Definition of the AFE4400 library, which implements features of the AFE4400 Pulse Oximetry.
 * @attention !!!This code was ported from https://github.com/mogar/AFE4400/!!!
 ******************************************************************************
 */

#include "AFE4400.h"

#include "fsl_device_registers.h"
#include "fsl_dspi.h"

#include "fsl_common.h"
#include "user.h"
#include "InterruptManager.h"
#include "prioAssigner.h"
#include "SystickUser.h"
/**
 @brief Writes a bit to a specified register

 @param handle AFE4400 driver handle to DSPI related parameters
 @param regAddress Address of the register (available on eAFE4400Registers enum)
 @param bit Bit position on the Register
 @param bit_high True if wants to write a bit high, False otherwise
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
static status_t _sAFE4400_SPIWriteBit(afe4400_handle_t *handle, eAFE4400Registers regAddress, uint8_t bit,
bool bit_high);
/**
 @brief Enable SPI read (registers can be read)

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
static status_t _sAFE4400_EnableSPIRead(afe4400_handle_t *handle);
/**
 @brief Disable SPI read (registers can't be read)

 @param handle AFE4400 driver handle to DSPI related parameters
 @return kStatus_Success if communication was successful, kStatus_Fail Otherwise
 */
static status_t _sAFE4400_DisableSPIRead(afe4400_handle_t *handle);
/**
 @brief Wait Master Complete Event on DSPI

 @param handle AFE4400 driver handle to DSPI related parameters
 @return DSPI event
 */
static uint32_t _u32AFE4400_WaitEvent(afe4400_handle_t *handle);

/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/

/*************************************************************************************************/
status_t sAFE4400_Init(afe4400_handle_t* handle, ARM_SPI_SignalEvent_t cb_event)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    handle->xSemaSignal = xTaskManager_SemaphoreCreate();
    configASSERT(handle->xSemaSignal != NULL);

    handle->dspi_driver.Initialize(cb_event);
    handle->dspi_driver.PowerControl(ARM_POWER_FULL);
    handle->dspi_driver.Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA1 | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8),
            handle->baudrate);

    NVIC_SetPriority(handle->DMATXIRQn, handle->DMAIRQpriority);
    NVIC_SetPriority(handle->DMARXIRQn, handle->DMAIRQpriority);

    sAFE4400_PowerDown(handle);
    sAFE4400_PowerOn(handle);

    sAFE4400_HWReset(handle);

    GPIO_SetPinsOutput(handle->CSGpio, 1U << handle->CSPin);

    if (sAFE4400_SWReset(handle) != kStatus_Success)
    {
        return kStatus_Fail;
    }
#if USE_TEST
    else if (sAFE4400_writeReg(handle, CONTROL0, 0x000000) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, CONTROL0, 0x000008) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, TIAGAIN, 0x000000) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, TIA_AMB_GAIN, 0x000001) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LEDCNTRL, 0x011212) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, CONTROL2, 0x000000) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, CONTROL1, 0x010707) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, PRPCOUNT, PRP) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED2STC, 0X001770) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED2ENDC, 0X001F3E) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED2LEDSTC, 0X001770) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED2LEDENDC, 0X001F3F) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED2STC, 0X000000) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED2ENDC, 0X0007CE) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED2CONVST, 0X000002) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED2CONVEND, 0X0007CF) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED2CONVST, 0X0007D2) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED2CONVEND, 0X000F9F) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED1STC, 0X0007D0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED1ENDC, 0X000F9E) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED1LEDSTC, 0X0007D0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED1LEDENDC, 0X000F9F) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED1STC, 0X000FA0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED1ENDC, 0X00176E) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED1CONVST, 0X000FA2) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, LED1CONVEND, 0X00176F) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED1CONVST, 0X001772) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ALED1CONVEND, 0X001F3F) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTSTCT0, 0X000000) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTENDCT0, 0X000000) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTSTCT1, 0X0007D0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTENDCT1, 0X0007D0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTSTCT2, 0X000FA0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTENDCT2, 0X000FA0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTSTCT3, 0X001770) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    else if (sAFE4400_writeReg(handle, ADCRSTENDCT3, 0X001770) != kStatus_Success)
    {
        return kStatus_Fail;
    }
#else
     else if (sAFE4400_writeReg(handle, CONTROL0, CONTROL0_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, TIAGAIN,
     (ENSEPGAIN + STAGE2EN_LED1 + STG2GAIN_LED1_3DB + CF_LED1_5P + RF_LED1_250K)) != kStatus_Success)
     {     // CF = 5pF, RF = 500kR
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, TIA_AMB_GAIN,
     (AMBDAC_5uA + FLTRCNRSEL_500HZ + STAGE2EN_LED2 + STG2GAIN_LED2_3DB + CF_LED2_5P + RF_LED2_250K))
     != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LEDCNTRL, LEDCNTRL_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, CONTROL2,
     (TX_REF_0 + RST_CLK_ON_PD_ALM_PIN_DISABLE + ADC_BYP_DISABLE + TXBRGMOD_H_BRIDGE + DIGOUT_TRISTATE_DISABLE
     + XTAL_ENABLE + EN_FAST_DIAG + PDN_TX_OFF + PDN_RX_OFF + PDN_AFE_OFF)) != kStatus_Success)
     {     // LED_RANGE=100mA, LED=50mA
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, CONTROL1, CONTROL1_VAL) != kStatus_Success)
     {     // Timers ON, average 3 samples
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, PRPCOUNT, PRP) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED2STC, LED2STC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED2ENDC, LED2ENDC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED2LEDSTC, LED2LEDSTC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED2LEDENDC, LED2LEDENDC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED2STC, ALED2STC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED2ENDC, ALED2ENDC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED2CONVST, LED2CONVST_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED2CONVEND, LED2CONVEND_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED2CONVST, ALED2CONVST_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED2CONVEND, ALED2CONVEND_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED1STC, LED1STC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED1ENDC, LED1ENDC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED1LEDSTC, LED1LEDSTC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED1LEDENDC, LED1LEDENDC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED1STC, ALED1STC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED1ENDC, ALED1ENDC_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED1CONVST, LED1CONVST_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, LED1CONVEND, LED1CONVEND_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED1CONVST, ALED1CONVST_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ALED1CONVEND, ALED1CONVEND_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTSTCT0, ADCRSTSTCT0_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTENDCT0, ADCRSTENDCT0_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTSTCT1, ADCRSTSTCT1_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTENDCT1, ADCRSTENDCT1_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTSTCT2, ADCRSTSTCT2_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTENDCT2, ADCRSTENDCT2_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTSTCT3, ADCRSTSTCT3_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
     else if (sAFE4400_writeReg(handle, ADCRSTENDCT3, ADCRSTENDCT3_VAL) != kStatus_Success)
     {
     return kStatus_Fail;
     }
#endif
    vSystickUser_Delay(1000);

    return kStatus_Success;

}

/*************************************************************************************************/
status_t sAFE4400_Deinit(afe4400_handle_t* handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    handle->dspi_driver.PowerControl(ARM_POWER_OFF);
    handle->dspi_driver.Uninitialize();

    return kStatus_Success;

}

/*****************************************************************************
 **************************** AFE4400  Functions *****************************
 *****************************************************************************/

/*************************************************************************************************/
status_t sAFE4400_readReg(afe4400_handle_t* handle, eAFE4400Registers address, uint32_t* data)
{
    status_t status = kStatus_Success;
    uint8_t dspi_buf[5];
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }
    dspi_buf[0] = address;
    dspi_buf[1] = 0;     //Dummy Byte

    if (_sAFE4400_EnableSPIRead(handle) != kStatus_Success)
    {
        status = kStatus_Fail;
        GPIO_SetPinsOutput(handle->CSGpio, 1U << handle->CSPin);
        return status;
    }

    GPIO_ClearPinsOutput(handle->CSGpio, 1U << handle->CSPin);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {     // send address to device
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[2], 1) != ARM_DRIVER_OK)
    {     // read top 8 bits data
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[3], 1) != ARM_DRIVER_OK)
    {     // read middle 8 bits  data
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Transfer(&dspi_buf[1], &dspi_buf[4], 1) != ARM_DRIVER_OK)
    {     // read bottom 8 bits data
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
        GPIO_SetPinsOutput(handle->CSGpio, 1U << handle->CSPin);
        return status;
    }

    GPIO_SetPinsOutput(handle->CSGpio, 1U << handle->CSPin);

    if (_sAFE4400_DisableSPIRead(handle) != kStatus_Success)
    {
        status = kStatus_Fail;
    }
    handle->dspi_driver.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    *data = (dspi_buf[2] << 16) | (dspi_buf[3] << 8) | dspi_buf[4];
    return status;
}

/*************************************************************************************************/
status_t sAFE4400_writeReg(afe4400_handle_t* handle, eAFE4400Registers address, uint32_t data)
{

    status_t status = kStatus_Success;
    uint8_t dspi_buf[4];
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    dspi_buf[0] = address;
    dspi_buf[1] = (data >> 16) & 0xFF;
    dspi_buf[2] = (data >> 8) & 0xFF;
    dspi_buf[3] = (data) & 0xFF;

    GPIO_ClearPinsOutput(handle->CSGpio, 1U << handle->CSPin);
    if (handle->dspi_driver.Send(&dspi_buf[0], 1) != ARM_DRIVER_OK)
    {     // send address to device
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[1], 1) != ARM_DRIVER_OK)
    {     // write top 8 bits
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[2], 1) != ARM_DRIVER_OK)
    {     // write middle 8 bits
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    else if (handle->dspi_driver.Send(&dspi_buf[3], 1) != ARM_DRIVER_OK)
    {     // write bottom 8 bits
        status = kStatus_Fail;
    }
    else if (_u32AFE4400_WaitEvent(handle) != ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        status = kStatus_Fail;
    }
    GPIO_SetPinsOutput(handle->CSGpio, 1U << handle->CSPin);
    return status;
}

/*************************************************************************************************/
status_t sAFE4400_ReadIRValue(afe4400_handle_t *handle, uint32_t* irvalue)
{
    uint32_t read = 0;
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (sAFE4400_readReg(handle, LED1_ALED1VAL, &read) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (read & 0x00200000)
    {
        read |= 0xFFD00000;
    }
    *irvalue = (int32_t) read;
    return kStatus_Success;
}

/*************************************************************************************************/
status_t sAFE4400_ReadRedValue(afe4400_handle_t *handle, uint32_t* redvalue)
{
    uint32_t read = 0;
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (sAFE4400_readReg(handle, LED2_ALED2VAL, &read) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    if (read & 0x00200000)
    {
        read |= 0xFFD00000;
    }
    *redvalue = (int32_t) read;
    return kStatus_Success;
}

/*************************************************************************************************/
status_t sAFE4400_PowerOn(afe4400_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    GPIO_SetPinsOutput(handle->PDNGpio, 1U << handle->PDNPin);
    vSystickUser_Delay(50);

    return kStatus_Success;
}

/*************************************************************************************************/
status_t sAFE4400_PowerDown(afe4400_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    GPIO_ClearPinsOutput(handle->PDNGpio, 1U << handle->PDNPin);
    vSystickUser_Delay(50);

    return kStatus_Success;
}

/*************************************************************************************************/
status_t sAFE4400_HWReset(afe4400_handle_t *handle)
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
status_t sAFE4400_SWReset(afe4400_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    // write to control0 reg bit 3 1
    // soft reset
    if (_sAFE4400_SPIWriteBit(handle, CONTROL0, 3, true) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    // wait for bit 3 of control0 to be 0
    vSystickUser_Delay(10);
    return kStatus_Success;
}

/*************************************************************************************************/
status_t sAFE4400_setLEDCurrent(afe4400_handle_t *handle, uint32_t led1_current, uint32_t led2_current)
{
    // read the reg
    uint32_t current_val;

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (sAFE4400_readReg(handle, LEDCNTRL, &current_val) != kStatus_Success)
    {
        return kStatus_Fail;
    }

    // set led 1 value
    led1_current = led1_current & 0xFF;
    current_val = current_val & ~(0xFF << 8);
    current_val = current_val | (led1_current << 8);

    // set led 2 value
    led2_current = led2_current & 0xFF;
    current_val = current_val & ~(0xFF);
    current_val = current_val | (led2_current);

    // write reg
    if (sAFE4400_writeReg(handle, LEDCNTRL, current_val) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/***************************** Private Functions *****************************/
static status_t _sAFE4400_SPIWriteBit(afe4400_handle_t *handle, eAFE4400Registers regAddress, uint8_t bit,
bool bit_high)
{
    status_t status = kStatus_Success;
    uint32_t current_val;
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    // read the reg
    if (sAFE4400_readReg(handle, regAddress, &current_val) != kStatus_Success)
    {
        status = kStatus_Fail;
    }
    else
    {
        // check to see if we need to change the bit
        if (bit_high & !(current_val & 1 << bit))
        {
            // set bit correct in reg
            current_val = current_val | (bit_high << bit);

            // write reg
            if (sAFE4400_writeReg(handle, regAddress, current_val) != kStatus_Success)
            {
                status = kStatus_Fail;
            }
        }
        else if ((!bit_high) & (current_val & (1 << bit)))
        {
            // set bit correct in reg
            current_val = current_val & ~(bit_high << bit);

            // write reg
            if (sAFE4400_writeReg(handle, regAddress, current_val) != kStatus_Success)
            {
                status = kStatus_Fail;
            }
        }

    }
    return status;
}

/*************************************************************************************************/
static status_t _sAFE4400_EnableSPIRead(afe4400_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (sAFE4400_writeReg(handle, CONTROL0, 1) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/*************************************************************************************************/
static status_t _sAFE4400_DisableSPIRead(afe4400_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    if (sAFE4400_writeReg(handle, CONTROL0, 0) != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

/*************************************************************************************************/
void vAFE4400_EventHandler(afe4400_handle_t *handle, uint32_t dspi_event)
{
    if (!handle)
    {
        return;
    }

    handle->dspi_event = dspi_event;
    handle->dspi_event_received = true;
    xTaskManager_SemaphoreGive(handle->xSemaSignal);
}

/*************************************************************************************************/
static uint32_t _u32AFE4400_WaitEvent(afe4400_handle_t *handle)
{
    uint32_t dspi_event;

    if (!handle)
    {
        return 0xFF;
    }

    xTaskManager_SemaphoreTake(handle->xSemaSignal, portMAX_DELAY);
//    while (!(handle->dspi_event_received));

    dspi_event = handle->dspi_event;
    handle->dspi_event_received = false;

    return dspi_event;
}
/*************************************************************************************************/
