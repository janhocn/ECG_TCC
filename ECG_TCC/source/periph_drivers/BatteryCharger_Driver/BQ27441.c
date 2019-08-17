/**
 ******************************************************************************
 * @file    BQ27441.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    22 October 2017
 * @brief   Implementation of all features of the BQ27441 LiPo Fuel Gauge.
 * @attention !!!This code was ported from https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library!!!
 ******************************************************************************
 */

#include "BQ27441.h"

#include "BQ27441_Defs.h"
#include "fsl_common.h"
#include "user.h"
#include "SystickUser.h"

static bool _sealFlag; /** @var Global to identify that IC was previously sealed*/
static bool _userConfigControl; /** @var Global to identify that user has control over entering/exiting config */

/**
 @brief Check if the BQ27441-G1A is sealed or not.

 @param BQ27441 driver handle to I2C related parameters
 @return true if the chip is sealed
 */
static bool _bBQ27441_isSealed(bq27441_handle_t *handle);

/**
 @brief Seal the BQ27441-G1A

 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
static bool _bBQ27441_setSeal(bq27441_handle_t *handle);

/**
 @brief UNseal the BQ27441-G1A

 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
static bool _bBQ27441_setUnseal(bq27441_handle_t *handle);

/**
 @brief Read the 16-bit opConfig register from extended data

 @param BQ27441 driver handle to I2C related parameters
 @return opConfig register contents
 */
static uint16_t _u16BQ27441_getOpConfig(bq27441_handle_t *handle);

/**
 @brief Write the 16-bit opConfig register in extended data

 @param BQ27441 driver handle to I2C related parameters
 @param New 16-bit value for opConfig
 @return true on success
 */
static bool _bBQ27441_writeOpConfig(bq27441_handle_t *handle, uint16_t value);

/**
 @brief Issue a soft-reset to the BQ27441-G1A

 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
static bool _bBQ27441_softReset(bq27441_handle_t *handle);

/**
 @brief Read a 16-bit command word from the BQ27441-G1A

 @param BQ27441 driver handle to I2C related parameters
 @param subAddress is the command to be read from
 @return 16-bit value of the command's contents
 */
static uint16_t _u16BQ27441_readWord(bq27441_handle_t *handle, uint16_t subAddress);

/**
 @brief Read a 16-bit subcommand() from the BQ27441-G1A's control()

 @param BQ27441 driver handle to I2C related parameters
 @param function is the subcommand of control() to be read
 @return 16-bit value of the subcommand's contents
 */
static uint16_t _u16BQ27441_readControlWord(bq27441_handle_t *handle, uint16_t function);

/**
 @brief Execute a subcommand() from the BQ27441-G1A's control()

 @param BQ27441 driver handle to I2C related parameters
 @param function is the subcommand of control() to be executed
 @return true on success
 */
static bool _bBQ27441_executeControlWord(bq27441_handle_t *handle, uint16_t function);

////////////////////////////
// Extended Data Commands //
////////////////////////////
/**
 @brief Issue a BlockDataControl() command to enable BlockData access

 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
static bool _bBQ27441_blockDataControl(bq27441_handle_t *handle);

/**
 @brief Issue a DataClass() command to set the data class to be accessed

 @param BQ27441 driver handle to I2C related parameters
 @param id is the id number of the class
 @return true on success
 */
static bool _bBQ27441_blockDataClass(bq27441_handle_t *handle, uint8_t id);

/**
 @brief Issue a DataBlock() command to set the data block to be accessed

 @param BQ27441 driver handle to I2C related parameters
 @param offset of the data block
 @return true on success
 */
static bool _BQ27441_blockDataOffset(bq27441_handle_t *handle, uint8_t offset);

/**
 @brief Read the current checksum using BlockDataCheckSum()

 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
static uint8_t _u8BQ27441_blockDataChecksum(bq27441_handle_t *handle);

/**
 @brief Use BlockData() to read a byte from the loaded extended data

 @param BQ27441 driver handle to I2C related parameters
 @param offset of data block byte to be read
 @return true on success
 */
static uint8_t _u8BQ27441_readBlockData(bq27441_handle_t *handle, uint8_t offset);

/**
 @brief Use BlockData() to write a byte to an offset of the loaded data

 @param BQ27441 driver handle to I2C related parameters
 @param offset is the position of the byte to be written
 data is the value to be written
 @return true on success
 */
static bool _bBQ27441_writeBlockData(bq27441_handle_t *handle, uint8_t offset, uint8_t data);

/**
 @brief Read all 32 bytes of the loaded extended data and compute a
 @brief checksum based on the values.

 @param BQ27441 driver handle to I2C related parameters
 @return 8-bit checksum value calculated based on loaded data
 */
static uint8_t _u8BQ27441_computeBlockChecksum(bq27441_handle_t *handle);

/**
 @brief Use the BlockDataCheckSum() command to write a checksum value

 @param BQ27441 driver handle to I2C related parameters
 @param csum is the 8-bit checksum to be written
 @return true on success
 */
static bool _bBQ27441_writeBlockChecksum(bq27441_handle_t *handle, uint8_t csum);

/**
 @brief Read a byte from extended data specifying a class ID and position offset

 @param BQ27441 driver handle to I2C related parameters
 @param classID is the id of the class to be read from
 offset is the byte position of the byte to be read
 @return 8-bit value of specified data
 */
static uint8_t _u8BQ27441_readExtendedData(bq27441_handle_t *handle, uint8_t classID, uint8_t offset);

/**
 @brief Write a specified number of bytes to extended data specifying a
 @brief class ID, position offset.

 @param BQ27441 driver handle to I2C related parameters
 @param classID is the id of the class to be read from
 offset is the byte position of the byte to be read
 data is the data buffer to be written
 len is the number of bytes to be written
 @return true on success
 */
static bool _bBQ27441_writeExtendedData(
        bq27441_handle_t *handle,
        uint8_t classID,
        uint8_t offset,
        uint8_t * data,
        uint8_t len);

/////////////////////////////////
// I2C Read and Write Routines //
/////////////////////////////////

/**
 @brief Read a specified number of bytes over I2C at a given subAddress

 @param BQ27441 driver handle to I2C related parameters
 @param subAddress is the 8-bit address of the data to be read
 dest is the data buffer to be written to
 count is the number of bytes to be read
 @return true on success
 */
static status_t _stBQ27441_i2cReadBytes(bq27441_handle_t *handle, uint8_t subAddress, uint8_t * dest, uint8_t count);

/**
 @brief Write a specified number of bytes over I2C to a given subAddress

 @param BQ27441 driver handle to I2C related parameters
 @param subAddress is the 8-bit address of the data to be written to
 src is the data buffer to be written
 count is the number of bytes to be written
 @return true on success
 */
static status_t _stBQ27441_i2cWriteBytes(bq27441_handle_t *handle, uint8_t subAddress, uint8_t * src, uint8_t count);

/**
 @brief Wait Master Complete Event on I2C

 @param handle BQ27441 handle to I2C related parameters
 @return I2C event
 */
static uint32_t _u32BQ27441_WaitEvent(bq27441_handle_t *handle);
/////////////////////////
// Utilities  Routines //
/////////////////////////

/**
 @brief Constrains a number to be within a range.

 @param val the 8bit number to constrain
 @param min the lower end of the range
 @param max the upper end of the range
 @return val: if val is between min and max
 min: if val is less than min
 max: if val is greater than max
 */
static uint8_t _constrain(uint8_t val, uint8_t min, uint8_t max);
/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/
/*************************************************************************************************/
status_t stBQ27441_Init(bq27441_handle_t *handle, ARM_I2C_SignalEvent_t cb_event)
{
    uint16_t deviceID = 0;

    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    NVIC_SetPriority(handle->I2CIRQn, handle->I2CIRQpriority);

    _sealFlag = false;
    _userConfigControl = false;

    handle->xSemaSignal = xTaskManager_SemaphoreCreate();
    configASSERT(handle->xSemaSignal != NULL);

    /*Init I2C1 */
    handle->i2c_driver.Initialize(cb_event);
    handle->i2c_driver.PowerControl(ARM_POWER_FULL);
    handle->i2c_driver.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);

    deviceID = u16BQ27441_getDeviceType(handle);     // Read deviceType from BQ27441

    if (deviceID == BQ27441_DEVICE_ID)
    {
        return kStatus_Success;     // If device ID is valid, return true
    }

    return kStatus_Fail;     // Otherwise return false
}

/*************************************************************************************************/
status_t stBQ27441_Deinit(bq27441_handle_t *handle)
{
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    handle->i2c_driver.PowerControl(ARM_POWER_OFF);
    handle->i2c_driver.Uninitialize();

    return kStatus_Success;
}

/*************************************************************************************************/
bool bBQ27441_setCapacity(bq27441_handle_t *handle, uint16_t capacity)
{
    // Write to STATE subclass (82) of BQ27441 extended memory.
    // Offset 0x0A (10)
    // Design capacity is a 2-byte piece of data - MSB first
    uint8_t capMSB = capacity >> 8;
    uint8_t capLSB = capacity & 0x00FF;
    uint8_t capacityData[2] = { capMSB, capLSB };
    return _bBQ27441_writeExtendedData(handle, BQ27441_ID_STATE, 10, capacityData, 2);
}

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

/*************************************************************************************************/
uint16_t u16BQ27441_getVoltage(bq27441_handle_t *handle)
{
    return _u16BQ27441_readWord(handle, BQ27441_COMMAND_VOLTAGE);
}

/*************************************************************************************************/
int16_t i16BQ27441_getCurrent(bq27441_handle_t *handle, current_measure type)
{
    int16_t current = 0;
    switch (type)
    {
        case AVG:
            current = (int16_t) _u16BQ27441_readWord(handle, BQ27441_COMMAND_AVG_CURRENT);
            break;
        case STBY:
            current = (int16_t) _u16BQ27441_readWord(handle, BQ27441_COMMAND_STDBY_CURRENT);
            break;
        case MAX:
            current = (int16_t) _u16BQ27441_readWord(handle, BQ27441_COMMAND_MAX_CURRENT);
            break;
    }

    return current;
}

/*************************************************************************************************/
uint16_t u16BQ27441_getCapacity(bq27441_handle_t *handle, capacity_measure type)
{
    uint16_t capacity = 0;
    switch (type)
    {
        case REMAIN:
            return _u16BQ27441_readWord(handle, BQ27441_COMMAND_REM_CAPACITY);
            break;
        case FULL:
            return _u16BQ27441_readWord(handle, BQ27441_COMMAND_FULL_CAPACITY);
            break;
        case AVAIL:
            capacity = _u16BQ27441_readWord(handle, BQ27441_COMMAND_NOM_CAPACITY);
            break;
        case AVAIL_FULL:
            capacity = _u16BQ27441_readWord(handle, BQ27441_COMMAND_AVAIL_CAPACITY);
            break;
        case REMAIN_F:
            capacity = _u16BQ27441_readWord(handle, BQ27441_COMMAND_REM_CAP_FIL);
            break;
        case REMAIN_UF:
            capacity = _u16BQ27441_readWord(handle, BQ27441_COMMAND_REM_CAP_UNFL);
            break;
        case FULL_F:
            capacity = _u16BQ27441_readWord(handle, BQ27441_COMMAND_FULL_CAP_FIL);
            break;
        case FULL_UF:
            capacity = _u16BQ27441_readWord(handle, BQ27441_COMMAND_FULL_CAP_UNFL);
            break;
        case DESIGN:
            capacity = _u16BQ27441_readWord(handle, BQ27441_EXTENDED_CAPACITY);
    }

    return capacity;
}

/*************************************************************************************************/
int16_t i16BQ27441_getPower(bq27441_handle_t *handle)
{
    return (int16_t) _u16BQ27441_readWord(handle, BQ27441_COMMAND_AVG_POWER);
}

/*************************************************************************************************/
uint16_t u16BQ27441_getSOC(bq27441_handle_t *handle, soc_measure type)
{
    uint16_t socRet = 0;
    switch (type)
    {
        case FILTERED:
            socRet = _u16BQ27441_readWord(handle, BQ27441_COMMAND_SOC);
            break;
        case UNFILTERED:
            socRet = _u16BQ27441_readWord(handle, BQ27441_COMMAND_SOC_UNFL);
            break;
    }

    return socRet;
}

/*************************************************************************************************/
uint8_t u8BQ27441_getSOH(bq27441_handle_t *handle, soh_measure type)
{
    uint16_t sohRaw = _u16BQ27441_readWord(handle, BQ27441_COMMAND_SOH);
    uint8_t sohStatus = sohRaw >> 8;
    uint8_t sohPercent = sohRaw & 0x00FF;

    if (type == PERCENT)
        return sohPercent;
    else
        return sohStatus;
}

/*************************************************************************************************/
uint16_t u16BQ27441_getTemperature(bq27441_handle_t *handle, temp_measure type)
{
    uint16_t temp = 0;
    switch (type)
    {
        case BATTERY:
            temp = _u16BQ27441_readWord(handle, BQ27441_COMMAND_TEMP);
            break;
        case INTERNAL_TEMP:
            temp = _u16BQ27441_readWord(handle, BQ27441_COMMAND_INT_TEMP);
            break;
    }
    return temp;
}

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/
/*************************************************************************************************/
bool bBQ27441_getGPOUTPolarity(bq27441_handle_t *handle)
{
    uint16_t opConfigRegister = _u16BQ27441_getOpConfig(handle);

    return (opConfigRegister & BQ27441_OPCONFIG_GPIOPOL);
}

/*************************************************************************************************/
bool bBQ27441_setGPOUTPolarity(bq27441_handle_t *handle, bool activeHigh)
{
    uint16_t oldOpConfig = _u16BQ27441_getOpConfig(handle);

    // Check to see if we need to update opConfig:
    if ((activeHigh && (oldOpConfig & BQ27441_OPCONFIG_GPIOPOL))
            || (!activeHigh && !(oldOpConfig & BQ27441_OPCONFIG_GPIOPOL)))
        return true;

    uint16_t newOpConfig = oldOpConfig;
    if (activeHigh)
        newOpConfig |= BQ27441_OPCONFIG_GPIOPOL;
    else
        newOpConfig &= ~(BQ27441_OPCONFIG_GPIOPOL);

    return _bBQ27441_writeOpConfig(handle, newOpConfig);
}

/*************************************************************************************************/
bool bBQ27441_getGPOUTFunction(bq27441_handle_t *handle)
{
    uint16_t opConfigRegister = _u16BQ27441_getOpConfig(handle);

    return (opConfigRegister & BQ27441_OPCONFIG_BATLOWEN);
}

/*************************************************************************************************/
bool bBQ27441_setGPOUTFunction(bq27441_handle_t *handle, gpout_function function)
{
    uint16_t oldOpConfig = _u16BQ27441_getOpConfig(handle);

    // Check to see if we need to update opConfig:
    if ((function && (oldOpConfig & BQ27441_OPCONFIG_BATLOWEN))
            || (!function && !(oldOpConfig & BQ27441_OPCONFIG_BATLOWEN)))
        return true;

    // Modify BATLOWN_EN bit of opConfig:
    uint16_t newOpConfig = oldOpConfig;
    if (function)
        newOpConfig |= BQ27441_OPCONFIG_BATLOWEN;
    else
        newOpConfig &= ~(BQ27441_OPCONFIG_BATLOWEN);

    // Write new opConfig
    return _bBQ27441_writeOpConfig(handle, newOpConfig);
}

/*************************************************************************************************/
uint8_t u8BQ27441_getSOC1SetThreshold(bq27441_handle_t *handle)
{
    return _u8BQ27441_readExtendedData(handle, BQ27441_ID_DISCHARGE, 0);
}

/*************************************************************************************************/
uint8_t u8BQ27441_getSOC1ClearThresholdd(bq27441_handle_t *handle)
{
    return _u8BQ27441_readExtendedData(handle, BQ27441_ID_DISCHARGE, 1);
}

/*************************************************************************************************/
bool bBQ27441_setSOC1Thresholds(bq27441_handle_t *handle, uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = _constrain(set, 0, 100);
    thresholds[1] = _constrain(clear, 0, 100);
    return _bBQ27441_writeExtendedData(handle, BQ27441_ID_DISCHARGE, 0, thresholds, 2);
}

/*************************************************************************************************/
uint8_t u8BQ27441_getSOCFSetThreshold(bq27441_handle_t *handle)
{
    return _u8BQ27441_readExtendedData(handle, BQ27441_ID_DISCHARGE, 2);
}

/*************************************************************************************************/
uint8_t u8BQ27441_getSOCFClearThreshold(bq27441_handle_t *handle)
{
    return _u8BQ27441_readExtendedData(handle, BQ27441_ID_DISCHARGE, 3);
}

/*************************************************************************************************/
bool bBQ27441_setSOCFThresholds(bq27441_handle_t *handle, uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = _constrain(set, 0, 100);
    thresholds[1] = _constrain(clear, 0, 100);
    return _bBQ27441_writeExtendedData(handle, BQ27441_ID_DISCHARGE, 2, thresholds, 2);
}

/*************************************************************************************************/
bool bBQ27441_getSOCFlag(bq27441_handle_t *handle)
{
    uint16_t flagState = u16BQ27441_getFlags(handle);

    return flagState & BQ27441_FLAG_SOC1;
}

/*************************************************************************************************/
bool bBQ27441_getSOCFFlag(bq27441_handle_t *handle)
{
    uint16_t flagState = u16BQ27441_getFlags(handle);

    return flagState & BQ27441_FLAG_SOCF;

}

/*************************************************************************************************/
uint8_t u8BQ27441_getSOCIDelta(bq27441_handle_t *handle)
{
    return _u8BQ27441_readExtendedData(handle, BQ27441_ID_STATE, 26);
}

/*************************************************************************************************/
bool bBQ27441_setSOCIDelta(bq27441_handle_t *handle, uint8_t delta)
{
    uint8_t soci = _constrain(delta, 0, 100);
    return _bBQ27441_writeExtendedData(handle, BQ27441_ID_STATE, 26, &soci, 1);
}

/*************************************************************************************************/
bool bBQ27441_pulseGPOUT(bq27441_handle_t *handle)
{
    return _bBQ27441_executeControlWord(handle, BQ27441_CONTROL_PULSE_SOC_INT);
}

/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/

/*************************************************************************************************/
uint16_t u16BQ27441_getDeviceType(bq27441_handle_t *handle)
{
    return _u16BQ27441_readControlWord(handle, BQ27441_CONTROL_DEVICE_TYPE);
}

/*************************************************************************************************/
bool bBQ27441_enterConfig(bq27441_handle_t *handle, bool userControl)
{
    if (userControl)
        _userConfigControl = true;

    if (_bBQ27441_isSealed(handle))
    {
        _sealFlag = true;
        _bBQ27441_setUnseal(handle);     // Must be unsealed before making changes
    }

    if (_bBQ27441_executeControlWord(handle, BQ27441_CONTROL_SET_CFGUPDATE))
    {
        int16_t timeout = BQ27441_I2C_TIMEOUT;
        int16_t timeStart = u32SystickUser_getTick();

        while (u16BQ27441_getStatus(handle) & BQ27441_FLAG_CFGUPMODE)
        {
            if (bSystickUser_Wait(timeStart, timeout))
            {
                return false;
            }
        }
        return true;
    }

    return false;
}

/*************************************************************************************************/
bool bBQ27441_exitConfig(bq27441_handle_t *handle, bool resim)
{
    // There are two methods for exiting config mode:
    //    1. Execute the EXIT_CFGUPDATE command
    //    2. Execute the SOFT_RESET command
    // EXIT_CFGUPDATE exits config mode _without_ an OCV (open-circuit voltage)
    // measurement, and without resimulating to update unfiltered-SoC and SoC.
    // If a new OCV measurement or resimulation is desired, SOFT_RESET or
    // EXIT_RESIM should be used to exit config mode.
    if (resim)
    {
        if (_bBQ27441_softReset(handle))
        {
            int16_t timeout = BQ27441_I2C_TIMEOUT;
            int16_t timeStart = u32SystickUser_getTick();

            while (((u16BQ27441_getFlags(handle) & BQ27441_FLAG_CFGUPMODE)))
            {
                if (bSystickUser_Wait(timeStart, timeout))
                {
                    return false;
                }
            }

            if (_sealFlag)
                _bBQ27441_setSeal(handle);     // Seal back up if the IC was sealed coming in
            return true;
        }
        return false;
    }
    else
    {
        return _bBQ27441_executeControlWord(handle, BQ27441_CONTROL_EXIT_CFGUPDATE);
    }
}

/*************************************************************************************************/
uint16_t u16BQ27441_getFlags(bq27441_handle_t *handle)
{
    return _u16BQ27441_readWord(handle, BQ27441_COMMAND_FLAGS);
}

/*************************************************************************************************/
uint16_t u16BQ27441_getStatus(bq27441_handle_t *handle)
{
    return _u16BQ27441_readControlWord(handle, BQ27441_CONTROL_STATUS);
}

/*************************************************************************************************/
bool bBQ27441_makeADCReading(bq27441_handle_t *handle)
{
    return _bBQ27441_executeControlWord(handle, BQ27441_CONTROL_CLEAR_HIBERNATE);
}

/*************************************************************************************************/
void vBQ27441_EventHandler(bq27441_handle_t *handle, uint32_t i2c_event)
{
    handle->i2c_event = i2c_event;
    handle->i2c_event_received = true;
    xTaskManager_SemaphoreGive(handle->xSemaSignal);
}

/***************************** Private Functions *****************************/
/*************************************************************************************************/
static bool _bBQ27441_isSealed(bq27441_handle_t *handle)
{
    uint16_t stat = u16BQ27441_getStatus(handle);
    return stat & BQ27441_STATUS_SS;
}

/*************************************************************************************************/
static bool _bBQ27441_setSeal(bq27441_handle_t *handle)
{
    return _u16BQ27441_readControlWord(handle, BQ27441_CONTROL_SEALED);
}

/*************************************************************************************************/
static bool _bBQ27441_setUnseal(bq27441_handle_t *handle)
{
    // To unseal the BQ27441, write the key to the control
    // command. Then immediately write the same key to control again.
    if (_u16BQ27441_readControlWord(handle, BQ27441_UNSEAL_KEY))
    {
        return _u16BQ27441_readControlWord(handle, BQ27441_UNSEAL_KEY);
    }
    return false;
}

/*************************************************************************************************/
static uint16_t _u16BQ27441_getOpConfig(bq27441_handle_t *handle)
{
    return _u16BQ27441_readWord(handle, BQ27441_EXTENDED_OPCONFIG);
}

/*************************************************************************************************/
bool bBQ27441_writeOpConfigB(bq27441_handle_t *handle, uint16_t value)
{
    uint8_t opConfigLSB = value & 0x00FF;
    uint8_t opConfigData[1] = { opConfigLSB };

    // OpConfig register location: BQ27441_ID_REGISTERS id, offset 0
    return _bBQ27441_writeExtendedData(handle, BQ27441_ID_REGISTERS, 2, opConfigData, 1);
}

/*************************************************************************************************/
static bool _bBQ27441_writeOpConfig(bq27441_handle_t *handle, uint16_t value)
{
    uint8_t opConfigMSB = value >> 8;
    uint8_t opConfigLSB = value & 0x00FF;
    uint8_t opConfigData[2] = { opConfigMSB, opConfigLSB };

    // OpConfig register location: BQ27441_ID_REGISTERS id, offset 0
    return _bBQ27441_writeExtendedData(handle, BQ27441_ID_REGISTERS, 0, opConfigData, 2);
}

/*************************************************************************************************/
static bool _bBQ27441_softReset(bq27441_handle_t *handle)
{
    return _bBQ27441_executeControlWord(handle, BQ27441_CONTROL_SOFT_RESET);
}

/*************************************************************************************************/
static uint16_t _u16BQ27441_readWord(bq27441_handle_t *handle, uint16_t subAddress)
{
    uint8_t data[2];
    _stBQ27441_i2cReadBytes(handle, subAddress, data, 2);
    return ((uint16_t) data[1] << 8) | data[0];
}

/*************************************************************************************************/
static uint16_t _u16BQ27441_readControlWord(bq27441_handle_t *handle, uint16_t function)
{
    uint8_t subCommandMSB = (function >> 8);
    uint8_t subCommandLSB = (function & 0x00FF);
    uint8_t command[2] = { subCommandLSB, subCommandMSB };
    uint8_t data[2] = { 0, 0 };

    _stBQ27441_i2cWriteBytes(handle, (uint8_t) 0, command, 2);

    if (_stBQ27441_i2cReadBytes(handle, (uint8_t) 0, data, 2) == kStatus_Success)
    {
        return ((uint16_t) data[1] << 8) | data[0];
    }

    return false;
}

/*************************************************************************************************/
static bool _bBQ27441_executeControlWord(bq27441_handle_t *handle, uint16_t function)
{
    uint8_t subCommandMSB = (function >> 8);
    uint8_t subCommandLSB = (function & 0x00FF);
    uint8_t command[2] = { subCommandLSB, subCommandMSB };

    if (_stBQ27441_i2cWriteBytes(handle, (uint8_t) 0, command, 2) == kStatus_Success)
        return true;

    return false;
}

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/

/*************************************************************************************************/
static bool _bBQ27441_blockDataControl(bq27441_handle_t *handle)
{
    uint8_t enableByte = 0x00;
    if (_stBQ27441_i2cWriteBytes(handle, BQ27441_EXTENDED_CONTROL, &enableByte, 1) == kStatus_Success)
    {
        return true;
    }
    return false;
}

/*************************************************************************************************/
static bool _bBQ27441_blockDataClass(bq27441_handle_t *handle, uint8_t id)
{
    if (_stBQ27441_i2cWriteBytes(handle, BQ27441_EXTENDED_DATACLASS, &id, 1) == kStatus_Success)
    {
        return true;
    }
    return false;
}

/*************************************************************************************************/
static bool _BQ27441_blockDataOffset(bq27441_handle_t *handle, uint8_t offset)
{
    if (_stBQ27441_i2cWriteBytes(handle, BQ27441_EXTENDED_DATABLOCK, &offset, 1) == kStatus_Success)
    {
        return true;
    }
    return false;
}

/*************************************************************************************************/
static uint8_t _u8BQ27441_blockDataChecksum(bq27441_handle_t *handle)
{
    uint8_t csum;
    _stBQ27441_i2cReadBytes(handle, BQ27441_EXTENDED_CHECKSUM, &csum, 1);
    return csum;
}

/*************************************************************************************************/
static uint8_t _u8BQ27441_readBlockData(bq27441_handle_t *handle, uint8_t offset)
{
    uint8_t ret;
    uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
    _stBQ27441_i2cReadBytes(handle, address, &ret, 1);
    return ret;
}

/*************************************************************************************************/
static bool _bBQ27441_writeBlockData(bq27441_handle_t *handle, uint8_t offset, uint8_t data)
{
    uint8_t address = offset + BQ27441_EXTENDED_BLOCKDATA;
    if (_stBQ27441_i2cWriteBytes(handle, address, &data, 1) == kStatus_Success)
    {
        return true;
    }
    return false;
}

/*************************************************************************************************/
static uint8_t _u8BQ27441_computeBlockChecksum(bq27441_handle_t *handle)
{
    uint8_t data[32];
    _stBQ27441_i2cReadBytes(handle, BQ27441_EXTENDED_BLOCKDATA, data, 32);

    uint8_t csum = 0;
    for (int i = 0; i < 32; i++)
    {
        csum += data[i];
    }
    csum = 255 - csum;

    return csum;
}

/*************************************************************************************************/
static bool _bBQ27441_writeBlockChecksum(bq27441_handle_t *handle, uint8_t csum)
{
    if (_stBQ27441_i2cWriteBytes(handle, BQ27441_EXTENDED_CHECKSUM, &csum, 1) == kStatus_Success)
    {
        return true;
    }
    return false;
}

/*************************************************************************************************/
static uint8_t _u8BQ27441_readExtendedData(bq27441_handle_t *handle, uint8_t classID, uint8_t offset)
{
    uint8_t retData = 0;
    if (!_userConfigControl)
        bBQ27441_enterConfig(handle, false);

    if (!_bBQ27441_blockDataControl(handle))     // // enable block data memory control
        return false;     // Return false if enable fails
    if (!_bBQ27441_blockDataClass(handle, classID))     // Write class ID using DataBlockClass()
        return false;

    _BQ27441_blockDataOffset(handle, offset / 32);     // Write 32-bit block offset (usually 0)

    _u8BQ27441_computeBlockChecksum(handle);     // Compute checksum going in
//	u8BQ27441_blockDataChecksum(handle);
    retData = _u8BQ27441_readBlockData(handle, offset % 32);     // Read from offset (limit to 0-31)

    if (!_userConfigControl)
        bBQ27441_exitConfig(handle, true);

    return retData;
}

/*************************************************************************************************/
static bool _bBQ27441_writeExtendedData(
        bq27441_handle_t *handle,
        uint8_t classID,
        uint8_t offset,
        uint8_t * data,
        uint8_t len)
{
    if (len > 32)
        return false;

    if (!_userConfigControl)
        bBQ27441_enterConfig(handle, false);

    if (!_bBQ27441_blockDataControl(handle))     // // enable block data memory control
        return false;     // Return false if enable fails
    if (!_bBQ27441_blockDataClass(handle, classID))     // Write class ID using DataBlockClass()
        return false;

    _BQ27441_blockDataOffset(handle, offset / 32);     // Write 32-bit block offset (usually 0)
    _u8BQ27441_computeBlockChecksum(handle);     // Compute checksum going in
    uint8_t oldCsum = _u8BQ27441_blockDataChecksum(handle);
    oldCsum = oldCsum;
    // Write data bytes:
    for (int i = 0; i < len; i++)
    {
        // Write to offset, mod 32 if offset is greater than 32
        // The blockDataOffset above sets the 32-bit block
        _bBQ27441_writeBlockData(handle, (offset % 32) + i, data[i]);
    }

    // Write new checksum using BlockDataChecksum (0x60)
    uint8_t newCsum = _u8BQ27441_computeBlockChecksum(handle);     // Compute the new checksum
    _bBQ27441_writeBlockChecksum(handle, newCsum);

    if (!_userConfigControl)
    {
        bBQ27441_exitConfig(handle, true);
    }

    return true;
}

/*****************************************************************************
 ************************ I2C Read and Write Routines ************************
 *****************************************************************************/

/*************************************************************************************************/
static status_t _stBQ27441_i2cReadBytes(bq27441_handle_t *handle, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    status_t status = kStatus_Success;
    uint8_t i2c_buf[1];
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    i2c_buf[0] = subAddress;
    if (handle->i2c_driver.MasterTransmit(handle->DeviceAddr, i2c_buf, 1, false) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32BQ27441_WaitEvent(handle) != ARM_I2C_EVENT_TRANSFER_DONE)
    {
        status = kStatus_Fail;
    }
    else if (handle->i2c_driver.MasterReceive(handle->DeviceAddr, dest, count, false) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32BQ27441_WaitEvent(handle) != ARM_I2C_EVENT_TRANSFER_DONE)
    {
        status = kStatus_Fail;
    }
    return status;
}

/*************************************************************************************************/
static status_t _stBQ27441_i2cWriteBytes(bq27441_handle_t *handle, uint8_t subAddress, uint8_t * src, uint8_t count)
{

    status_t status = kStatus_Success;
    uint8_t i2c_buf[1];
    if (!handle)
    {
        return kStatus_InvalidArgument;
    }

    i2c_buf[0] = subAddress;
    if (handle->i2c_driver.MasterTransmit(handle->DeviceAddr, i2c_buf, 1, true) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32BQ27441_WaitEvent(handle) != ARM_I2C_EVENT_TRANSFER_DONE)
    {
        status = kStatus_Fail;
    }
    else if (handle->i2c_driver.MasterTransmit(handle->DeviceAddr, src, count, false) != ARM_DRIVER_OK)
    {
        status = kStatus_Fail;
    }
    else if (_u32BQ27441_WaitEvent(handle) != ARM_I2C_EVENT_TRANSFER_DONE)
    {
        status = kStatus_Fail;
    }
    return status;
}

/*************************************************************************************************/
void vBQ27441_parseFlags(flags_t *flagsParsed, uint16_t flags)
{
    //low byte
    flagsParsed->dsg = (flags & 0x01);
    flagsParsed->socf = (flags & 0x02) >> 1;
    flagsParsed->soc1 = (flags & 0x04) >> 2;
    flagsParsed->bat_det = (flags & 0x08) >> 3;
    flagsParsed->cfgupmode = (flags & 0x10) >> 4;
    flagsParsed->itpor = (flags & 0x20) >> 5;
    flagsParsed->ocvtaken = (flags & 0x80) >> 7;

    //high byte
    flagsParsed->chg = (flags & 0x100) >> 8;
    flagsParsed->fc = (flags & 0x200) >> 9;
    flagsParsed->ut = (flags & 0x4000) >> 14;
    flagsParsed->ot = (flags & 0x8000) >> 15;
}

/*************************************************************************************************/
static uint32_t _u32BQ27441_WaitEvent(bq27441_handle_t *handle)
{
    uint32_t i2c_event;

    xTaskManager_SemaphoreTake(handle->xSemaSignal, portMAX_DELAY);
    /*    while (!(handle->i2c_event_received))
     ;*/

    i2c_event = handle->i2c_event;
    handle->i2c_event_received = false;

    return i2c_event;
}

/*************************************************************************************************/
static uint8_t _constrain(uint8_t val, uint8_t min, uint8_t max)
{
    if (val >= min && val <= max)
    {
        return val;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return min;
    }
}
/*************************************************************************************************/
