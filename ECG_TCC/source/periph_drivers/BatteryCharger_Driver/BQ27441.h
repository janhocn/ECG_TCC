/**
 ******************************************************************************
 * @file    BQ27441.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    22 October 2017
 * @brief   Implementation of all features of the BQ27441 LiPo Fuel Gauge.
 * @attention !!!This code was ported from https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library!!!
 ******************************************************************************
 */
#ifndef PERIPH_DRIVERS_BATTERYCHARGER_DRIVER_BQ27441_H_
#define PERIPH_DRIVERS_BATTERYCHARGER_DRIVER_BQ27441_H_

#include "BQ27441_Defs.h"
#include "fsl_i2c_cmsis.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task_manager.h"

#define BQ27441_I2C_TIMEOUT 3000

#define BQ27441_I2C Driver_I2C1
#define BQ27441_I2C_IRQ I2C1_IRQn

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to configure and use this driver
 */
typedef struct _bq27441_handle
{
    ARM_DRIVER_I2C i2c_driver; /** @< I2C CMSIS Driver*/
    volatile uint32_t i2c_event; /** @< Hold the event coming from DMA*/
    volatile bool i2c_event_received; /** @< Signals event received*/
    IRQn_Type I2CIRQn; /** @< I2C IRQ*/
    uint32_t I2CIRQpriority; /** @< I2C IRQ priority*/
    uint8_t DeviceAddr; /** I2C device address */
    SemaphoreHandle_t xSemaSignal; /** @< Semaphore to control the singals coming from DMA*/
} bq27441_handle_t;

/*************************************************************************************************/
/**
 * @struct Structure containning the flags bits
 */
typedef struct {
	//high byte
	uint8_t ot;
	uint8_t ut;
	uint8_t fc;
	uint8_t chg;
	//low byte
	uint8_t ocvtaken;
	uint8_t itpor;
	uint8_t cfgupmode;
	uint8_t bat_det;
	uint8_t soc1;
	uint8_t socf;
	uint8_t dsg;
} flags_t;


/*************************************************************************************************/
/**
 * @enum Parameters for the current() function, to specify which current to read
 */
typedef enum
{
    AVG, /** @< Average Current (DEFAULT)*/
    STBY, /** @< Standby Current*/
    MAX /** @< Max Current*/
} current_measure;

/*************************************************************************************************/
/**
 * @enum Parameters for the capacity() function, to specify which capacity to read
 */
typedef enum
{
    REMAIN, /** @< Remaining Capacity (DEFAULT)*/
    FULL, /** @< Full Capacity*/
    AVAIL, /** @< Available Capacity*/
    AVAIL_FULL, /** @< Full Available Capacity*/
    REMAIN_F, /** @< Remaining Capacity Filtered*/
    REMAIN_UF, /** @< Remaining Capacity Unfiltered*/
    FULL_F, /** @< Full Capacity Filtered*/
    FULL_UF, /** @< Full Capacity Unfiltered*/
    DESIGN /** @< Design Capacity*/
} capacity_measure;

/*************************************************************************************************/
/**
 * @enum Parameters for the soc() function
 */
typedef enum
{
    FILTERED, /** @< State of Charge Filtered (DEFAULT)*/
    UNFILTERED /** @< State of Charge Unfiltered*/
} soc_measure;

/*************************************************************************************************/
/**
 * @enum Parameters for the soh() function
 */
typedef enum
{
    PERCENT, /** @< State of Health Percentage (DEFAULT)*/
    SOH_STAT /** @< State of Health Status Bits*/
} soh_measure;

/*************************************************************************************************/
/**
 * @enum Parameters for the temperature() function
 */
typedef enum
{
    BATTERY, /** @< Battery Temperature (DEFAULT)*/
    INTERNAL_TEMP /** @< Internal IC Temperature*/
} temp_measure;

/*************************************************************************************************/
/**
 * @enum Parameters for the setGPOUTFunction() funciton
 */
typedef enum
{
    SOC_INT, /** @< Set GPOUT to SOC_INT functionality*/
    BAT_LOW /** @< Set GPOUT to BAT_LOW functionality*/
} gpout_function;

/**
 @brief Initializes I2C and verifies communication with the BQ27441.
 @brief Must be called before using any other functions.

 @param BQ27441 driver handle to I2C related parameters
 @param cb_event callback to I2C signal event
 @return true if communication was successful.
 */
status_t stBQ27441_Init(bq27441_handle_t *handle, ARM_I2C_SignalEvent_t cb_event);

/**
 @brief Configures the design capacity of the connected battery.

 @param BQ27441 driver handle to I2C related parameters
 @param capacity of battery (unsigned 16-bit value)
 @return true if capacity successfully set.
 */
bool bBQ27441_setCapacity(bq27441_handle_t *handle, uint16_t capacity);

/////////////////////////////
// Battery Characteristics //
/////////////////////////////
/**
 @brief Reads and returns the battery voltage

 @param BQ27441 driver handle to I2C related parameters
 @return battery voltage in mV
 */
uint16_t u16BQ27441_getVoltage(bq27441_handle_t *handle);

/**
 @brief Reads and returns the specified current measurement

 @param BQ27441 driver handle to I2C related parameters
 @param current_measure enum specifying current value to be read
 @return specified current measurement in mA. >0 indicates charging.
 */
int16_t i16BQ27441_getCurrent(bq27441_handle_t *handle, current_measure type);

/**
 @brief Reads and returns the specified capacity measurement

 @param BQ27441 driver handle to I2C related parameters
 @param capacity_measure enum specifying capacity value to be read
 @return specified capacity measurement in mAh.
 */
uint16_t u16BQ27441_getCapacity(bq27441_handle_t *handle, capacity_measure type);

/**
 @brief Reads and returns measured average power

 @param BQ27441 driver handle to I2C related parameters
 @return average power in mAh. >0 indicates charging.
 */
int16_t i16BQ27441_getPower(bq27441_handle_t *handle);

/**
 @brief Reads and returns specified state of charge measurement

 @param BQ27441 driver handle to I2C related parameters
 @param soc_measure enum specifying filtered or unfiltered measurement
 @return specified state of charge measurement in %
 */
uint16_t u16BQ27441_getSOC(bq27441_handle_t *handle, soc_measure type);

/**
 @brief Reads and returns specified state of health measurement

 @param BQ27441 driver handle to I2C related parameters
 @param soh_measure enum specifying filtered or unfiltered measurement
 @return specified state of health measurement in %, or status bits
 */
uint8_t u8BQ27441_getSOH(bq27441_handle_t *handle, soh_measure type);

/**
 @brief Reads and returns specified temperature measurement

 @param BQ27441 driver handle to I2C related parameters
 @param temp_measure enum specifying internal or battery measurement
 @return specified temperature measurement in degrees C
 */
uint16_t u16BQ27441_getTemperature(bq27441_handle_t *handle, temp_measure type);

////////////////////////////
// GPOUT Control Commands //
////////////////////////////
/**
 @brief Get GPOUT polarity setting (active-high or active-low)

 @param BQ27441 driver handle to I2C related parameters
 @return true if active-high, false if active-low
 */
bool bBQ27441_getGPOUTPolarity(bq27441_handle_t *handle);

/**
 @brief Set GPOUT polarity to active-high or active-low

 @param BQ27441 driver handle to I2C related parameters
 @param activeHigh is true if active-high, false if active-low
 @return true on success
 */
bool bBQ27441_setGPOUTPolarity(bq27441_handle_t *handle, bool activeHigh);

/**
 @brief Get GPOUT function (BAT_LOW or SOC_INT)

 @param BQ27441 driver handle to I2C related parameters
 @return true if BAT_LOW or false if SOC_INT
 */
bool bBQ27441_getGPOUTFunction(bq27441_handle_t *handle);

/**
 @brief Set GPOUT function to BAT_LOW or SOC_INT

 @param BQ27441 driver handle to I2C related parameters
 @param function should be either BAT_LOW or SOC_INT
 @return true on success
 */
bool bBQ27441_setGPOUTFunction(bq27441_handle_t *handle, gpout_function function);

/**
 @brief Get SOC1_Set Threshold - threshold to set the alert flag

 @param BQ27441 driver handle to I2C related parameters
 @return state of charge value between 0 and 100%
 */
uint8_t u8BQ27441_getSOC1SetThreshold(bq27441_handle_t *handle);

/**
 @brief Get SOC1_Clear Threshold - threshold to clear the alert flag

 @param BQ27441 driver handle to I2C related parameters
 @return state of charge value between 0 and 100%
 */
uint8_t u8BQ27441_getSOC1ClearThreshold(bq27441_handle_t *handle);

/**
 @brief Set the SOC1 set and clear thresholds to a percentage

 @param BQ27441 driver handle to I2C related parameters
 @param set and clear percentages between 0 and 100. clear > set.
 @return true on success
 */
bool bBQ27441_setSOC1Thresholds(bq27441_handle_t *handle, uint8_t set, uint8_t clear);

/**
 @brief Get SOCF_Set Threshold - threshold to set the alert flag

 @param BQ27441 driver handle to I2C related parameters
 @return state of charge value between 0 and 100%
 */
uint8_t u8BQ27441_getSOCFSetThreshold(bq27441_handle_t *handle);

/**
 @brief Get SOCF_Clear Threshold - threshold to clear the alert flag

 @param BQ27441 driver handle to I2C related parameters
 @return state of charge value between 0 and 100%
 */
uint8_t u8BQ27441_getSOCFClearThreshold(bq27441_handle_t *handle);

/**
 @brief Set the SOCF set and clear thresholds to a percentage

 @param BQ27441 driver handle to I2C related parameters
 @param set and clear percentages between 0 and 100. clear > set.
 @return true on success
 */
bool bBQ27441_setSOCFThresholds(bq27441_handle_t *handle, uint8_t set, uint8_t clear);

/**
 Check if the SOC1 flag is set in flags()

 @param BQ27441 driver handle to I2C related parameters
 @return true if flag is set
 */
bool bBQ27441_getSOCFlag(bq27441_handle_t *handle);

/**
 Check if the SOCF flag is set in flags()

 @param BQ27441 driver handle to I2C related parameters
 @return true if flag is set
 */
bool bBQ27441_getSOCFFlag(bq27441_handle_t *handle);

/**
 @brief Get the SOC_INT interval delta

 @param BQ27441 driver handle to I2C related parameters
 @return interval percentage value between 1 and 100
 */
uint8_t u8BQ27441_getSOCIDelta(bq27441_handle_t *handle);

/**
 @brief Set the SOC_INT interval delta to a value between 1 and 100

 @param BQ27441 driver handle to I2C related parameters
 @param interval percentage value between 1 and 100
 @return true on success
 */
bool bBQ27441_setSOCIDelta(bq27441_handle_t *handle, uint8_t delta);

/**
 @brief Pulse the GPOUT pin - must be in SOC_INT mode

 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
bool bBQ27441_pulseGPOUT(bq27441_handle_t *handle);

//////////////////////////
// Control Sub-commands //
//////////////////////////

/**
 @brief Read the device type - should be 0x0421

 @param BQ27441 driver handle to I2C related parameters
 @return 16-bit value read from DEVICE_TYPE subcommand
 */
uint16_t u16BQ27441_getDeviceType(bq27441_handle_t *handle);

/**
 @brief Enter configuration mode - set userControl if calling from an Arduino
 @brief sketch and you want control over when to exitConfig.

 @param BQ27441 driver handle to I2C related parameters
 @param userControl is true if the Arduino sketch is handling entering
 and exiting config mode (should be false in library calls).
 @return true on success
 */
bool bBQ27441_enterConfig(bq27441_handle_t *handle, bool userControl);

/**
 @brief Exit configuration mode with the option to perform a resimulation

 @param BQ27441 driver handle to I2C related parameters
 @param resim is true if resimulation should be performed after exiting
 @return true on success
 */
bool bBQ27441_exitConfig(bq27441_handle_t *handle, bool resim);

/**
 @brief Read the flags() command

 @param BQ27441 driver handle to I2C related parameters
 @return 16-bit representation of flags() command register
 */
uint16_t u16BQ27441_getFlags(bq27441_handle_t *handle);

/**
 @brief Read the CONTROL_STATUS subcommand of control()

 @param BQ27441 driver handle to I2C related parameters
 @return 16-bit representation of CONTROL_STATUS subcommand
 */
uint16_t u16BQ27441_getStatus(bq27441_handle_t *handles);

/**
 @brief Make sure that the device is awake and has taken a reading.
 @param BQ27441 driver handle to I2C related parameters
 @return true on success
 */
bool bBQ27441_makeADCReading(bq27441_handle_t *handle);

/**
 @brief Write the 16-bit opConfigB register in extended data

 @param BQ27441 driver handle to I2C related parameters
 @param New 16-bit value for opConfig
 @return true on success
 */
bool bBQ27441_writeOpConfigB(bq27441_handle_t *handle, uint16_t value);

/**
 @brief Parsing of Flag Register result.
 @param flagsParsed flags structure parsed
 @param flags flags register value
 @return void.
 */
void vBQ27441_parseFlags(flags_t *flagsParsed, uint16_t flags);
/**
 @brief Handles the I2C event

 @param BQ27441 driver handle to I2C related parameters
 @param i2c_event I2C event
 @return void
 */
void vBQ27441_EventHandler(bq27441_handle_t *handle, uint32_t i2c_event);

#endif /* PERIPH_DRIVERS_BATTERYCHARGER_DRIVER_BQ27441_H_ */
