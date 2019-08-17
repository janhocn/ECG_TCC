/**
 ******************************************************************************
 * @file    BatteryCharger.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    18 April 2018
 * @brief   File System Module
 ******************************************************************************
 */

#include "BatteryCharger.h"
#include "fsl_gpio.h"
#include "fsl_device_registers.h"
#include "fsl_i2c.h"
#include "fsl_common.h"
#include "pin_mux.h"
#include "user.h"
#include "prioAssigner.h"

/*************************************************************************************************/
/**
 * @var BQ27441 driver handler
 */
static bq27441_handle_t chargerHandle;

/*************************************************************************************************/
/**
 * @var BQ27441 flags
 */
static uint16_t flags;
/*************************************************************************************************/
/**
 * @var signals driver corrected initialized
 */
static bool isDriverInitialized = false;

/*************************************************************************************************/
/**
 * @def CMSIS I2C Driver
 */
#define BQ27441_I2C_Driver Driver_I2C1

/*************************************************************************************************/
/**
 * @def I2C1 IRQn
 */
#define BQ27441_I2C_IRQn I2C1_IRQn

/*************************************************************************************************/
/**
 * @brief Handles the i2c event signal
 * @param event Signal coming from spi CMSIS IRQ handler
 * @return void
 */
static void _vBatteryCharger_I2CMasterSignalEvent(uint32_t event);

/*************************************************************************************************/
static void _vBatteryCharger_I2CMasterSignalEvent(uint32_t event)
{
    vBQ27441_EventHandler(&chargerHandle, event);
}

/*************************************************************************************************/
void vBatteryCharger_GPOUT_IRQHandler(uint32_t status)
{
    if (status == (1 << GPIO_GAUGE_GPOUT_PIN))
    {
        GPIO_ClearPinsInterruptFlags(GPIO_GAUGE_GPOUT_GPIO, 1 << GPIO_GAUGE_GPOUT_PIN);
    }
}

/*************************************************************************************************/
status_t stBatteryCharger_Init(void)
{
    status_t ret = kStatus_Success;

    memcpy((void * restrict) &chargerHandle.i2c_driver, (const void * restrict) &Driver_I2C1, sizeof(ARM_DRIVER_I2C));

    chargerHandle.DeviceAddr = BQ72441_I2C_ADDRESS ;
    chargerHandle.I2CIRQn = BQ27441_I2C_IRQn;
    chargerHandle.I2CIRQpriority = BATCHARGER_I2C_PRIO;
    /****************Enable interrupt on PORTA***************************/
    EnableIRQ(GPIO_GAUGE_GPOUT_IRQn);
    /********************************************************************/

    ret = stBQ27441_Init(&chargerHandle, _vBatteryCharger_I2CMasterSignalEvent);

    if (ret == kStatus_Success)
    {
    	vTaskManager_Delay(1);
    	if(bBQ27441_enterConfig(&chargerHandle, true))
    	{
    		vTaskManager_Delay(10);
			if (bBQ27441_setCapacity(&chargerHandle, 1000))
			{
//				vTaskManager_Delay(10);
//			    bBQ27441_writeOpConfigB(&chargerHandle,0x0B);
				vTaskManager_Delay(10);
				if(bBQ27441_exitConfig(&chargerHandle, true) == false)
				{
					ret = kStatus_Fail;
				}
			}
			else
			{
				ret = kStatus_Fail;
			}
    	}
    	else
    	{
    		ret = kStatus_Fail;
    	}
    }

    if (ret == kStatus_Success)
    {
        isDriverInitialized = true;
    }
    vTaskManager_Delay(1);
    return ret;
}

/*************************************************************************************************/
void vBatteryCharger_UpdateInfo(BatteryChargerInfo_t* info)
{

    if (!isDriverInitialized)
    {
        return;
    }

    if (bBQ27441_makeADCReading(&chargerHandle))
    {
        info->stateOfCharge = u16BQ27441_getSOC(&chargerHandle, UNFILTERED);
        vTaskManager_Delay(10);
        info->voltage = u16BQ27441_getVoltage(&chargerHandle);
        vTaskManager_Delay(10);
        info->current = i16BQ27441_getCurrent(&chargerHandle, AVG);
        vTaskManager_Delay(10);
        info->fullCapacity = u16BQ27441_getCapacity(&chargerHandle, FULL_F);
        vTaskManager_Delay(10);
        info->designCapacity = u16BQ27441_getCapacity(&chargerHandle, DESIGN);
        vTaskManager_Delay(10);
        info->capacity = u16BQ27441_getCapacity(&chargerHandle, REMAIN);
        vTaskManager_Delay(10);
        info->power = i16BQ27441_getPower(&chargerHandle);
        vTaskManager_Delay(10);
        info->health = u8BQ27441_getSOH(&chargerHandle, PERCENT);
        vTaskManager_Delay(10);
        info->temperature = ((int16_t)u16BQ27441_getTemperature(&chargerHandle, BATTERY)/10) - 273;
        vTaskManager_Delay(10);
    }
}

/*************************************************************************************************/
void bBatteryCharger_UpdateFlags(flags_t* sflags)
{
    if (!isDriverInitialized)
    {
        return;
    }

    if (bBQ27441_makeADCReading(&chargerHandle))
    {
        vTaskManager_Delay(10);
    	flags = u16BQ27441_getFlags(&chargerHandle);
    	vTaskManager_Delay(10);
        vBQ27441_parseFlags(sflags,flags);
    }
}

/*************************************************************************************************/
uint16_t u16BatteryCharger_getFlags(void)
{
    if (!isDriverInitialized)
    {
        return 0;
    }

    return flags;
}

/*************************************************************************************************/
