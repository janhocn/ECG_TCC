/**
 ******************************************************************************
 * @file    powerManagerTask.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    14 April 2018
 * @brief   Module to manage power supplies and battery status
 ******************************************************************************
 */

#include "BatteryCharger.h"
#include "powerManagerTask.h"
#include "task_manager.h"
#include "GUI.h"
#include "LCD.h"
#include "LCDConf.h"
#include "task_manager.h"
#include "prioAssigner.h"
#include "StatusBar.h"
#include "pin_mux.h"
#include "MK64F12.h"
#include "fsl_common.h"
#include "fsl_gpio.h"

static void vPowerManager_Task(void *pvParameters);
void vPowerManager_DisconnectChargerFromBattery(void);
void vPowerManager_ConnectChargerToBattery(void);

bool oldStateBatteryDetection = false;
bool oldStateChargingDetection = false;

/*************************************************************************************************/
static void vPowerManager_Task(void *pvParameters)
{
    const portTickType xUpdateFrequency = (refreshPowerManagerPeriod / portTICK_PERIOD_MS);
    BatteryChargerInfo_t batInfo;
    flags_t flags;
    //Initial States
    bBatteryCharger_UpdateFlags(&flags);
    vStatusBar_setBatteryDetected(flags.bat_det);
    vStatusBar_setCharging(!flags.dsg);

    for (;;)
    {
        if (bTaskManager_isSystemInitOK())
        {
            vBatteryCharger_UpdateInfo(&batInfo);
            vStatusBar_setBatteryPercentage(batInfo.stateOfCharge);
            bBatteryCharger_UpdateFlags(&flags);
            vStatusBar_setBatteryDetected(flags.bat_det);
            vStatusBar_setCharging(!flags.dsg);
        }
        vTaskManager_Delay(xUpdateFrequency);
    }
    vTaskManager_TaskDelete( NULL);

}

/*************************************************************************************************/
void vPowerManager_DisconnectChargerFromBattery(void)
{
    GPIO_SetPinsOutput(GPIO_CHG_SYSOFF_GPIO, 1U << GPIO_CHG_SYSOFF_PIN);
}

/*************************************************************************************************/
void vPowerManager_ConnectChargerToBattery(void)
{
    GPIO_ClearPinsOutput(GPIO_CHG_SYSOFF_GPIO, 1U << GPIO_CHG_SYSOFF_PIN);
}

/*************************************************************************************************/
void vPowerManagerTaskInit(void)
{
    status_t stat = kStatus_Fail;
    BQ4075_InitPins();

    stat = stBatteryCharger_Init();
    if (stat != kStatus_Success)
    {
        stat = stBatteryCharger_Init();
    }
    if (stat == kStatus_Success)
    {
        xTaskManager_TaskCreate(vPowerManager_Task, "PowerManager_Task",
        configMINIMAL_STACK_SIZE + 200,
        NULL, powerManagerTask_PRIORITY, NULL);
    }
}

/*************************************************************************************************/
