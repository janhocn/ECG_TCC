/**
 ******************************************************************************
 * @file    HX8357.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    6 November 2017
 * @brief   Display HX8357 Driver
 ******************************************************************************
 */

#include "HX8357.h"

#include "fsl_gpio.h"
#include "fsl_device_registers.h"
#include "fsl_dspi.h"
#include "fsl_common.h"
#include "user.h"
#include <string.h>
#include "SystickUser.h"
#include "InterruptManager.h"
#include "prioAssigner.h"
#include "pin_mux.h"
/*************************************************************************************************/
/**
 * @var Pack with all members of this module
 */
static hx8357_handle_t _ThisHandle;
/*************************************************************************************************/
/**
 @brief Wait Master Complete Event on DSPI
 @return DSPI event
 */
static uint32_t _u32HX8357_WaitEvent(void);

/*************************************************************************************************/
/**
 @brief Signals the event from DSPI
 @param event Event come from DSPI master
 @return DSPI event
 */
static void _vHX8357_DSPIMasterSignalEvent(uint32_t event);

/*************************************************************************************************/
status_t sHX8357_Init(hx8357_handle_t* ConfigHandle)
{
    if (ConfigHandle == NULL)
    {
        return kStatus_InvalidArgument;
    }

    memcpy(&_ThisHandle, ConfigHandle, sizeof(hx8357_handle_t));

    _ThisHandle.xSemaTxDone = xTaskManager_SemaphoreCreate();
    configASSERT(_ThisHandle.xSemaTxDone != NULL);

    _ThisHandle.dspi_driver.Initialize(_vHX8357_DSPIMasterSignalEvent);
    _ThisHandle.dspi_driver.PowerControl(ARM_POWER_FULL);
    _ThisHandle.dspi_driver.Control(
    ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA1 | ARM_SPI_SS_MASTER_SW| ARM_SPI_DATA_BITS(8), _ThisHandle.baudrate);

    NVIC_SetPriority(_ThisHandle.DMARXIRQn, _ThisHandle.DMAIRQpriority);
    NVIC_SetPriority(_ThisHandle.DMATXIRQn, _ThisHandle.DMAIRQpriority);

    _ThisHandle.dspi_event_received = false;

    vHX8357_TurnBacklightON();
    vHX8357_ResetDisplay();
    vHX8357_ResetDisplay();

    /* Initialization Sequency */

    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);

    vHX8357_pfWrite8_A0(HX8357_SWRESET);
    vSystickUser_Delay(120);


    // setextc
    vHX8357_pfWrite8_A0(HX8357D_SETC);
    vHX8357_pfWrite8_A1(0xFF);
    vHX8357_pfWrite8_A1(0x83);
    vHX8357_pfWrite8_A1(0x57);

    vSystickUser_Delay(300);

    // setRGB which also enables SDO
    vHX8357_pfWrite8_A0(HX8357_SETRGB);
    vHX8357_pfWrite8_A1(0x80);     //enable SDO pin!
    vHX8357_pfWrite8_A1(0x0);
    vHX8357_pfWrite8_A1(0x01);
    vHX8357_pfWrite8_A1(0x01);

    vHX8357_pfWrite8_A0(HX8357D_SETCOM);
    vHX8357_pfWrite8_A1(0x25);     // -1.52V

    vHX8357_pfWrite8_A0(HX8357_SETOSC);
    vHX8357_pfWrite8_A1(0x6F);     // Normal mode 70Hz, Idle mode 55 Hz

    vHX8357_pfWrite8_A0(HX8357_SETPANEL);     //Set Panel
    vHX8357_pfWrite8_A1(0x05);     // BGR, Gate direction swapped

    vHX8357_pfWrite8_A0(HX8357_SETPWR1);
    vHX8357_pfWrite8_A1(0x00);     // Not deep standby
    vHX8357_pfWrite8_A1(0x15);     //BT
    vHX8357_pfWrite8_A1(0x1C);     //VSPR
    vHX8357_pfWrite8_A1(0x1C);     //VSNR
    vHX8357_pfWrite8_A1(0x83);     //AP
    vHX8357_pfWrite8_A1(0xAA);     //FS

    vHX8357_pfWrite8_A0(HX8357D_SETSTBA);
    vHX8357_pfWrite8_A1(0x50);     //OPON normal
    vHX8357_pfWrite8_A1(0x50);     //OPON idle
    vHX8357_pfWrite8_A1(0x01);     //STBA
    vHX8357_pfWrite8_A1(0x3C);     //STBA
    vHX8357_pfWrite8_A1(0x1E);     //STBA
    vHX8357_pfWrite8_A1(0x08);     //GEN

    vHX8357_pfWrite8_A0(HX8357D_SETCYC);
    vHX8357_pfWrite8_A1(0x02);     //NW 0x02
    vHX8357_pfWrite8_A1(0x40);     //RTN
    vHX8357_pfWrite8_A1(0x00);     //DIV
    vHX8357_pfWrite8_A1(0x2A);     //DUM
    vHX8357_pfWrite8_A1(0x2A);     //DUM
    vHX8357_pfWrite8_A1(0x0D);     //GDON
    vHX8357_pfWrite8_A1(0x78);     //GDOFF

    vHX8357_pfWrite8_A0(HX8357D_SETGAMMA);
    vHX8357_pfWrite8_A1(0x02);
    vHX8357_pfWrite8_A1(0x0A);
    vHX8357_pfWrite8_A1(0x11);
    vHX8357_pfWrite8_A1(0x1d);
    vHX8357_pfWrite8_A1(0x23);
    vHX8357_pfWrite8_A1(0x35);
    vHX8357_pfWrite8_A1(0x41);
    vHX8357_pfWrite8_A1(0x4b);
    vHX8357_pfWrite8_A1(0x4b);
    vHX8357_pfWrite8_A1(0x42);
    vHX8357_pfWrite8_A1(0x3A);
    vHX8357_pfWrite8_A1(0x27);
    vHX8357_pfWrite8_A1(0x1B);
    vHX8357_pfWrite8_A1(0x08);
    vHX8357_pfWrite8_A1(0x09);
    vHX8357_pfWrite8_A1(0x03);
    vHX8357_pfWrite8_A1(0x02);
    vHX8357_pfWrite8_A1(0x0A);
    vHX8357_pfWrite8_A1(0x11);
    vHX8357_pfWrite8_A1(0x1d);
    vHX8357_pfWrite8_A1(0x23);
    vHX8357_pfWrite8_A1(0x35);
    vHX8357_pfWrite8_A1(0x41);
    vHX8357_pfWrite8_A1(0x4b);
    vHX8357_pfWrite8_A1(0x4b);
    vHX8357_pfWrite8_A1(0x42);
    vHX8357_pfWrite8_A1(0x3A);
    vHX8357_pfWrite8_A1(0x27);
    vHX8357_pfWrite8_A1(0x1B);
    vHX8357_pfWrite8_A1(0x08);
    vHX8357_pfWrite8_A1(0x09);
    vHX8357_pfWrite8_A1(0x03);
    vHX8357_pfWrite8_A1(0x00);
    vHX8357_pfWrite8_A1(0x01);

    vHX8357_pfWrite8_A0(HX8357_CASET);
    vHX8357_pfWrite8_A1(0x00);
    vHX8357_pfWrite8_A1(0x00);
    vHX8357_pfWrite8_A1(0x01);
    vHX8357_pfWrite8_A1(0x3F);

    vHX8357_pfWrite8_A0(HX8357_PASET);
    vHX8357_pfWrite8_A1(0x00);
    vHX8357_pfWrite8_A1(0x00);
    vHX8357_pfWrite8_A1(0x01);
    vHX8357_pfWrite8_A1(0xDF);

    vHX8357_pfWrite8_A0(HX8357_COLMOD);
    vHX8357_pfWrite8_A1(HX8357_SET_PIXEL_FORMAT_DPI_16BIT | HX8357_SET_PIXEL_FORMAT_DBI_16BIT);

    vHX8357_pfWrite8_A0(HX8357_MADCTL);
#if USE_PROTOTYPE
    vHX8357_pfWrite8_A1(0xC0);
#else
    vHX8357_pfWrite8_A1(0x00);
#endif
    vHX8357_pfWrite8_A0(HX8357_TEON);     // TE off
    vHX8357_pfWrite8_A1(0x00);

    vHX8357_pfWrite8_A0(HX8357_TEARLINE);     // tear line
    vHX8357_pfWrite8_A1(0x00);
    vHX8357_pfWrite8_A1(0x02);

    vHX8357_pfWrite8_A0(HX8357_SLPOUT);     //Exit Sleep
    vSystickUser_Delay(150);

    vHX8357_pfWrite8_A0(HX8357_DISPON);     // display on
    vSystickUser_Delay(50);


    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);

    return kStatus_Success;
}

/*************************************************************************************************/
void vHX8357_SetCS(uint8_t NotActive)
{
    if (NotActive)
    {
        GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    }
    else
    {
        GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    }
}

/*************************************************************************************************/
void vHX8357_Deinit(void)
{
    _ThisHandle.dspi_driver.PowerControl(ARM_POWER_OFF);
    _ThisHandle.dspi_driver.Uninitialize();
}

/*************************************************************************************************/
void vHX8357_TurnBacklightON(void)
{
    GPIO_SetPinsOutput(_ThisHandle.BKLGpio, 1U << _ThisHandle.BKLPin);
}

/*************************************************************************************************/
void vHX8357_TurnBacklightOFF(void)
{
    GPIO_ClearPinsOutput(_ThisHandle.BKLGpio, 1U << _ThisHandle.BKLPin);
}

/*************************************************************************************************/
void vHX8357_StandBy(bool enter)
{
    if (enter)
    {
        //Exit stand by mode
        vHX8357_pfWrite8_A0(HX8357_DISPOFF);
    }
    else
    {
        //Exit stand by mode
        vHX8357_pfWrite8_A0(HX8357_DISPON);
    }
    vSystickUser_Delay(50);
}

/*************************************************************************************************/
void vHX8357_ResetDisplay(void)
{
    GPIO_SetPinsOutput(_ThisHandle.RSTGpio, 1U << _ThisHandle.RSTPin);
    vSystickUser_Delay(50);
    GPIO_ClearPinsOutput(_ThisHandle.RSTGpio, 1U << _ThisHandle.RSTPin);
    vSystickUser_Delay(50);
    GPIO_SetPinsOutput(_ThisHandle.RSTGpio, 1U << _ThisHandle.RSTPin);
    vSystickUser_Delay(100);
}

/*************************************************************************************************/
void vHX8357_pfWrite8_A0(uint8_t Data)
{

    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    GPIO_ClearPinsOutput(_ThisHandle.DCGpio, 1u << _ThisHandle.DCPin);
    _ThisHandle.dspi_driver.Send(&Data, 1);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.DCGpio, 1u << _ThisHandle.DCPin);
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
}

/*************************************************************************************************/
void vHX8357_pfWrite8_A1(uint8_t Data)
{

    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    _ThisHandle.dspi_driver.Send(&Data, 1);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
}

/*************************************************************************************************/
void vHX8357_pfWriteM8_A0(uint8_t *pData, int NumItems)
{
    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);

    GPIO_ClearPinsOutput(_ThisHandle.DCGpio, 1u << _ThisHandle.DCPin);
    _ThisHandle.dspi_driver.Send(pData, NumItems);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.DCGpio, 1u << _ThisHandle.DCPin);
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
}

/*************************************************************************************************/
void vHX8357_pfWriteM8_A1(uint8_t *pData, int NumItems)
{
    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    _ThisHandle.dspi_driver.Send(pData, NumItems);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
}

/*************************************************************************************************/
uint8_t vHX8357_pfRead8_A0(void)
{
    uint8_t Data = 0;
    uint8_t du = 0x00;
    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    _ThisHandle.dspi_driver.Transfer(&du,&Data, 1);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    return Data;
}

/*************************************************************************************************/
void vHX8357_pfReadM8_A0(uint8_t *pData, int NumItems)
{
    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    _ThisHandle.dspi_driver.Receive(pData, NumItems);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
}

/*************************************************************************************************/
uint8_t vHX8357_pfRead8_A1(void)
{
    uint8_t Data;
    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    _ThisHandle.dspi_driver.Receive(&Data, 1);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    return Data;
}

/*************************************************************************************************/
void vHX8357_pfReadM8_A1(uint8_t *pData, int NumItems)
{
    GPIO_ClearPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);
    _ThisHandle.dspi_driver.Receive(pData, NumItems);
    _u32HX8357_WaitEvent();
    GPIO_SetPinsOutput(_ThisHandle.CSGpio, 1U << _ThisHandle.CSPin);

}

/*************************************************************************************************/
static void _vHX8357_DSPIMasterSignalEvent(uint32_t event)
{
    _ThisHandle.dspi_event = event;
    _ThisHandle.dspi_event_received = true;
    xTaskManager_SemaphoreGive(_ThisHandle.xSemaTxDone);
}

/*************************************************************************************************/
static uint32_t _u32HX8357_WaitEvent(void)
{

    xTaskManager_SemaphoreTake(_ThisHandle.xSemaTxDone, portMAX_DELAY);

    _ThisHandle.dspi_event_received = false;

    return _ThisHandle.dspi_event;
}
/*************************************************************************************************/
