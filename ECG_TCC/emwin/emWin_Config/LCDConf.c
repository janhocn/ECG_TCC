#include "GUI.h"

#include "GUIDRV_FLEXCOLOR.h"
#include "GUIDRV_DCACHE.h"
#include "LCDConf.h"
#include "LCD.h"

#include "HX8357.h"
#include "pin_mux.h"
#include "prioAssigner.h"

/*********************************************************************
 *
 *       Layer configuration (to be modified)
 *
 **********************************************************************
 */
//
// Color conversion
//
#define COLOR_CONVERSION GUICC_M565

//
// Display driver
//
#define DISPLAY_DRIVER GUIDRV_FLEXCOLOR

//
// Orientation
//
#if USE_PROTOTYPE == 1
#define DISPLAY_ORIENTATION (GUI_SWAP_XY | GUI_MIRROR_Y)
#else
#define DISPLAY_ORIENTATION (GUI_SWAP_XY | GUI_MIRROR_X)
#endif
/*********************************************************************
 *
 *       Configuration checking
 *
 **********************************************************************
 */
/*********************************************************************
 *
 *       Configuration checking
 *
 **********************************************************************
 */
#ifndef   VXSIZE_PHYS
#define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
#define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
#error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
#error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
#error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
#error No display driver defined!
#endif
#ifndef   DISPLAY_ORIENTATION
#define DISPLAY_ORIENTATION 0
#endif

/*********************************************************************
 *
 *       Static code
 *
 **********************************************************************
 */
/*********************************************************************
 *
 *       _InitController
 *
 * Purpose:
 *   Initializes the display controller
 */
static void _InitController(void)
{
    hx8357_handle_t config;

    memcpy((void * restrict) &config.dspi_driver, (const void * restrict) &LCD_SPI_Driver, sizeof(ARM_DRIVER_SPI));
    config.DMABase = LCD_DSPI_DMA_BASEADDR;
    config.DMAMuxBase = LCD_DSPI_DMA_MUX_BASEADDR;
    config.DMATXIRQn = LCD_DMA_TX_IRQn;
    config.DMARXIRQn = LCD_DMA_RX_IRQn;
    config.DMAIRQpriority = HX8357_SPI_INT_PRIO;
    config.DCGpio = GPIO_TFT_DC_GPIO;
    config.DCPin = GPIO_TFT_DC_PIN;
    config.CSGpio = GPIO_TFT_CS_GPIO;
    config.CSPin = GPIO_TFT_CS_PIN;
    config.RSTGpio = GPIO_TFT_RST_GPIO;
    config.RSTPin = GPIO_TFT_RST_PIN;
    config.BKLGpio = GPIO_TFT_BKL_GPIO;
    config.BKLPin = GPIO_TFT_BKL_PIN;
    config.baudrate = LCD_TRANSFER_BAUDRATE;
    TFT_InitPins();
    sHX8357_Init(&config);
}

/*********************************************************************
 *
 *       Public code
 *
 **********************************************************************
 */
/*********************************************************************
 *
 *       LCD_X_Config
 */
void LCD_X_Config(void)
{
    /*
        GUI_DEVICE_CreateAndLink()
        GUIDRV_FlexColor_Config()
        LCD_SetSizeEx()
        LCD_SetVSizeEx()
        GUIDRV_FlexColor_SetInterface()
        GUIDRV_FlexColor_SetReadFunc()
        GUIDRV_FlexColor_SetFunc()
    */
    GUI_DEVICE * pDevice;
//    GUI_DEVICE * pCache;
    GUI_PORT_API PortAPI =
            { 0 };
    CONFIG_FLEXCOLOR Config =
            { 0 };
    //
    // Set display driver and color conversion for 1st layer
    //
    pDevice = GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER, COLOR_CONVERSION, 0, 0);
    //
    // Display size configuration
    //
    if (DISPLAY_ORIENTATION & GUI_SWAP_XY)
    {
        LCD_SetSizeEx(0, YSIZE_PHYS, XSIZE_PHYS);
        LCD_SetVSizeEx(0, VYSIZE_PHYS, VXSIZE_PHYS);
    }
    else
    {
        LCD_SetSizeEx(0, XSIZE_PHYS, YSIZE_PHYS);
        LCD_SetVSizeEx(0, VXSIZE_PHYS, VYSIZE_PHYS);
    }
    //
    // Orientation
    //
    Config.Orientation = DISPLAY_ORIENTATION;
    Config.NumDummyReads = 1;
    GUIDRV_FlexColor_Config(pDevice, &Config);
    //
    // Driver configuration
    //
    PortAPI.pfWrite8_A0 = vHX8357_pfWrite8_A0;
    PortAPI.pfWrite8_A1 = vHX8357_pfWrite8_A1;
    PortAPI.pfWriteM8_A0 = vHX8357_pfWriteM8_A0;
    PortAPI.pfWriteM8_A1 = vHX8357_pfWriteM8_A1;
    PortAPI.pfRead8_A1 = vHX8357_pfRead8_A1;
    PortAPI.pfReadM8_A1 = vHX8357_pfReadM8_A1;
    PortAPI.pfRead8_A0 = vHX8357_pfRead8_A0;
    PortAPI.pfReadM8_A0 = vHX8357_pfReadM8_A0;
    PortAPI.pfSetCS = vHX8357_SetCS;
    GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);

//    //
//    // Create and configure (double) cache driver, ...
//    //
//    pCache = GUI_DEVICE_CreateAndLink(GUIDRV_DCACHE, GUICC_1, 0, 0);
//
//    //
//    // ... set size, ...
//    //
//    GUIDRV_DCache_SetMode1bpp(pCache);
//
//    //
//    // ... and add real driver.
//    //
//    GUIDRV_DCache_AddDriver(pCache, pDevice);
}

/*********************************************************************
 *
 *       LCD_X_DisplayDriver
 */
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData)
{
    int r;

    switch (Cmd)
    {
        //
        // Required
        //
        case LCD_X_INITCONTROLLER:
            //
            // Called during the initialization process in order to set up the
            // display controller and put it into operation. If the display
            // controller is not initialized by any external routine this needs
            // to be adapted by the customer...
            //
            _InitController();
            return 0;

        case LCD_X_ON:
            vHX8357_StandBy(false);
            return 0;

        case LCD_X_OFF:
            vHX8357_StandBy(true);
            return 0;

        default:
            r = -1;
    }
    return r;
}

/*************************** End of file ****************************/
