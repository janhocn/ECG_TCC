/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2016  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.38 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:

The software has been licensed to  NXP Semiconductors USA, Inc.  whose
registered  office  is  situated  at 411 E. Plumeria Drive, San  Jose,
CA 95134, USA  solely for  the  purposes  of  creating  libraries  for
NXPs M0, M3/M4 and  ARM7/9 processor-based  devices,  sublicensed  and
distributed under the terms and conditions of the NXP End User License
Agreement.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information

Licensor:                 SEGGER Microcontroller Systems LLC
Licensed to:              NXP Semiconductors, 1109 McKay Dr, M/S 76, San Jose, CA 95131, USA
Licensed SEGGER software: emWin
License number:           GUI-00186
License model:            emWin License Agreement, dated August 20th 2011
Licensed product:         -
Licensed platform:        NXP's ARM 7/9, Cortex-M0,M3,M4
Licensed number of seats: -
----------------------------------------------------------------------
File        : LCDConf.h
Purpose     : Display driver configuration file
----------------------------------------------------------------------
*/

#ifndef LCDCONF_H
#define LCDCONF_H

#include "pin_mux.h"
//
// Physical display size
//
#define XSIZE_PHYS 480
#define YSIZE_PHYS 320


#define LCD_DSPI_DMA_MUX_BASEADDR DMAMUX
#define LCD_DSPI_DMA_BASEADDR DMA0

#if USE_PROTOTYPE
#define LCD_SPI_Driver Driver_SPI0
#define LCD_DSPI_IRQn SPI0_IRQn
#define LCD_DMA_TX_IRQn DMA0_IRQn
#define LCD_DMA_RX_IRQn DMA1_IRQn
#else
#define LCD_SPI_Driver Driver_SPI2
#define LCD_DSPI_IRQn SPI2_IRQn
#define LCD_DMA_TX_IRQn DMA6_IRQn
#define LCD_DMA_RX_IRQn DMA7_IRQn
#endif



/*************************************************************************************************/
/**
 * @der SPI Transfer baudrate
 */
#define LCD_TRANSFER_BAUDRATE 100000000U

#endif /* LCDCONF_H */

/*************************** End of file ****************************/
