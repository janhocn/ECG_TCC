/**
 ******************************************************************************
 * @file    GrapCommon.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    17 September 2017
 * @brief   Module to with common configuration for Graphs
 ******************************************************************************
 */
#ifndef USER_WINDOWS_GRAPHCOMMON_H_
#define USER_WINDOWS_GRAPHCOMMON_H_

#include "WM.h"
#include "GRAPH_Private.h"

/**
 * \defgroup Graph Dimensions and configuration
 * @{
 */
#define BKGraphGrayCustom 0x004B4B4B
#define BKGraphGreenCustom 0x00031405

#define VGridPX 15
#define HGridPX 20
#define VNumberGrid 15
#define HNumberGrid 16
#define VTotalGridPX VGridPX*VNumberGrid
#define HTotalGridPX HGridPX*HNumberGrid

#define GRAPH_YMAX HGridPX*HNumberGrid

#define BORDERLEFTPX 3
#define BORDERRIGHTPX 3
#define BORDERTOPPX 5
#define BORDERBOTTOMPX 24

#define XStartGraph 8
#define YStartGraph 20

#define XMAXDisplay (350)
#define WinXSize (XMAXDisplay + BORDERRIGHTPX + BORDERLEFTPX)

#define YMAXDisplay ((VGridPX*((2*7)+3)) + 1)
#define WinYSize (YMAXDisplay + BORDERTOPPX + BORDERBOTTOMPX)

#define WindowWidth 480
#define WindowHeight 300
/**@}*/

#endif /* USER_WINDOWS_GRAPHCOMMON_H_ */
