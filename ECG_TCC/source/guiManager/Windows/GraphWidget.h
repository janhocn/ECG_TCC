/**
 ******************************************************************************
 * @file    GraphWidget.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    19 February 2018
 * @brief   Module to use the graph emwin widget with sweep bar
 ******************************************************************************
 */

#ifndef SOURCE_GRAPHWIDGET_H_
#define SOURCE_GRAPHWIDGET_H_

/* SEGGER emWin includes. */
#include "GUI.h"
#include "GRAPH.h"
#include "WM.h"
#include "LCDConf.h"
#include "arm_math.h"
#include <stdbool.h>

/*************************************************************************************************/
/**
 * @typedef Emwin UserDraw Callback.
 */
typedef void GRAPH_USERDRAW_CALLBACK( WM_HWIN, int);

/*************************************************************************************************/
/**
 * @struct Structure containing a data buffer to be added to the graph and the curve color
 */
typedef struct _sGraphData_t
{
        int16_t* YData; /** @< Pointer to hold the data buffer */
        GUI_COLOR lineColor; /** @< Color of the curve */
} sGraphData_t;

/*************************************************************************************************/
/**
 * @struct Structure containing all data necessary to configure and work with the graph. Including
 *         emWin handlers, graph properties and the data to be plotted.
 */
typedef struct _sGraph_t
{
        GRAPH_Handle hGraph_Handler; /** @< emWin handle to the graph widget*/
        int16_t u16ID; /** @< User ID*/
        WM_CALLBACK * cbGraphCallback; /** @< holds the Graph callback*/
        GRAPH_USERDRAW_CALLBACK * cbGraphUserDrawCallback; /** @< holds the user draw callback*/
        GRAPH_SCALE_Handle hVerticalScale_Handler; /** @< emWin handler to the vertical scale*/
        GRAPH_SCALE_Handle hHorizontalScale_Handler; /** @< emWin handler to the horizontal scale*/

        /** @struct Structure to configure the graph properties*/
        struct
        {
                int16_t x0; /** @< Starting X position*/
                int16_t y0; /** @< Starting y position*/
                int16_t xSize; /** @< X Size of the widget*/
                int16_t ySize; /** @< Y Size of the widget*/
                GUI_COLOR backgroundColor; /** @< Background color of the graph*/
                GUI_COLOR frameColor; /** @< Frame color of the graph*/
        } sGraphProps;

        /** @struct Structure to the data curve*/
        struct
        {
                int16_t TIndex; /** @< Time index to count data*/
                GUI_RECT rectSweepBar; /** @< Rectangle to be the sweep bar*/
                GUI_COLOR sweepBarColor; /** @< Color of sweep bar*/
                uint8_t u8SweepBarSize; /** @< Size of the sweep bar*/
                uint8_t u8NumData; /** @< Number of data curves*/
                int16_t u16XSize; /** @< Number os points*/
                sGraphData_t* graphData;
        } sDataOnGraph;

        /** @struct Structure to configure the grids properties*/
        struct
        {
                GUI_COLOR gridColor; /** @< Color of grid*/
                uint16_t distX; /** @< X Distance in pixels of grid lines*/
                uint16_t distY; /** @< Y Distance in pixels of grid lines*/
                uint8_t u8LineStyleH; /** @< grid horizontal line style*/
                uint8_t u8LineStyleV; /** @< grid vertical line style*/
        } sGridProps;

        /** @struct Structure to configure the scales properties*/
        struct
        {
                bool isVerticalScaleOn; /** @< Tells that the vertical scale is visible*/
                bool isHorizontalScaleOn; /** @< Tells that the horizontal scale is visible*/
                GUI_COLOR textColor; /** @< Color of the scale texts*/
                float32_t fScaleFactorH; /** @< Horizontal Scale Factor for the major ticks*/
                float32_t fScaleFactorV; /** @< Vertical Scale Factor for the major ticks*/
                uint16_t u16PosH; /** @< Horizontal Scale start position*/
                uint16_t u16PosV; /** @< Vertical Scale start position*/
                uint16_t u16TickDistH; /** @< Horizontal Scale major tick distance*/
                uint16_t u16TickDistV; /** @< Vertical Scale major tick distance*/
        } sScaleProps;

        /** @struct Structure to configure the border properties*/
        struct
        {
                uint16_t u16BorderL; /** @< Size in pixels of the left border*/
                uint16_t u16BorderT; /** @< Size in pixels of the top border*/
                uint16_t u16BorderR; /** @< Size in pixels of the right border*/
                uint16_t u16BorderB; /** @< Size in pixels of the bottom border*/
                GUI_COLOR borderColor;
        } sBorderProps;
} sGraph_t;

/*************************************************************************************************/
/**
 * @brief Creates the graph
 * @param graph Structure containing the internal data to configure the graph.
 * @param numData Number of data curves.
 * @param _pwin Parent Window.
 * @return 0 if success, -1 otherwise.
 */
int8_t i8Graph_create(sGraph_t* graph, uint8_t numData, WM_HWIN _pwin);

/*************************************************************************************************/
/**
 * @brief Change the color of a curve
 * @param graph Structure containing the internal data to configure the graph.
 * @param color Curve Color.
 * @param dataNumber Number of the data curve.
 * @return void.
 */
void vGraph_SetDataColor(sGraph_t* graph, GUI_COLOR color, uint8_t dataNumber);

/*************************************************************************************************/
/**
 * @brief Adds a value to a data curve.
 * @param graph Structure containing the internal data to configure the graph.
 * @param value Value to be plotted.
 * @param dataNumber Number of the data curve.
 * @attention to properly exhibit the data the function vGraph_UpdateGraphData need to be called after
 *            adding the desired data to all data curves
 * @return void.
 */
void vGraph_AddData(sGraph_t* graph, int16_t value, uint8_t dataNumber);

/*************************************************************************************************/
/**
 * @brief Clear the data curves (all) and return the sweep bar to the beginning
 * @param graph Structure containing the internal data to configure the graph.
 * @return void.
 */
void vGraph_ClearData(sGraph_t* graph);

/*************************************************************************************************/
/**
 * @brief Plot the data curves into the graph.
 * @param graph Structure containing the internal data to configure the graph.
 * @attention This function should be called after vGraph_AddData
 * @return void.
 */
void vGraph_UpdateGraphData(sGraph_t* graph);

/*************************************************************************************************/
/**
 * @brief Clears the data curves (all) and return the sweep bar to the beginning
 * @param graph Structure containing the internal data to configure the graph.
 * @return void.
 */
void vGraph_ClearData(sGraph_t* graph);

#endif /* SOURCE_GRAPHWIDGET_H_ */
