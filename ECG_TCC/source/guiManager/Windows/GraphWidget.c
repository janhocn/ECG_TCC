/**
 ******************************************************************************
 * @file    GraphWidget.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    19 February 2018
 * @brief   Module to use the graph emwin widget with sweep bar
 ******************************************************************************
 */
#include "GraphWidget.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "WM.h"
#include "GRAPH.h"
#include "FreeRTOS.h"
#include "portable.h"
/*************************************************************************************************/
/**
 * @brief Callback function to override the default paint scheme of the Graph Widget.
 * @param pMsg Message from the emWin API.
 * @return void.
 */
static void _cbGraph_CallBackGraph(WM_MESSAGE *pMsg);

/*************************************************************************************************/
/**
 * @brief Function to draw a sweep bar on the graph.
 * @param graph Structure containing the internal data.
 * @param RectSweep Pointer to the Rectangle to be drawn.
 * @param x0 Left Position X.
 * @param y0 Left Position Y.
 * @param y1 Right Position Y.
 * @param width Width in pixels.
 * @return void.
 */
static void _Graph_DrawSweepBar(
        sGraph_t* graph,
        GUI_RECT* RectSweep,
        uint16_t x0,
        uint16_t y0,
        uint16_t y1,
        uint16_t width);

/*************************************************************************************************/
static void _Graph_DrawSweepBar(
        sGraph_t* graph,
        GUI_RECT* RectSweep,
        uint16_t x0,
        uint16_t y0,
        uint16_t y1,
        uint16_t width)
{

    if(graph == NULL)
    {
        return;
    }

    GUI_SetColor(graph->sDataOnGraph.sweepBarColor);
    if (!graph->sDataOnGraph.TIndex)
    {
        RectSweep->x0 = x0;
    }
    else
    {
        RectSweep->x0 = x0 - 1;
    }
    RectSweep->y0 = y0;

    if (graph->sDataOnGraph.TIndex >= (graph->sDataOnGraph.u16XSize - 8))
    {
        RectSweep->x1 = x0;
    }
    else
    {
        RectSweep->x1 = RectSweep->x0 + width;
    }
    RectSweep->y1 = y1;
}

/*************************************************************************************************/
static void _cbGraph_CallBackGraph(WM_MESSAGE *pMsg)
{
    GUI_RECT Rect;
    sGraph_t* graph;

    switch (pMsg->MsgId)
    {
        case WM_PAINT:

            GRAPH_Callback(pMsg);

            GRAPH_GetUserData(pMsg->hWin, (void *) &graph, sizeof(void *));
            if (graph == NULL)
            {
                break;
            }
            WM_GetInsideRectExScrollbar(graph->hGraph_Handler, &Rect);

            Rect.x0 += graph->sBorderProps.u16BorderL;
            Rect.y0 += graph->sBorderProps.u16BorderT;
            Rect.x1 -= graph->sBorderProps.u16BorderR;
            Rect.y1 -= graph->sBorderProps.u16BorderB;

            for (uint8_t i = 0; i < graph->sDataOnGraph.u8NumData; i++)
            {
                GUI_SetColor(graph->sDataOnGraph.graphData[i].lineColor);
                GUI_DrawGraphEx(graph->sDataOnGraph.graphData[i].YData, graph->sDataOnGraph.u16XSize, Rect.x0, Rect.y1,
                        -1, 1, 0);
            }

            _Graph_DrawSweepBar(graph, &graph->sDataOnGraph.rectSweepBar, graph->sDataOnGraph.TIndex + Rect.x0, Rect.y0,
                    Rect.y1, graph->sDataOnGraph.u8SweepBarSize);
            GUI_FillRectEx(&graph->sDataOnGraph.rectSweepBar);
            break;

        default:
            GRAPH_Callback(pMsg);
            break;
    }
}

/*************************************************************************************************/
void vGraph_SetDataColor(sGraph_t* graph, GUI_COLOR color, uint8_t dataNumber)
{
    if (graph == NULL)
    {
        return;
    }

    if (dataNumber < graph->sDataOnGraph.u8NumData)
    {
        graph->sDataOnGraph.graphData[dataNumber].lineColor = color;
    }
}

/*************************************************************************************************/
void vGraph_AddData(sGraph_t* graph, int16_t value, uint8_t dataNumber)
{
    if (graph == NULL)
    {
        return;
    }
    if (dataNumber < graph->sDataOnGraph.u8NumData)
    {
        if (graph->sDataOnGraph.graphData[dataNumber].YData)
        {
            graph->sDataOnGraph.graphData[dataNumber].YData[graph->sDataOnGraph.TIndex] = value;
        }
    }
}

/*************************************************************************************************/
void vGraph_ClearData(sGraph_t* graph)
{
    if (graph == NULL)
    {
        return;
    }
    graph->sDataOnGraph.TIndex = 0;
    for (uint8_t i = 0; i < graph->sDataOnGraph.u8NumData; i++)
    {
        memset(graph->sDataOnGraph.graphData[i].YData, 0x00, graph->sDataOnGraph.u16XSize);
    }

    /* Invalidate only the rectSweepBar area*/
    WM_InvalidateRect(graph->hGraph_Handler, &graph->sDataOnGraph.rectSweepBar);
}

/*************************************************************************************************/
void vGraph_UpdateGraphData(sGraph_t* graph)
{
    if(graph == NULL)
    {
        return;
    }
    graph->sDataOnGraph.TIndex++;
    if (graph->sDataOnGraph.TIndex >= graph->sDataOnGraph.u16XSize)
    {
        graph->sDataOnGraph.TIndex = 0;
    }

    /* Invalidate only the rectSweepBar area*/
    WM_InvalidateRect(graph->hGraph_Handler, &graph->sDataOnGraph.rectSweepBar);
}

/*************************************************************************************************/
int8_t i8Graph_create(sGraph_t* graph, uint8_t numData, WM_HWIN _pwin)
{
    void * pData;
    /* Create widget */
    graph->hGraph_Handler = GRAPH_CreateUser(graph->sGraphProps.x0, graph->sGraphProps.y0, graph->sGraphProps.xSize,
            graph->sGraphProps.ySize, _pwin, WM_CF_SHOW | WM_CF_MEMDEV,
            GRAPH_CF_GRID_FIXED_X, graph->u16ID, sizeof(void *));


    GRAPH_SetColor(graph->hGraph_Handler, graph->sGraphProps.backgroundColor, GRAPH_CI_BK);
    GRAPH_SetColor(graph->hGraph_Handler, graph->sGraphProps.frameColor, GRAPH_CI_FRAME);

    /* Configure Grid */
    GRAPH_SetGridVis(graph->hGraph_Handler, 1);
    GRAPH_SetGridFixedX(graph->hGraph_Handler, 1);
    GRAPH_SetGridDistX(graph->hGraph_Handler, graph->sGridProps.distX);
    GRAPH_SetGridDistY(graph->hGraph_Handler, graph->sGridProps.distY);
    GRAPH_SetLineStyleH(graph->hGraph_Handler, graph->sGridProps.u8LineStyleH);
    GRAPH_SetLineStyleV(graph->hGraph_Handler, graph->sGridProps.u8LineStyleV);
    GRAPH_SetColor(graph->hGraph_Handler, graph->sGridProps.gridColor, GRAPH_CI_GRID);

    /* Configure Border */
    GRAPH_SetBorder(graph->hGraph_Handler, graph->sBorderProps.u16BorderL, graph->sBorderProps.u16BorderT,
            graph->sBorderProps.u16BorderR, graph->sBorderProps.u16BorderB);
    GRAPH_SetColor(graph->hGraph_Handler, graph->sBorderProps.borderColor, GRAPH_CI_BORDER);

    /* Configure Scales */
    graph->hVerticalScale_Handler = GRAPH_SCALE_Create(graph->sScaleProps.u16PosV, GUI_TA_RIGHT,
    GRAPH_SCALE_CF_VERTICAL, graph->sScaleProps.u16TickDistV);
    graph->hHorizontalScale_Handler = GRAPH_SCALE_Create(graph->sScaleProps.u16PosH, GUI_TA_HCENTER,
    GRAPH_SCALE_CF_HORIZONTAL, graph->sScaleProps.u16TickDistH);

    if (graph->sScaleProps.isVerticalScaleOn)
    {
        GRAPH_SCALE_SetTextColor(graph->hVerticalScale_Handler, graph->sScaleProps.textColor);
        GRAPH_SCALE_SetFactor(graph->hVerticalScale_Handler, graph->sScaleProps.fScaleFactorV);
        GRAPH_AttachScale(graph->hGraph_Handler, graph->hVerticalScale_Handler);
    }

    if (graph->sScaleProps.isHorizontalScaleOn)
    {
        GRAPH_SCALE_SetTextColor(graph->hHorizontalScale_Handler, graph->sScaleProps.textColor);
        GRAPH_SCALE_SetFactor(graph->hHorizontalScale_Handler, graph->sScaleProps.fScaleFactorH);
        GRAPH_AttachScale(graph->hGraph_Handler, graph->hHorizontalScale_Handler);
    }

    graph->sDataOnGraph.graphData = (sGraphData_t *) pvPortMalloc(numData * sizeof(sGraphData_t));

    if (graph->sDataOnGraph.graphData == NULL)
    {
        return -1;
    }

    graph->sDataOnGraph.u8NumData = numData;
    for (uint8_t i = 0; i < graph->sDataOnGraph.u8NumData; i++)
    {
        graph->sDataOnGraph.graphData[i].YData = (int16_t *) pvPortMalloc(
                graph->sDataOnGraph.u16XSize * sizeof(int16_t));
        if (graph->sDataOnGraph.graphData[i].YData == NULL)
        {
            return -1;
        }
        memset(graph->sDataOnGraph.graphData[i].YData, 0x00, graph->sDataOnGraph.u16XSize);
    }

    graph->sDataOnGraph.TIndex = 0;

    if (graph->cbGraphUserDrawCallback != NULL)
    {
        GRAPH_SetUserDraw(graph->hGraph_Handler, graph->cbGraphUserDrawCallback);
    }

    pData = (void *) graph;
    GRAPH_SetUserData(graph->hGraph_Handler, (void *) &pData, sizeof(void *));

    graph->cbGraphCallback = _cbGraph_CallBackGraph;
    WM_SetCallback(graph->hGraph_Handler, graph->cbGraphCallback);

    return 0;
}
/*************************************************************************************************/
