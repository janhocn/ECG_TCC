/**
 ******************************************************************************
 * @file    GraphSPO2.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    19 February 2018
 * @brief   Module to use the SPO2 graph window.
 ******************************************************************************
 */
#include "GraphSPO2.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "MK64F12.h" 
#include "fsl_clock.h"
/* SEGGER emWin includes. */
#include "ICONVIEW.h"
#include "TEXT.h"
#include "IMAGE.h"
/* User includes. */
#include "Display.h"
#include "Home.h"
#include "HeartIcon.h"
#include "LogIcon.h"
#include "SPO2ValueIcon.h"
#include "GraphCommon.h"
#include "GraphWidget.h"
#include "HomeIcon.h"
#include "GUI_IDS.h"
#include "logTask.h"
#include "Messages.h"
/*************************************************************************************************/
/**
 * @var data structure containing all elements in this module.
 */
static sGraphSPO2Pack_t GraphSPO2Pack;
/*************************************************************************************************/
/**
 * @brief Creates the window, graph widget and texts.
 * @param _pwin Parent Window.
 * @return void.
 */
static void _GraphSPO2_CreateGraphSPO2(WM_HWIN _pwin);

/*************************************************************************************************/
/**
 * @brief User Draw Callback for the graph widget (customize the scales).
 * @param hWin Parent Window.
 * @param Stage emWin Stage.
 * @return void.
 */
static void _cbGraphSPO2_DrawGraphTitles(WM_HWIN hWin, int Stage);

/*************************************************************************************************/
/**
 * @brief Callback function to the window (Paint the background and initialize the sibling windows).
 * @param pMsg Message from the emWin API.
 * @return void.
 */
static void _cbGraphSPO2_CallBackGraphSPO2Window(WM_MESSAGE * pMsg);

/*************************************************************************************************/
static void _cbGraphSPO2_DrawGraphTitles(WM_HWIN hWin, int Stage)
{
    GUI_RECT Rect, RectInvalid;
    int FontSizeY;

    switch (Stage)
    {

        case GRAPH_DRAW_LAST:
            GUI_SetFont(GUI_FONT_13_1);
            FontSizeY = GUI_GetFontSizeY();
            WM_GetInsideRect(&Rect);
            WM_GetInvalidRect(hWin, &RectInvalid);
            Rect.x1 = Rect.x0 + FontSizeY;
            GUI_SetColor(GUI_WHITE);
            Rect.x1 = Rect.x0 + XMAXDisplay + BORDERLEFTPX + BORDERRIGHTPX;
            Rect.y0 = YMAXDisplay + BORDERTOPPX * 2 + 4;
            Rect.y1 = Rect.y0 + FontSizeY;
            GUI_DispStringInRectEx(xMessage_getSPO2Graph()->timeAxis, &Rect, GUI_TA_HCENTER,
                    strlen(xMessage_getSPO2Graph()->timeAxis), GUI_ROTATE_0);
            break;

        case GRAPH_DRAW_FIRST:
            break;
    }
}

/*************************************************************************************************/
static void _cbGraphSPO2_CallBackGraphSPO2Window(WM_MESSAGE * pMsg)
{
    int NCode, Id;
    int LogChecked = 0;
    switch (pMsg->MsgId)
    {
        case WM_CREATE:
            _GraphSPO2_CreateGraphSPO2(pMsg->hWin);
            break;

        case WM_PAINT:
            GUI_Clear();
            GUI_SetBkColor(GUI_BLACK);
            break;

        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc);
            NCode = pMsg->Data.v;
            switch (NCode)
            {
                case WM_NOTIFICATION_RELEASED:
                    if (Id == SPO2_LOG_CHECKBOX_ID)
                    {
                        LogChecked = CHECKBOX_IsChecked(GraphSPO2Pack.hLOGCheckbox);
                        if (LogChecked == 1)
                        {
                            vLogTask_StartLog(_LOG_SPO2_DATA);
                        }
                        else
                        {
                            vLogTask_StopLog();
                        }
                    }
                    break;
            }
            break;
        default:
            WM_DefaultProc(pMsg);
            break;
    }
}

/*************************************************************************************************/
void GraphSPO2_UncheckLog(void)
{
    CHECKBOX_SetState(GraphSPO2Pack.hLOGCheckbox, 0);
}

/*************************************************************************************************/
static void _GraphSPO2_CreateGraphSPO2(WM_HWIN _pwin)
{
    GraphSPO2Pack.GraphSPO2Widget.u16ID = SPO2_GRAPH_ID;
    GraphSPO2Pack.GraphSPO2Widget.cbGraphUserDrawCallback = _cbGraphSPO2_DrawGraphTitles;
    GraphSPO2Pack.GraphSPO2Widget.sGraphProps.x0 = XStartGraph;
    GraphSPO2Pack.GraphSPO2Widget.sGraphProps.y0 = YStartGraph;
    GraphSPO2Pack.GraphSPO2Widget.sGraphProps.xSize = WinXSize;
    GraphSPO2Pack.GraphSPO2Widget.sGraphProps.ySize = WinYSize;
    GraphSPO2Pack.GraphSPO2Widget.sGraphProps.backgroundColor = BKGraphGreenCustom;
    GraphSPO2Pack.GraphSPO2Widget.sGraphProps.frameColor = GUI_BLACK;

    GraphSPO2Pack.GraphSPO2Widget.sDataOnGraph.u16XSize = XMAXDisplay;
    GraphSPO2Pack.GraphSPO2Widget.sDataOnGraph.sweepBarColor = GUI_BLACK;
    GraphSPO2Pack.GraphSPO2Widget.sDataOnGraph.u8SweepBarSize = 20;

    GraphSPO2Pack.GraphSPO2Widget.sGridProps.gridColor = GUI_DARKGREEN;
    GraphSPO2Pack.GraphSPO2Widget.sGridProps.distX = HGridPX;
    GraphSPO2Pack.GraphSPO2Widget.sGridProps.distY = VGridPX;
    GraphSPO2Pack.GraphSPO2Widget.sGridProps.u8LineStyleH = GUI_LS_DOT;
    GraphSPO2Pack.GraphSPO2Widget.sGridProps.u8LineStyleV = GUI_LS_DOT;

    GraphSPO2Pack.GraphSPO2Widget.sBorderProps.borderColor = GUI_BLACK;
    GraphSPO2Pack.GraphSPO2Widget.sBorderProps.u16BorderB = BORDERBOTTOMPX;
    GraphSPO2Pack.GraphSPO2Widget.sBorderProps.u16BorderT = BORDERTOPPX;
    GraphSPO2Pack.GraphSPO2Widget.sBorderProps.u16BorderL = BORDERLEFTPX;
    GraphSPO2Pack.GraphSPO2Widget.sBorderProps.u16BorderR = BORDERRIGHTPX;

    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.isHorizontalScaleOn = true;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.isVerticalScaleOn = false;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.fScaleFactorH = 10.0;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.fScaleFactorV = 1.0;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.textColor = GUI_WHITE;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.u16TickDistH = HGridPX * 2;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.u16TickDistV = VGridPX;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.u16PosH = YMAXDisplay + BORDERTOPPX + 3;
    GraphSPO2Pack.GraphSPO2Widget.sScaleProps.u16PosV = BORDERLEFTPX - 2;

    if (i8Graph_create(&GraphSPO2Pack.GraphSPO2Widget, _E_DATA_SPO2_NUMBER, _pwin) != 0)
    {
        /*ERROR*/
        while (1)
            ;
    }

    vGraph_SetDataColor(&GraphSPO2Pack.GraphSPO2Widget, GUI_RED, _E_DATA_SPO2_RED_RAW);
    vGraph_SetDataColor(&GraphSPO2Pack.GraphSPO2Widget, GUI_BLUE, _E_DATA_SPO2_IR_RAW);
    vGraph_SetDataColor(&GraphSPO2Pack.GraphSPO2Widget, GUI_GREEN, _E_DATA_SPO2_FILTERED);

    GraphSPO2Pack.hIconSPO2 = IMAGE_CreateEx((LCD_GetXSize() - 1) - (&bmSPO2ValueIcon)->XSize - 30, 40,
            (&bmSPO2ValueIcon)->XSize, (&bmSPO2ValueIcon)->YSize, _pwin, WM_CF_SHOW | WM_CF_HASTRANS | WM_CF_MEMDEV, 0,
            SPO2_ICON_SPO2_ID);
    IMAGE_SetBitmap(GraphSPO2Pack.hIconSPO2, &bmSPO2ValueIcon);

    GraphSPO2Pack.hTextSPO2Text = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmSPO2ValueIcon)->XSize - 20, 100, 150, 30,
            _pwin, WM_CF_SHOW,
            TEXT_CF_LEFT, SPO2_SPO2_ID, "%SPO2");
    TEXT_SetFont(GraphSPO2Pack.hTextSPO2Text, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphSPO2Pack.hTextSPO2Text, GUI_WHITE);
    TEXT_SetWrapMode(GraphSPO2Pack.hTextSPO2Text, GUI_WRAPMODE_WORD);

    GraphSPO2Pack.hTextSPO2Value = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmSPO2ValueIcon)->XSize - 10,
            (80 + 2 + (&bmSPO2ValueIcon)->YSize), 100, 30, _pwin, WM_CF_SHOW,
            TEXT_CF_HCENTER, SPO2_SPO2_VALUE_ID, "000");
    TEXT_SetFont(GraphSPO2Pack.hTextSPO2Value, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphSPO2Pack.hTextSPO2Value, GUI_WHITE);
    TEXT_SetWrapMode(GraphSPO2Pack.hTextSPO2Value, GUI_WRAPMODE_WORD);

    GraphSPO2Pack.hTextHRText = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmSPO2ValueIcon)->XSize - 20,
            (80 + 5 + (&bmSPO2ValueIcon)->YSize) + 25, 150, 30, _pwin, WM_CF_SHOW,
            TEXT_CF_LEFT, SPO2_HR_ID, "BPM");
    TEXT_SetFont(GraphSPO2Pack.hTextHRText, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphSPO2Pack.hTextHRText, GUI_WHITE);
    TEXT_SetWrapMode(GraphSPO2Pack.hTextHRText, GUI_WRAPMODE_WORD);

    GraphSPO2Pack.hTextHRValue = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmSPO2ValueIcon)->XSize - 10,
            (80 + 5 + (&bmSPO2ValueIcon)->YSize) + 25 + 25, 100, 30, _pwin, WM_CF_SHOW,
            TEXT_CF_HCENTER, SPO2_HR_VALUE_ID, "000");
    TEXT_SetFont(GraphSPO2Pack.hTextHRValue, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphSPO2Pack.hTextHRValue, GUI_WHITE);
    TEXT_SetWrapMode(GraphSPO2Pack.hTextHRValue, GUI_WRAPMODE_WORD);

    GraphSPO2Pack.hIconLog = IMAGE_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize - 5,
            (LCD_GetVYSize() - 1) - 2 * (&bmHomeIcon)->YSize - 5, (&bmlogIcon2)->XSize, (&bmlogIcon2)->YSize, _pwin,
            WM_CF_SHOW | WM_CF_HASTRANS | WM_CF_MEMDEV, 0, SPO2_LOGICON_ID);
    IMAGE_SetBitmap(GraphSPO2Pack.hIconLog, &bmlogIcon2);

    GraphSPO2Pack.hLOGCheckbox = CHECKBOX_Create(
            (LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize + (&bmlogIcon2)->XSize + 2 - 5,
            (LCD_GetVYSize() - 1) - 2 * (&bmHomeIcon)->YSize - 5, 20, 20, _pwin, SPO2_LOG_CHECKBOX_ID,
            WM_CF_SHOW | WM_CF_MEMDEV);

    GraphSPO2Pack.hLogText = TEXT_CreateEx(
            (LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize + (&bmlogIcon2)->XSize + 2 - 5 + 21,
            (LCD_GetVYSize() - 1) - 2 * (&bmHomeIcon)->YSize, 80, 20, _pwin, WM_CF_SHOW,
            TEXT_CF_LEFT, SPO2_LOG_TEXT_ID, xMessage_getECGGraph()->startLog);
    TEXT_SetFont(GraphSPO2Pack.hLogText, &GUI_Font10_1);
    TEXT_SetTextColor(GraphSPO2Pack.hLogText, GUI_WHITE);

}

/*************************************************************************************************/
void vGraphSPO2_setSPO2Value(uint8_t value)
{
    uint8_t buffer[5] = { 0 };
    GUI_COLOR colorWarning = GUI_RED;

    if (value >= 100)
    {
        value = 100;
    }

    if (value < 85 && value > 70)
    {
        colorWarning = GUI_YELLOW;
    }
    else if (value < 70)
    {
        colorWarning = GUI_RED;
    }
    else
    {
        colorWarning = GUI_GREEN;
    }

    sprintf((char *) buffer, "%03d", value);
    TEXT_SetTextColor(GraphSPO2Pack.hTextSPO2Value, colorWarning);
    TEXT_SetText(GraphSPO2Pack.hTextSPO2Value, (const char *) buffer);
}

/*************************************************************************************************/
void vGraphSPO2_setHeartRateValue(uint16_t value)
{
    uint8_t buffer[5] = { 0 };
    GUI_COLOR colorWarning = GUI_RED;

    if (value > 300)
    {
        /* Possibly wrong reading */
        TEXT_SetTextColor(GraphSPO2Pack.hTextHRValue, colorWarning);
        TEXT_SetText(GraphSPO2Pack.hTextHRValue, (const char *) "---");
        return;
    }

    if (value > 160 && value < 180)
    {
        colorWarning = GUI_YELLOW;
    }
    else if (value > 180)
    {
        colorWarning = GUI_RED;
    }
    else if (value < 47 && value > 40)
    {
        colorWarning = GUI_YELLOW;
    }
    else if (value < 40)
    {
        colorWarning = GUI_RED;
    }
    else
    {
        colorWarning = GUI_GREEN;
    }

    sprintf((char *) buffer, "%03d", value);
    TEXT_SetTextColor(GraphSPO2Pack.hTextHRValue, colorWarning);
    TEXT_SetText(GraphSPO2Pack.hTextHRValue, (const char *) buffer);
}

/*************************************************************************************************/
void vGraphSPO2_AddGraphSPO2Screen(void)
{
    GraphSPO2Pack.hWindowGraphSPO2 = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN,
    WM_CF_HIDE | WM_CF_MEMDEV, _cbGraphSPO2_CallBackGraphSPO2Window, 0);
}

/*************************************************************************************************/
void vGraphSPO2_ClearMainGraph(void)
{
    vGraph_ClearData(&GraphSPO2Pack.GraphSPO2Widget);
}

/*************************************************************************************************/
void vGraphSPO2_AddDataMainGraph(uint16_t value, eGraphSPO2Data Data)
{
    vGraph_AddData(&GraphSPO2Pack.GraphSPO2Widget, value, (uint8_t) Data);
}

/*************************************************************************************************/
void vGraphSPO2_PostData(void)
{
    vGraph_UpdateGraphData(&GraphSPO2Pack.GraphSPO2Widget);
}

/*************************************************************************************************/
WM_HWIN hGraphSPO2_getWindow(void)
{
    return GraphSPO2Pack.hWindowGraphSPO2;
}
/*************************************************************************************************/
