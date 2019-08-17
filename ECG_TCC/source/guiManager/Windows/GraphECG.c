/**
 ******************************************************************************
 * @file    GraphECG.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    19 February 2018
 * @brief   Module to use the ECG graph window.
 ******************************************************************************
 */
#include "GraphECG.h"

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
#include "HomeIcon.h"
#include "GraphCommon.h"
#include "GraphWidget.h"
#include "GUI_IDS.h"
#include "Messages.h"
#include "logTask.h"
/*************************************************************************************************/
/**
 * @var data structure containing all elements in this module.
 */
static sGraphECGPack_t GraphECGPack;

/*************************************************************************************************/
/**
 * @brief Creates the window, graph widget and texts.
 * @param _pwin Parent Window.
 * @return void.
 */
static void _GraphECG_CreateGraphECG(WM_HWIN _pwin);

/*************************************************************************************************/
/**
 * @brief User Draw Callback for the graph widget (customize the scales).
 * @param hWin Parent Window.
 * @param Stage emWin Stage.
 * @return void.
 */
static void _cbGraphECG_DrawGraphTitles(WM_HWIN hWin, int Stage);

/*************************************************************************************************/
/**
 * @brief Callback function to the window (Paint the background and initialize the sibling windows).
 * @param pMsg Message from the emWin API.
 * @return void.
 */
static void _cbGraphECG_CallBackGraphECGWindow(WM_MESSAGE * pMsg);

/*************************************************************************************************/
static void _cbGraphECG_DrawGraphTitles(WM_HWIN hWin, int Stage)
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
            GUI_DispStringInRectEx(xMessage_getECGGraph()->timeAxis, &Rect, GUI_TA_HCENTER,
                    strlen(xMessage_getECGGraph()->timeAxis), GUI_ROTATE_0);
            break;

        case GRAPH_DRAW_FIRST:
            break;
    }
}

/*************************************************************************************************/
static void _cbGraphECG_CallBackGraphECGWindow(WM_MESSAGE * pMsg)
{
    int NCode, Id;
    volatile int LogChecked = 0;
    switch (pMsg->MsgId)
    {
        case WM_CREATE:
            _GraphECG_CreateGraphECG(pMsg->hWin);
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
                    if (Id == ECG_LOG_CHECKBOX_ID)
                    {
                        LogChecked = CHECKBOX_IsChecked(GraphECGPack.hLOGCheckbox);
                        if (LogChecked == 1)
                        {
                            vLogTask_StartLog(_LOG_ECG_DATA);
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
void GraphECG_UncheckLog(void)
{
    CHECKBOX_SetState(GraphECGPack.hLOGCheckbox, 0);
}

/*************************************************************************************************/
static void _GraphECG_CreateGraphECG(WM_HWIN _pwin)
{
    GraphECGPack.GraphECGWidget.u16ID = ECG_GRAPH_ID;
    GraphECGPack.GraphECGWidget.cbGraphUserDrawCallback = _cbGraphECG_DrawGraphTitles;
    GraphECGPack.GraphECGWidget.sGraphProps.x0 = XStartGraph;
    GraphECGPack.GraphECGWidget.sGraphProps.y0 = YStartGraph;
    GraphECGPack.GraphECGWidget.sGraphProps.xSize = WinXSize;
    GraphECGPack.GraphECGWidget.sGraphProps.ySize = WinYSize;
    GraphECGPack.GraphECGWidget.sGraphProps.backgroundColor = BKGraphGreenCustom;
    GraphECGPack.GraphECGWidget.sGraphProps.frameColor = GUI_BLACK;

    GraphECGPack.GraphECGWidget.sDataOnGraph.u16XSize = XMAXDisplay;
    GraphECGPack.GraphECGWidget.sDataOnGraph.sweepBarColor = GUI_BLACK;
    GraphECGPack.GraphECGWidget.sDataOnGraph.u8SweepBarSize = 20;

    GraphECGPack.GraphECGWidget.sGridProps.gridColor = GUI_DARKGREEN;
    GraphECGPack.GraphECGWidget.sGridProps.distX = HGridPX;
    GraphECGPack.GraphECGWidget.sGridProps.distY = VGridPX;
    GraphECGPack.GraphECGWidget.sGridProps.u8LineStyleH = GUI_LS_DOT;
    GraphECGPack.GraphECGWidget.sGridProps.u8LineStyleV = GUI_LS_DOT;

    GraphECGPack.GraphECGWidget.sBorderProps.borderColor = GUI_BLACK;
    GraphECGPack.GraphECGWidget.sBorderProps.u16BorderB = BORDERBOTTOMPX;
    GraphECGPack.GraphECGWidget.sBorderProps.u16BorderT = BORDERTOPPX;
    GraphECGPack.GraphECGWidget.sBorderProps.u16BorderL = BORDERLEFTPX;
    GraphECGPack.GraphECGWidget.sBorderProps.u16BorderR = BORDERRIGHTPX;

    GraphECGPack.GraphECGWidget.sScaleProps.isHorizontalScaleOn = true;
    GraphECGPack.GraphECGWidget.sScaleProps.isVerticalScaleOn = false;
    GraphECGPack.GraphECGWidget.sScaleProps.fScaleFactorH = 6.0;
    GraphECGPack.GraphECGWidget.sScaleProps.fScaleFactorV = 1.0;
    GraphECGPack.GraphECGWidget.sScaleProps.textColor = GUI_WHITE;
    GraphECGPack.GraphECGWidget.sScaleProps.u16TickDistH = HGridPX * 2;
    GraphECGPack.GraphECGWidget.sScaleProps.u16TickDistV = VGridPX;
    GraphECGPack.GraphECGWidget.sScaleProps.u16PosH = YMAXDisplay + BORDERTOPPX + 3;
    GraphECGPack.GraphECGWidget.sScaleProps.u16PosV = BORDERLEFTPX - 2;

    if (i8Graph_create(&GraphECGPack.GraphECGWidget, _E_DATA_NUMBER, _pwin) != 0)
    {
        /*ERROR*/
        while (1)
            ;
    }

    vGraph_SetDataColor(&GraphECGPack.GraphECGWidget, GUI_LIGHTGREEN, _E_DATA_ECG_RAW);
    vGraph_SetDataColor(&GraphECGPack.GraphECGWidget, GUI_LIGHTYELLOW, _E_DATA_ECG_FILTERED);

    GraphECGPack.hIconHeart = IMAGE_CreateEx((LCD_GetXSize() - 1) - (&bmHeartIcon)->XSize - 5, 40,
            (&bmHeartIcon)->XSize, (&bmHeartIcon)->YSize, _pwin, WM_CF_SHOW | WM_CF_HASTRANS | WM_CF_MEMDEV, 0,
            ECG_HEART_ID);
    IMAGE_SetBitmap(GraphECGPack.hIconHeart, &bmHeartIcon);

    GraphECGPack.hTextHeartRateText = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize - 2, 50, 80, 30,
            _pwin, WM_CF_SHOW,
            TEXT_CF_LEFT, ECG_HR_ID, "HR");
    TEXT_SetFont(GraphECGPack.hTextHeartRateText, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphECGPack.hTextHeartRateText, GUI_WHITE);
    TEXT_SetWrapMode(GraphECGPack.hTextHeartRateText, GUI_WRAPMODE_WORD);

    GraphECGPack.hTextHeartRateValue = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize - 10,
            (55 + (&bmHeartIcon)->YSize), 100, 30, _pwin, WM_CF_SHOW,
            TEXT_CF_HCENTER, ECG_HR_VALUE_ID, "000");
    TEXT_SetFont(GraphECGPack.hTextHeartRateValue, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphECGPack.hTextHeartRateValue, GUI_WHITE);
    TEXT_SetWrapMode(GraphECGPack.hTextHeartRateValue, GUI_WRAPMODE_WORD);

    GraphECGPack.hTextBPMUnit = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize - 10,
            55 + (&bmHeartIcon)->YSize + 30, 100, 30, _pwin, WM_CF_SHOW,
            TEXT_CF_HCENTER, ECG_BPM_ID, "BPM");
    TEXT_SetFont(GraphECGPack.hTextBPMUnit, &GUI_Font32B_1);
    TEXT_SetTextColor(GraphECGPack.hTextBPMUnit, GUI_WHITE);
    TEXT_SetWrapMode(GraphECGPack.hTextBPMUnit, GUI_WRAPMODE_WORD);

    GraphECGPack.hTextRRInterval = TEXT_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize - 2,
            55 + (&bmHeartIcon)->YSize + 30 + 50, 80, 30, _pwin, WM_CF_SHOW,
            TEXT_CF_LEFT, ECG_RR_ID, "RR: 0000 ms");
    TEXT_SetFont(GraphECGPack.hTextRRInterval, &GUI_Font16B_1);
    TEXT_SetTextColor(GraphECGPack.hTextRRInterval, GUI_WHITE);
    TEXT_SetWrapMode(GraphECGPack.hTextRRInterval, GUI_WRAPMODE_WORD);
    GraphECGPack.toggleTextHR = false;

    GraphECGPack.hIconLog = IMAGE_CreateEx((LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize - 5,
            (LCD_GetVYSize() - 1) - 2 * (&bmHomeIcon)->YSize - 5, (&bmlogIcon2)->XSize, (&bmlogIcon2)->YSize, _pwin,
            WM_CF_SHOW | WM_CF_HASTRANS | WM_CF_MEMDEV, 0, ECG_LOGICON_ID);
    IMAGE_SetBitmap(GraphECGPack.hIconLog, &bmlogIcon2);

    GraphECGPack.hLOGCheckbox = CHECKBOX_Create(
            (LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize + (&bmlogIcon2)->XSize + 2 - 5,
            (LCD_GetVYSize() - 1) - 2 * (&bmHomeIcon)->YSize - 5, 20, 20, _pwin, ECG_LOG_CHECKBOX_ID,
            WM_CF_SHOW | WM_CF_MEMDEV);

    GraphECGPack.hLogText = TEXT_CreateEx(
            (LCD_GetXSize() - 1) - 2 * (&bmHeartIcon)->XSize + (&bmlogIcon2)->XSize + 2 - 5 + 21,
            (LCD_GetVYSize() - 1) - 2 * (&bmHomeIcon)->YSize, 80, 20, _pwin, WM_CF_SHOW,
            TEXT_CF_LEFT, ECG_LOG_TEXT_ID, xMessage_getECGGraph()->startLog);
    TEXT_SetFont(GraphECGPack.hLogText, &GUI_Font10_1);
    TEXT_SetTextColor(GraphECGPack.hLogText, GUI_WHITE);

    vGraphECG_ClearMainGraph();
}

/*************************************************************************************************/
void vGraphECG_setRRInterval(uint16_t value)
{
    uint8_t buffer[12] = { 0 };
    sprintf((char *) buffer, "RR: %04d ms", value);
    TEXT_SetTextColor(GraphECGPack.hTextRRInterval, GUI_WHITE);
    TEXT_SetText(GraphECGPack.hTextRRInterval, (const char *) buffer);
}
/*************************************************************************************************/
void vGraphECG_toggleHRTextColor(void)
{
    GraphECGPack.toggleTextHR = !GraphECGPack.toggleTextHR;
}

/*************************************************************************************************/
void vGraphECG_setHeartRateValue(uint16_t value)
{
    uint8_t buffer[5] = { 0 };
    GUI_COLOR colorWarning = GUI_RED;

    if (GraphECGPack.toggleTextHR)
    {
        TEXT_SetTextColor(GraphECGPack.hTextHeartRateText, GUI_RED);
    }
    else
    {
        TEXT_SetTextColor(GraphECGPack.hTextHeartRateText, GUI_WHITE);
    }

    if (value > 300)
    {
        /* Possibly wrong reading */
        TEXT_SetTextColor(GraphECGPack.hTextHeartRateValue, colorWarning);
        TEXT_SetText(GraphECGPack.hTextHeartRateValue, (const char *) "---");
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
    TEXT_SetTextColor(GraphECGPack.hTextHeartRateValue, colorWarning);
    TEXT_SetText(GraphECGPack.hTextHeartRateValue, (const char *) buffer);
}

/*************************************************************************************************/
void vGraphECG_AddGraphECGScreen(void)
{
    GraphECGPack.hWindowGraphECG = WM_CreateWindowAsChild(0, 0, LCD_GetXSize(), LCD_GetYSize(), WM_HBKWIN,
    WM_CF_HIDE | WM_CF_MEMDEV, _cbGraphECG_CallBackGraphECGWindow, 0);
}

/*************************************************************************************************/
void vGraphECG_ClearMainGraph(void)
{
    vGraph_ClearData(&GraphECGPack.GraphECGWidget);
}

/*************************************************************************************************/
void vGraphECG_AddDataMainGraph(uint16_t value, eGraphECGData Data)
{
    vGraph_AddData(&GraphECGPack.GraphECGWidget, value, (uint8_t) Data);
}

/*************************************************************************************************/
void vGraphECG_PostData(void)
{
    vGraph_UpdateGraphData(&GraphECGPack.GraphECGWidget);
}

/*************************************************************************************************/
WM_HWIN hGraphECG_getWindow(void)
{
    return GraphECGPack.hWindowGraphECG;
}
/*************************************************************************************************/
