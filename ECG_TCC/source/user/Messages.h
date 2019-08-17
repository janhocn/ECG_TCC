/**
 ******************************************************************************
 * @file    Messages.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    9 November 2017
 * @brief   All messages for the projects are placed here in English and Portuguese
 ******************************************************************************
 */
#ifndef USER_MESSAGES_H_
#define USER_MESSAGES_H_

/*************************************************************************************************/
/**
 * @enum Languages supported by the project
 */
typedef enum _language
{
    _E_ENGLISH, /** @< English */
    _E_PORTUGUESE, /** @< Portuguese */
    _E_NUM_LANGUAGES /** @< Number of supported languages */
} eLanguage;

/*************************************************************************************************/
/**
 * @def Macro to check if the language being set is valid (if not assume portuguese)
 */
#define __CHECK_LANGUAGE(__X__)   if((__X__ == _E_ENGLISH) || \
							      (__X__ == _E_PORTUGUESE)) \
								  {							\
										         			\
								  }							\
								  else						\
								  {  						\
									  __X__	= _E_PORTUGUESE; \
								  }							\

/*************************************************************************************************/
/**
 * @struct Structure to hold pointers to all messages
 */
typedef struct _Messages
{
    /*************************************************************************************************/
    /**
     * @struct Messages related to the Home module
     */
    struct _Home
    {
        const char * ecgIcon; /** @< Home ECG Icon label */
        const char * spO2Icon; /** @< Home SPO2 Icon label */
        const char * settingsIcon; /** @< Home Settings Icon label */
        const char * aboutIcon; /** @< Home About Icon label */
        const char * helpIcon; /** @< Home Help Icon label */
        const char * debugIcon; /** @< Home Debug Icon label */
    } msgHome[_E_NUM_LANGUAGES];

    /*************************************************************************************************/
    /**
     * @struct Messages related to the Settings module
     */
    struct _Settings
    {
        const char * pressTouch; /** @< Settings press touch screen message */
        const char * touchCalibration; /** @< Settings touch calibration title message */
        const char * calibrationDone; /** @< Settings touch calibration message */
        const char * touchRing; /** @< Settings press touch ring message */
        const char * itemBrightness; /** @< Settings Brightness item text */
        const char * itemTime; /** @< Settings Time item text */
        const char * itemDate; /** @< Settings Date item text */
        const char * itemBattery; /** @< Settings Battery item text */
        const char * itemTouchCal; /** @< Settings Touch Calibration item text */
        const char * itemLanguage; /** @< Settings Language item text */
        const char * itemTextBrightness; /** @< Settings Brightness item description text */
        const char * itemTextTime; /** @< Settings Time item description text */
        const char * itemTextDate; /** @< Settings Date item description text */
        const char * itemTextBattery; /** @< Settings Battery item description text */
        const char * itemTextTouchCal; /** @< Settings Touch Calibration item description text */
        const char * itemTextLanguage; /** @< Settings Language item description text */
        const char * sepTextGeneral; /** @< Settings Separation General text */
        const char * sepTextInput; /** @< Settings Separation Input text */
        const char * buttonOk; /** @< Settings Calendar Button Ok text */
        const char * buttonCancel; /** @< Settings Calendar Button Cancel text */
        const char * buttonSaveTime; /** @< Settings Time Button Save text */
        const char * buttonSaveLang; /** @< Settings Language Button Save text */
        const char * batteryStatusTitle; /** @< Settings Battery Status Title text */
        const char * batteryStatusVoltage; /** @< Settings Battery Status Voltage text */
        const char * batteryStatusCurrent; /** @< Settings Battery Status Current text */
        const char * batteryStatusPower; /** @< Settings Battery Status Power text */
        const char * batteryStatusCapacity; /** @< Settings Battery Status Capacity text */
        const char * batteryStatusTemperature; /** @< Settings Battery Status Temperature text */
        const char * batteryStatusStateCharge; /** @< Settings Battery Status State of Charge text */
        const char * batteryStatusStateHealth; /** @< Settings Battery Status State of Health text */
    } msgSettings[_E_NUM_LANGUAGES];

    /*************************************************************************************************/
    /**
     * @struct Messages related to the Display module
     */
    struct _Display
    {
        const char * projectName; /** @< Project Name */
        const char * version; /** @< Firmware Version */
        const char * debugTitle; /** @< Debug window title */
        const char * debugInfo; /** @< Debug window information text */
    } msgDisplay[_E_NUM_LANGUAGES];

    /*************************************************************************************************/
    /**
     * @struct Messages related to the Initialization screen
     */
    struct _Initialization
    {
        const char * title; /** @< Project Name */
        const char * initSDCard; /** @< SDCard and File system */
        const char * insertSDCard; /** @< Insert the SD Card */
        const char * SDCardOK; /** @< File System initialized*/
        const char * SDCardNOK; /** @< File System not initialized*/
        const char * initClock; /** @< RTC */
        const char * initDisplay; /** @< Display and GUI */
        const char * initECG; /** @< ECG tasks and rountines*/
        const char * initSPO2; /** @< SPO2 Driver and tasks */
        const char * initDone; /** @< Done */
    } msgInitialization[_E_NUM_LANGUAGES];

    /*************************************************************************************************/
    /**
     * @struct Messages related to the ECG Graph screen
     */
    struct _ECGGrpah
    {
        const char * startLog; /** @< Checkbox label */
        const char * timeAxis; /** @< Time axis label */
        const char * LogFinished; /** @< Messagebox log finished */
    } msgECGGrpah[_E_NUM_LANGUAGES];

    /*************************************************************************************************/
    /**
     * @struct Messages related to the SPO2 Graph screen
     */
    struct _SPO2Grpah
    {
        const char * startLog; /** @< Checkbox label */
        const char * timeAxis; /** @< Time axis label */
        const char * LogFinished; /** @< Messagebox log finished */
    } msgSPO2Graph[_E_NUM_LANGUAGES];

    /*************************************************************************************************/
    /**
     * @struct Messages related to the help window
     */
    struct _HelpBox
    {
        const char * title; /** @< Help window title */
        const char * text; /** @< Help window text */
    } msgHelpText[_E_NUM_LANGUAGES];

} stMessages;

/*************************************************************************************************/
/**
 * @brief Initialize the language Module
 *        Load the language from the flash memory
 * @return void.
 */
void vMessages_Init(void);

/*************************************************************************************************/
/**
 * @brief Set the current language
 * @param lang Language
 * @return void.
 */
void vMessages_setLanguage(eLanguage lang);

/*************************************************************************************************/
/**
 * @brief Returns the pointer to the language variable
 * @return Pointer to _LanguageSet.
 */
eLanguage * xMessages_getLanguageAddr(void);

struct _ECGGrpah * xMessage_getECGGraph(void);
struct _SPO2Grpah * xMessage_getSPO2Graph(void);
struct _Home * xMessage_getMessageHome(void);
struct _Settings * xMessage_getMessageSettings(void);
struct _Display * xMessage_getMessageDisplay(void);
struct _HelpBox * xMessage_getMessageHelp(void);
struct _Initialization * xMessage_getMessageInitialization(void);
/*************************************************************************************************/
#endif /* USER_MESSAGES_H_ */
