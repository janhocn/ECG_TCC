/**
 ******************************************************************************
 * @file    PWM.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    2 December 2017
 * @brief   Module to control PWM
 ******************************************************************************
 */
#include "PWM.h"

/*************************************************************************************************/
/**
 * @struct Structure containning the data necessary to configure and use this driver
 */
static struct _sstPWM
{
    uint8_t _ui8CurrentBLDuttyCycle; /** @< Keep track of the last PWM value of backlight*/
} sstPWM;

/*************************************************************************************************/
void vPWM_Init(void)
{
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;

    /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber = TFT_BKL_FTM_CHANNEL;
    ftmParam.level = pwmLevel;
    ftmParam.dutyCyclePercent = 0u;
    ftmParam.firstEdgeDelayPercent = 0U;

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(TFT_BKL_FTM_BASEADDR, &ftmInfo);

    FTM_SetupPwm(TFT_BKL_FTM_BASEADDR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 24000U, FTM_SOURCE_CLOCK);
    FTM_StartTimer(TFT_BKL_FTM_BASEADDR, kFTM_SystemClock);

    vPWM_UpdateDutyCycle(_E_PWM_TFT_BACKLIGHT,25); //40% of duty cycle
}

/*************************************************************************************************/
void vPWM_UpdateDutyCycle(ePWMDevices device, uint8_t _dc)
{
    switch (device)
    {
        case _E_PWM_TFT_BACKLIGHT:
            default:
            sstPWM._ui8CurrentBLDuttyCycle = _dc;
            /* Update PWM duty cycle */
            FTM_UpdatePwmDutycycle(TFT_BKL_FTM_BASEADDR, TFT_BKL_FTM_CHANNEL, kFTM_EdgeAlignedPwm, _dc);

            /* Software trigger to update registers */
            FTM_SetSoftwareTrigger(TFT_BKL_FTM_BASEADDR, true);
            break;
    }
}

/*************************************************************************************************/
uint8_t uiPWM_GetDutyCycle(ePWMDevices device)
{
    switch (device)
    {
        case _E_PWM_TFT_BACKLIGHT:
            default:
            return sstPWM._ui8CurrentBLDuttyCycle;
    }
}
