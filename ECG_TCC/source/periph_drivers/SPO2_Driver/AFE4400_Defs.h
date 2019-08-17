/*
 * AFE4400_Defs.h
 *
 *  Created on: 5 de nov de 2017
 *      Author: Diego
 *  Description: AFE4400 hardware constants, register addresses.
 *  !!!This code was ported from https://github.com/mogar/AFE4400/!!!
 */

#ifndef PERIPH_DRIVERS_SPO2_DRIVER_AFE4400_DEFS_H_
#define PERIPH_DRIVERS_SPO2_DRIVER_AFE4400_DEFS_H_

typedef enum _eAFE4400Registers
{
    CONTROL0,
    LED2STC,
    LED2ENDC,
    LED2LEDSTC,
    LED2LEDENDC,
    ALED2STC,
    ALED2ENDC,
    LED1STC,
    LED1ENDC,
    LED1LEDSTC,
    LED1LEDENDC,
    ALED1STC,
    ALED1ENDC,
    LED2CONVST,
    LED2CONVEND,
    ALED2CONVST,
    ALED2CONVEND,
    LED1CONVST,
    LED1CONVEND,
    ALED1CONVST,
    ALED1CONVEND,
    ADCRSTSTCT0,
    ADCRSTENDCT0,
    ADCRSTSTCT1,
    ADCRSTENDCT1,
    ADCRSTSTCT2,
    ADCRSTENDCT2,
    ADCRSTSTCT3,
    ADCRSTENDCT3,
    PRPCOUNT,
    CONTROL1,
    SPARE1,
    TIAGAIN,
    TIA_AMB_GAIN,
    LEDCNTRL,
    CONTROL2,
    SPARE2,
    SPARE3,
    SPARE4,
    RESERVED1,
    RESERVED2,
    ALARM,
    LED2VAL,
    ALED2VAL,
    LED1VAL,
    ALED1VAL,
    LED2_ALED2VAL,
    LED1_ALED1VAL,
    DIAG,
} eAFE4400Registers;

#define PRF             100
#define DUTYCYCLE       25
#define AFECLK          4000000ul

/*----------------------------------------------------------------------------+
 | Bits Definition                                                         |
 +----------------------------------------------------------------------------*/

// CONTROL0 - Write Only register
#define    CONTROL0_VAL (0x000000ul)
#define    SPI_READ     (0x000001ul)        //SPI read
#define    TIM_CNT_RST  (0x000002ul)        //Timer counter reset
#define    DIAG_EN      (0x000004ul)        //Diagnostic enable
#define    SW_RST       (0x000008ul)        //Software reset

// CONTROL1 - Read/Write register
#define    CONTROL1_VAL                         (TIMEREN + NUMAV)
#define    SAMPLE_LED2_SAMPLE_LED1              (0x000000ul)        //Clocks on ALM pins
#define    LED2_PULSE_LED1_PULSE                (0x000200ul)        //Clocks on ALM pins
#define    SAMPLE_LED2_SAMPLE_LED1_PULSE        (0x000400ul)        //Clocks on ALM pins
#define    LED2_CONVERT_LED1_CONVERT            (0x000600ul)        //Clocks on ALM pins
#define    LED2_AMBIENT_LED1_AMBIENT            (0x000800ul)        //Clocks on ALM pins
#define    NO_OUTPUT_NO_OUTPUT                  (0x000A00ul)        //Clocks on ALM pins
#define    TIMEREN                              (0x000100ul)//Timer enable
//#define NUMAV                                 (0x000009ul)        //Number of averages-1 (8 bits [LSB])
#define    NUMAV                                (0x000006ul)        //Number of averages-1 (8 bits [LSB])

#define    TIAGAIN_VAL                 (0x000000ul)
#define    RF_LED1_500K                (0x000000ul)        //Program RF for LED1
#define    RF_LED1_250K                (0x000001ul)        //Program RF for LED1
#define    RF_LED1_100K                (0x000002ul)        //Program RF for LED1
#define    RF_LED1_50K                 (0x000003ul)        //Program RF for LED1
#define    RF_LED1_25K                 (0x000004ul)        //Program RF for LED1
#define    RF_LED1_10K                 (0x000005ul)        //Program RF for LED1
#define    RF_LED1_1M                  (0x000006ul)        //Program RF for LED1
#define    RF_LED1_NONE                (0x000007ul)        //Program RF for LED1

#define    CF_LED1_5P                  (0x000000ul)        //Program CF for LED1
#define    CF_LED1_5P_5P               (0x000008ul)        //Program CF for LED1
#define    CF_LED1_15P_5P              (0x000010ul)        //Program CF for LED1
#define    CF_LED1_20P_5P              (0x000018ul)        //Program CF for LED1
#define    CF_LED1_25P_5P              (0x000020ul)        //Program CF for LED1
#define    CF_LED1_30P_5P              (0x000028ul)        //Program CF for LED1
#define    CF_LED1_40P_5P              (0x000030ul)        //Program CF for LED1
#define    CF_LED1_45P_5P              (0x000038ul)        //Program CF for LED1
#define    CF_LED1_50P_5P              (0x000040ul)        //Program CF for LED1
#define    CF_LED1_55P_5P              (0x000048ul)        //Program CF for LED1
#define    CF_LED1_65P_5P              (0x000050ul)        //Program CF for LED1
#define    CF_LED1_70P_5P              (0x000058ul)        //Program CF for LED1
#define    CF_LED1_75P_5P              (0x000060ul)        //Program CF for LED1
#define    CF_LED1_80P_5P              (0x000068ul)        //Program CF for LED1
#define    CF_LED1_90P_5P              (0x000070ul)        //Program CF for LED1
#define    CF_LED1_95P_5P              (0x000078ul)        //Program CF for LED1
#define    CF_LED1_150P_5P             (0x000080ul)        //Program CF for LED1
#define    CF_LED1_155P_5P             (0x000088ul)        //Program CF for LED1
#define    CF_LED1_165P_5P             (0x000090ul)        //Program CF for LED1
#define    CF_LED1_170P_5P             (0x000098ul)        //Program CF for LED1
#define    CF_LED1_175P_5P             (0x0000A0ul)        //Program CF for LED1
#define    CF_LED1_180P_5P             (0x0000A8ul)        //Program CF for LED1
#define    CF_LED1_190P_5P             (0x0000B0ul)        //Program CF for LED1
#define    CF_LED1_195P_5P             (0x0000B8ul)        //Program CF for LED1
#define    CF_LED1_200P_5P             (0x0000C0ul)        //Program CF for LED1
#define    CF_LED1_205P_5P             (0x0000C8ul)        //Program CF for LED1
#define    CF_LED1_215P_5P             (0x0000D0ul)        //Program CF for LED1
#define    CF_LED1_220P_5P             (0x0000D8ul)        //Program CF for LED1
#define    CF_LED1_225P_5P             (0x0000E0ul)        //Program CF for LED1
#define    CF_LED1_230P_5P             (0x0000E8ul)        //Program CF for LED1
#define    CF_LED1_240P_5P             (0x0000F0ul)        //Program CF for LED1
#define    CF_LED1_245P_5P             (0x0000F8ul)        //Program CF for LED1

#define    STG2GAIN_LED1_0DB           0x000000ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_3DB           0x000100ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_6DB           0x000200ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_9DB           0x000300ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1_12DB          0x000400ul        //Stage 2 gain setting for LED1
#define    STG2GAIN_LED1               0x000700ul        //Stage 2 gain setting for LED1

#define    STAGE2EN_LED1               0x004000ul        //Stage 2 enable for LED1
#define    ENSEPGAIN                   0x008000ul

#define    TIA_AMB_GAIN_VAL            0x000000ul
#define    RF_LED2_500K                0x000000ul        //Program RF for LED2
#define    RF_LED2_250K                0x000001ul        //Program RF for LED2
#define    RF_LED2_100K                0x000002ul        //Program RF for LED2
#define    RF_LED2_50K                 0x000003ul        //Program RF for LED2
#define    RF_LED2_25K                 0x000004ul        //Program RF for LED2
#define    RF_LED2_10K                 0x000005ul        //Program RF for LED2
#define    RF_LED2_1M                  0x000006ul        //Program RF for LED2
#define    RF_LED2_NONE                0x000007ul        //Program RF for LED2

#define    CF_LED2_5P                  0x000000ul        //Program CF for LED2
#define    CF_LED2_5P_5P               0x000008ul        //Program CF for LED2
#define    CF_LED2_15P_5P              0x000010ul        //Program CF for LED2
#define    CF_LED2_20P_5P              0x000018ul        //Program CF for LED2
#define    CF_LED2_25P_5P              0x000020ul        //Program CF for LED2
#define    CF_LED2_30P_5P              0x000028ul        //Program CF for LED2
#define    CF_LED2_40P_5P              0x000030ul        //Program CF for LED2
#define    CF_LED2_45P_5P              0x000038ul        //Program CF for LED2
#define    CF_LED2_50P_5P              0x000040ul        //Program CF for LED2
#define    CF_LED2_55P_5P              0x000048ul        //Program CF for LED2
#define    CF_LED2_65P_5P              0x000050ul        //Program CF for LED2
#define    CF_LED2_70P_5P              0x000058ul        //Program CF for LED2
#define    CF_LED2_75P_5P              0x000060ul        //Program CF for LED2
#define    CF_LED2_80P_5P              0x000068ul        //Program CF for LED2
#define    CF_LED2_90P_5P              0x000070ul        //Program CF for LED2
#define    CF_LED2_95P_5P              0x000078ul        //Program CF for LED2
#define    CF_LED2_150P_5P             0x000080ul        //Program CF for LED2
#define    CF_LED2_155P_5P             0x000088ul        //Program CF for LED2
#define    CF_LED2_165P_5P             0x000090ul        //Program CF for LED2
#define    CF_LED2_170P_5P             0x000098ul        //Program CF for LED2
#define    CF_LED2_175P_5P             0x0000A0ul        //Program CF for LED2
#define    CF_LED2_180P_5P             0x0000A8ul        //Program CF for LED2
#define    CF_LED2_190P_5P             0x0000B0ul        //Program CF for LED2
#define    CF_LED2_195P_5P             0x0000B8ul        //Program CF for LED2
#define    CF_LED2_200P_5P             0x0000C0ul        //Program CF for LED2
#define    CF_LED2_205P_5P             0x0000C8ul        //Program CF for LED2
#define    CF_LED2_215P_5P             0x0000D0ul        //Program CF for LED2
#define    CF_LED2_220P_5P             0x0000D8ul        //Program CF for LED2
#define    CF_LED2_225P_5P             0x0000E0ul        //Program CF for LED2
#define    CF_LED2_230P_5P             0x0000E8ul        //Program CF for LED2
#define    CF_LED2_240P_5P             0x0000F0ul        //Program CF for LED2
#define    CF_LED2_245P_5P             0x0000F8ul        //Program CF for LED2

#define    STG2GAIN_LED2_0DB           0x000000ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_3DB           0x000100ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_6DB           0x000200ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_9DB           0x000300ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2_12DB          0x000400ul        //Stage 2 gain setting for LED2
#define    STG2GAIN_LED2               0x000700ul        //Stage 2 gain setting for LED2

#define    STAGE2EN_LED2               0x004000ul        //Stage 2 enable for LED2

#define    FLTRCNRSEL_500HZ            0x000000ul        //Filter corner selection
#define    FLTRCNRSEL_1000HZ           0x008000ul        //Filter corner selection

#define    AMBDAC_0uA                  0x000000ul        //Ambient DAC value
#define    AMBDAC_1uA                  0x010000ul        //Ambient DAC value
#define    AMBDAC_2uA                  0x020000ul        //Ambient DAC value
#define    AMBDAC_3uA                  0x030000ul        //Ambient DAC value
#define    AMBDAC_4uA                  0x040000ul        //Ambient DAC value
#define    AMBDAC_5uA                  0x050000ul        //Ambient DAC value
#define    AMBDAC_6uA                  0x060000ul        //Ambient DAC value
#define    AMBDAC_7uA                  0x070000ul        //Ambient DAC value
#define    AMBDAC_8uA                  0x080000ul        //Ambient DAC value
#define    AMBDAC_9uA                  0x090000ul        //Ambient DAC value
#define    AMBDAC_10uA                 0x0A0000ul        //Ambient DAC value

#define    LEDCNTRL_VAL                (0x011616ul)
#define    LED2_CURRENT                (0x0000FFul)        //Program LED current for LED2 signal
#define    LED1_CURRENT                (0x00FF00ul)        //Program LED current for LED1 signal
#define    LED_RANGE_0                 (0x000000ul)        //Full-Scale LED current range   // 150mA / 100mA / 200mA / 150mA
#define    LED_RANGE_1                 (0x010000ul)        //Full-Scale LED current range   // 75mA  / 50mA  / 100mA / 75mA
#define    LED_RANGE_2                 (0x020000ul)        //Full-Scale LED current range   // 150mA / 100mA / 200mA / 150mA
#define    LED_RANGE_3                 (0x030000ul)        //Full-Scale LED current range   // OFF   / OFF   / OFF   / OFF

#define CONTROL2_VAL                    (0x000000ul)
#define PDN_AFE_OFF                     (0x000000ul)        //AFE power-down (Powered on)
#define PDN_AFE_ON                      (0x000001ul)        //AFE power-down (Powered off)

#define PDN_RX_OFF                      (0x000000ul)        //Rx power-down (Powered on)
#define PDN_RX_ON                       (0x000002ul)        //Rx power-down (Powered off)

#define PDN_TX_OFF                      (0x000000ul)        //Tx power-down (Powered on)
#define PDN_TX_ON                       (0x000004ul)        //Tx power-down (Powered off)

#define EN_FAST_DIAG                    (0x000000ul)        //Fast diagnostics mode enable
#define EN_SLOW_DIAG                    (0x000100ul)        //Slow diagnostics mode enable

#define XTAL_ENABLE                     (0x000000ul)        //The crystal module is enabled
#define XTAL_DISABLE                    (0x000200ul)        //The crystal module is disabled

#define DIGOUT_TRISTATE_DISABLE         (0x000000ul)        //Digital tristate disabled
#define DIGOUT_TRISTATE_ENABLE          (0x000400ul)        //Digital tristate enabled

#define TXBRGMOD_H_BRIDGE               (0x000000ul)        //Tx bridge mode
#define TXBRGMOD_PUSH_PULL              (0x000800ul)        //Tx bridge mode

#define ADC_BYP_DISABLE                 (0x000000ul)        //ADC bypass mode enable
#define ADC_BYP_ENABLE                  (0x008000ul)        //ADC bypass mode enable

#define RST_CLK_ON_PD_ALM_PIN_DISABLE   (0x000000ul)        //RST CLK on PD_ALM pin disable
#define RST_CLK_ON_PD_ALM_PIN_ENABLE    (0x010000ul)        //RST CLK on PD_ALM pin enable

#define TX_REF_0                        (0x000000ul)        //Tx reference voltage - 0.75V
#define TX_REF_1                        (0x020000ul)        //Tx reference voltage - 0.5V
#define TX_REF_2                        (0x040000ul)        //Tx reference voltage - 1.0V
#define TX_REF_3                        (0x060000ul)        //Tx reference voltage - 0.75V

#define    ALARM_VAL                    (0x000000ul)
#define    ALMPINCLKEN                  (0x000080ul)        //Alarm pin clock enable (Enables CLKALMPIN)

#define PRP             ((AFECLK/PRF)-1)                // for 100HZ - 39999

#define DELTA           (((PRP+1)*DUTYCYCLE)/100)       // for 100HZ - 8000
#define CONV_DELTA      ((PRP+1)/4)                     // for 100HZ - 10000
#define ADCRESET_DELAY  5

#define LED2STC_VAL     ((((PRP+1)*3)/4)+80)            // for 100HZ - 30080

#define LED2ENDC_VAL    (LED2STC_VAL-80+DELTA-2)        // for 100HZ - 37998

#define LED2LEDSTC_VAL  (LED2STC_VAL-80)                // for 100HZ - 30000

#define LED2LEDENDC_VAL (LED2ENDC_VAL+1)                // for 100HZ - 37999

#define ALED2STC_VAL    80                              // for 100HZ - 80

#define ALED2ENDC_VAL   (DELTA-2)                       // for 100HZ - 7998

#define LED1STC_VAL     (((PRP+1)/4)+80)                // for 100HZ - 10080

#define LED1ENDC_VAL    (LED1STC_VAL-80+DELTA-2)        // for 100HZ - 17998

#define LED1LEDSTC_VAL  (LED1STC_VAL-80)                // for 100HZ - 10000

#define LED1LEDENDC_VAL (LED1ENDC_VAL+1)                // for 100HZ - 17999

#define ALED1STC_VAL    (((PRP+1)/2)+80)                // for 100HZ - 20080

#define ALED1ENDC_VAL   (ALED1STC_VAL-80+DELTA-2)       // for 100HZ - 27998

#define LED2CONVST_VAL  (ADCRESET_DELAY+2)              // for 100HZ - 6

#define LED2CONVEND_VAL (LED2CONVST_VAL-(ADCRESET_DELAY+1)+CONV_DELTA-1)        // for 100HZ - 9999

#define ALED2CONVST_VAL (CONV_DELTA+7)                  // for 100HZ - 10006

#define ALED2CONVEND_VAL (ALED2CONVST_VAL-(ADCRESET_DELAY+1)+CONV_DELTA-1)      // for 100HZ - 19999

#define LED1CONVST_VAL  ((CONV_DELTA*2)+(ADCRESET_DELAY+2))     // for 100HZ - 20006

#define LED1CONVEND_VAL (LED1CONVST_VAL-(ADCRESET_DELAY+1)+CONV_DELTA-1)        // for 100HZ - 29999

#define ALED1CONVST_VAL ((CONV_DELTA*3)+(ADCRESET_DELAY+2))     // for 100HZ - 30006

#define ALED1CONVEND_VAL (ALED1CONVST_VAL-(ADCRESET_DELAY+1)+CONV_DELTA-1)      // for 100HZ - 39999

#define ADCRSTSTCT0_VAL 0       // for 100HZ - 0

#define ADCRSTENDCT0_VAL (ADCRSTSTCT0_VAL+ADCRESET_DELAY)       // for 100HZ - 5

#define ADCRSTSTCT1_VAL CONV_DELTA      // for 100HZ - 10000

#define ADCRSTENDCT1_VAL (ADCRSTSTCT1_VAL+ADCRESET_DELAY)       // for 100HZ - 10005

#define ADCRSTSTCT2_VAL (CONV_DELTA*2)  // for 100HZ - 20000

#define ADCRSTENDCT2_VAL (ADCRSTSTCT2_VAL+ADCRESET_DELAY)       // for 100HZ - 20005

#define ADCRSTSTCT3_VAL (CONV_DELTA*3)  // for 100HZ - 30000

#define ADCRSTENDCT3_VAL (ADCRSTSTCT3_VAL+ADCRESET_DELAY)       // for 100HZ - 30005

#endif /* PERIPH_DRIVERS_SPO2_DRIVER_AFE4400_DEFS_H_ */
