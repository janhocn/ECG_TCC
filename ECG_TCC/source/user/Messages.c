/**
 ******************************************************************************
 * @file    Messages.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    9 November 2017
 * @brief   All messages for the projects are placed here in English and Portuguese
 ******************************************************************************
 */
#include "Messages.h"

#include "MCU_Flash.h"
#include "fsl_flash.h"
#include "fsl_device_registers.h"

/** @var Indicates the actual language */
static eLanguage _LanguageSet = _E_PORTUGUESE;

/** @var Message Pack */
const stMessages _Messages =
        {
                .msgHome[_E_ENGLISH]=
                {
                        "Measure ECG",
                        "Measure SpO2",
                        "Settings",
                        "About",
                        "Help",
                        "Debug"

                },
                .msgHome[_E_PORTUGUESE]=
                {
                        "Medir ECG",
                        "Medir SpO2",
                        "Configurações",
                        "Sobre",
                        "Ajuda",
                        "Debug"
                },
                .msgSettings[_E_ENGLISH]=
                {
                        "Please press the touch\nscreen to continue...",
                        "Touch Screen Calibration",
                        "Congratulation, your\ntouch screen has been\ncalibrated.",
                        "Please touch the screen\nat the center of the ring.",
                        "Brightness",
                        "Time",
                        "Date",
                        "Battery Status",
                        "Touch Calibration",
                        "Language",
                        "Change the backlight intensity",
                        "Adjust the clock time",
                        "Adjust the date",
                        "See Battery Status",
                        "Calibrate the touch Screen",
                        "Change the Language",
                        "General",
                        "Input",
                        "OK",
                        "Cancel",
                        "Save",
                        "Save and Reset",
                        "Informações da Bateria",
                        "\nTensão: %d mV\n",
                        "\nCorrente: %d mA\n",
                        "\nPotência: %d mW\n",
                        "\nCapacidade: %d mAh\n",
                        "\nTemperatura: %d °C\n",
                        "\nEstado de carga: %d %%\n",
                        "\nEstado de Saúde: %d %%\n"
                },
                .msgSettings[_E_PORTUGUESE]=
                {
                        "Por favor pressione\na tela para continuar...",
                        "Calibração da Tela de Toque",
                        "Parabéns, sua\ntela de toque está\ncalibrada",
                        "Por favor toque a tela\nno centro do anel.",
                        "Brilho",
                        "Hora",
                        "Data",
                        "Estado da Bateria",
                        "Calibração da tela de toque",
                        "Linguagem",
                        "Mude a intensidade de brilho da tela",
                        "Ajuste o relógio",
                        "Ajuste a data",
                        "Visualize o estado da bateria",
                        "Calibre a tela de toque",
                        "Mude a Língua",
                        "Geral",
                        "Entrada",
                        "OK",
                        "Cancelar",
                        "Salvar",
                        "Salvar e Reiniciar",
                        "Battery Information",
                        "\nVoltage: %d mV\n",
                        "\nCurrent: %d mA\n",
                        "\nPower: %d mW\n",
                        "\nCapacity: %d mAh\n",
                        "\nTemperature: %d °C\n",
                        "\nState of charge: %d %%\n",
                        "\nState of health: %d %%\n"
                },
                .msgDisplay[_E_ENGLISH]=
                {
                        "TCC - Portable Microcontrolled ECG",
                        "Authors:\n"
                        "Diego Reis Caldas\n"
                        "Gabriel Giorgetto Dalho\n"
                        "Contact: diegoreis.caldas@gmail.com\n"
                        "              gabrielggd2@gmail.com\n"
                        "Advisor: Professor Phd Paulo Batista Lopes\n"
                        "Firmware Version: 1.0A\n"
                        "Processor: MK64fn1M0VLL12\n"
                        "Clock Frequency: 120MHz\n"
                        "Operating System: FreeRTOS",
                        "Debug Information",
                        "GUI Free Bytes: %06lu\nGUI Used Bytes: %06lu\nTask Number: %06lu"
                },
                .msgDisplay[_E_PORTUGUESE]=
                {
                        "TCC - ECG Portátil Microcontrolado",
                        "Autores:\n"
                        "Diego Reis Caldas\n"
                        "Gabriel Giorgetto Dalho\n"
                        "Contato: diegoreis.caldas@gmail.com\n"
                        "              gabrielggd2@gmail.com\n"
                        "Orientador: Professor Dr. Paulo Batista Lopes\n"
                        "Versão de Firmware: 1.0A\n"
                        "Processador: MK64fn1M0VLL12\n"
                        "Frequência de Clock: 120MHz\n"
                        "Sistema Operacional: FreeRTOS",
                        "Informa��es de Debug",
                        "GUI Bytes Livres: %06lu\nGUI Bytes Utilizados: %06lu\nQntd Tarefas: %06lu"
                },
                .msgInitialization[_E_ENGLISH]=
                {
                        "TCC - Portable Microcontrolled ECG",
                        "\nInitializing SD Card and File System . . .\n",
						"\nPlease Insert the SD Card . . .\n",
                        "\nFile System and SD Card initialized with success . . .\n",
                        "\nFile System and SD Card initialized with failure . . .\n",
                        "\nConfiguring Home Button and Clock . . .\n",
                        "\nConfiguring Display and touch screen . . .\n",
                        "\nConfiguring ECG acquisition routines . . .\n",
                        "\nConfiguring Pulse Oxymeter routines and hardware . . .\n",
                        "\nInitialization Done!!! . . .\n"
                },
                .msgInitialization[_E_PORTUGUESE]=
                {
                        "TCC - ECG Portátil Microcontrolado",
                        "\nIniciando SD Card e Sistema de Arquivos . . .\n",
						"\nPor Favor insira o cartão SD . . .\n",
                        "\nSistema de arquivos e Cartão SD iniciados com sucesso . . .\n",
                        "\nSistema de arquivos e Cartão SD iniciados com falha . . .\n",
                        "\nConfigurando Botão Home e Relógio . . .\n",
                        "\nConfigurando Display e touchscreen . . .\n",
                        "\nConfigurando rotinas de aquisição do ECG . . .\n",
                        "\nConfigurando hardware e rotinas do oximetro de pulso . . .\n",
                        "\nInicialização Terminada!!! . . .\n"
                },
                .msgECGGrpah[_E_ENGLISH]=
                {
                        "Start Log",
                        "Time [ms]",
                        "ECG Log Finished",
                },
                .msgECGGrpah[_E_PORTUGUESE]=
                {
                        "Iniciar Log",
                        "Tempo [ms]",
                        "Log ECG Terminado",
                },
                .msgSPO2Graph[_E_ENGLISH]=
                {
                        "Start Log",
                        "Time [ms]",
                        "SPO2 Log Finished",
                },
                .msgSPO2Graph[_E_PORTUGUESE]=
                {
                        "Iniciar Log",
                        "Tempo [ms]",
                        "Log SPO2 Terminado",
                },
                .msgHelpText[_E_ENGLISH]=
                {
                        "Help",
                        "To start measuring the eletrocardiograph, go to\n"
                        "the start menu and select |Measure ECG|.\n"
                        "Position your fingers on the electrodes, without\n"
                        "pressioning too much, and wait until the trace\n"
                        "become stable.\n"
                        "To start measuring the photoplethysmogram go to\n"
                        "the start menu and select |Measure SpO2|.\n"
                        "Position your finger on the LED window, and wait\n"
                        "until the trace become stable.\n"
                        "For both measures, it is possible to store a data log.\n"
                        "On the measure window select the checkbox |Start Log|\n"
                        "and wait 35s, or select it again to stop.",
                },
                .msgHelpText[_E_PORTUGUESE]=
                {
                        "Ajuda",
                        "Para realizar o eletrocardiograma, vá até\n"
                        "o menu inicial e selecione |Medir ECG|.\n"
                        "Posicione os dedos nos eletrodos,\n"
                        "sem apertar, e espere o sinal ficar estável.\n"
                        "Para realizar a fotopletismografia vá até\n"
                        "o menu inicial e selecione |Medir SpO2|.\n"
                        "Posicione os dedos sobre a janela\n"
                        "com LEDs, e espere o sinal ficar estável.\n"
                        "Para ambas as medidas, é possível gravar\n"
                        "um Log de dados.\n"
                        "Na tela de medição selecione a caixa |Iniciar Log|\n"
                        "e aguarde 35s, ou selecione denovo para parar.",
                },
        };

/*************************************************************************************************/
struct _Initialization * xMessage_getMessageInitialization(void)
{
        return (struct _Initialization *)(&_Messages.msgInitialization[_LanguageSet]);
}

/*************************************************************************************************/
struct _Display * xMessage_getMessageDisplay(void)
{
        return (struct _Display *)(&_Messages.msgDisplay[_LanguageSet]);
}

/*************************************************************************************************/
struct _HelpBox * xMessage_getMessageHelp(void)
{
        return (struct _HelpBox *)(&_Messages.msgHelpText[_LanguageSet]);
}

/*************************************************************************************************/
struct _Settings * xMessage_getMessageSettings(void)
{
        return (struct _Settings *)(&_Messages.msgSettings[_LanguageSet]);
}

/*************************************************************************************************/
struct _Home * xMessage_getMessageHome(void)
{
        return (struct _Home *)(&_Messages.msgHome[_LanguageSet]);
}

/*************************************************************************************************/
struct _ECGGrpah * xMessage_getECGGraph(void)
{
        return (struct _ECGGrpah *)(&_Messages.msgECGGrpah[_LanguageSet]);
}

/*************************************************************************************************/
struct _SPO2Grpah * xMessage_getSPO2Graph(void)
{
        return (struct _SPO2Grpah *)(&_Messages.msgSPO2Graph[_LanguageSet]);
}

/*************************************************************************************************/
void vMessages_Init(void)
{
	if(stMCU_Flash_retrieveKnownData(_FLASH_USER_LANGUAGE,&_LanguageSet) == kStatus_Success)
	{
	    __CHECK_LANGUAGE(_LanguageSet);
	}
	else
	{
	    _LanguageSet = _E_PORTUGUESE;
	}
}

/*************************************************************************************************/
void vMessages_setLanguage(eLanguage lang)
{
    _LanguageSet = lang;
    __CHECK_LANGUAGE(_LanguageSet);
}

/*************************************************************************************************/
eLanguage * xMessages_getLanguageAddr(void)
{
    return &_LanguageSet;
}
/*************************************************************************************************/
