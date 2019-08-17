/**
 ******************************************************************************
 * @file    user.c
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    25 June 2017
 * @brief   General Module
 ******************************************************************************
 */
#include "user.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

/*************************************************************************************************/
uint32_t I2C0_GetFreq(void)
{
	return CLOCK_GetFreq(I2C0_CLK_SRC);
}

/*************************************************************************************************/
uint32_t I2C1_GetFreq(void)
{
	return CLOCK_GetFreq(I2C1_CLK_SRC);
}

/*************************************************************************************************/
uint32_t DSPI2_GetFreq(void)
{
    return CLOCK_GetBusClkFreq();
}


/*************************************************************************************************/
uint32_t DSPI1_GetFreq(void)
{
	return CLOCK_GetBusClkFreq();
}

/*************************************************************************************************/
uint32_t DSPI0_GetFreq(void)
{
	return CLOCK_GetBusClkFreq();
}

/*************************************************************************************************/
void vUser_InitDMA(void)
{
    edma_config_t edmaConfig =
            { 0 };

    /* DMA Mux init and EDMA init */
    EDMA_GetDefaultConfig(&edmaConfig);

    EDMA_Init(DMA0, &edmaConfig);
    DMAMUX_Init(DMAMUX);
}

/*************************************************************************************************/
PORT_Type* xUser_GetPortByGPIO(uint32_t base)
{
    switch(base)
    {
        case GPIOA_BASE:
            return PORTA;
        case GPIOB_BASE:
            return PORTB;
        case GPIOC_BASE:
            return PORTC;
        case GPIOD_BASE:
            return PORTD;
        case GPIOE_BASE:
            return PORTE;
    }
    return NULL;
}
/*************************************************************************************************/
