/*
 * TSC2007_Defs.h
 *
 *  Created on: Mar 20, 2018
 *      Author: TSI
 */

#ifndef PERIPH_DRIVERS_TOUCHSCREEN_DRIVER_TSC2007_DEFS_H_
#define PERIPH_DRIVERS_TOUCHSCREEN_DRIVER_TSC2007_DEFS_H_

#define TSC2007_MEASURE_TEMP0		(0x0 << 4)
#define TSC2007_MEASURE_AUX			(0x2 << 4)
#define TSC2007_MEASURE_TEMP1		(0x4 << 4)
#define TSC2007_ACTIVATE_XN			(0x8 << 4)
#define TSC2007_ACTIVATE_YN			(0x9 << 4)
#define TSC2007_ACTIVATE_YP_XN		(0xa << 4)
#define TSC2007_SETUP				(0xb << 4)
#define TSC2007_MEASURE_X			(0xc << 4)
#define TSC2007_MEASURE_Y			(0xd << 4)
#define TSC2007_MEASURE_Z1			(0xe << 4)
#define TSC2007_MEASURE_Z2			(0xf << 4)

#define TSC2007_POWER_OFF_IRQ_EN	(0x0 << 2)
#define TSC2007_ADC_ON_IRQ_DIS0		(0x1 << 2)
#define TSC2007_ADC_OFF_IRQ_EN		(0x2 << 2)
#define TSC2007_ADC_ON_IRQ_DIS1		(0x3 << 2)

#define TSC2007_12BIT				(0x0 << 1)
#define TSC2007_8BIT				(0x1 << 1)

#define	MAX_12BIT					((1 << 12) - 1)

#define ADC_ON_12BIT	            (TSC2007_12BIT | TSC2007_ADC_ON_IRQ_DIS0)

#define READ_Y		                (ADC_ON_12BIT | TSC2007_MEASURE_Y)
#define READ_Z1		                (ADC_ON_12BIT | TSC2007_MEASURE_Z1)
#define READ_Z2		                (ADC_ON_12BIT | TSC2007_MEASURE_Z2)
#define READ_X		                (ADC_ON_12BIT | TSC2007_MEASURE_X)
#define PWRDOWN		                (TSC2007_12BIT | TSC2007_POWER_OFF_IRQ_EN)

#define PENIRQDIS                   (TSC2007_12BIT | TSC2007_ADC_ON_IRQ_DIS0)

#endif /* PERIPH_DRIVERS_TOUCHSCREEN_DRIVER_TSC2007_DEFS_H_ */
