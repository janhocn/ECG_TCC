/**
 ******************************************************************************
 * @file    spo2AcqTask.h
 * @author  Diego Reis Caldas - diegoreis.caldas@gmail.com
 * @version V0.0.0
 * @date    01 March 2017
 * @brief   Module to manage the spo2 acquisition tasks
 ******************************************************************************
 */
#ifndef TASKS_SPO2ACQTASK_H_
#define TASKS_SPO2ACQTASK_H_

#define SAMPLES_SPO2_PER_BLOCK 10

/*************************************************************************************************/
/**
 * @brief Initialize the tasks
 * @return void.
 */
void vSPO2AcqTaskInit(void);

/*************************************************************************************************/
/**
 * @brief Pin ADC_READY IRQ Handlers
 * @param status true if the interrupt is for this handler (this pin)
 * @return void.
 */
void vSPO2Task_IRQ_DataReady(uint32_t status);

/*************************************************************************************************/
/**
 * @brief Resume all related Tasks and interrupts
 * @return void
 */
void vSPO2AcqTaskResume(void);

/*************************************************************************************************/
/**
 * @brief Suspend all related Tasks and interrupts
 * @return void
 */
void vSPO2AcqTaskSuspend(void);

#endif /* TASKS_SPO2ACQTASK_H_ */
