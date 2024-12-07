/*
 * bcc_wait.c
 *
 *  Created on: Nov 12, 2024
 *      Author: Anh Huy
 */


#include "bcc_wait.h"
#include "stm32f1xx_hal.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_GetSystemClockFreq
 * Description   : Returns SCG system clock frequency.
 *
 *END**************************************************************************/
//uint32_t BCC_MCU_GetSystemClockFreq(void)
//{
//    uint32_t freq;
//    CLOCK_SYS_GetFreq(CORE_CLOCK, &freq);
//    return freq;
//}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitSec
 * Description   : Waits for specified amount of seconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitSec(uint16_t delay)
{
    for (; delay > 0U; delay--) {
        HAL_Delay(1000U);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitMs(uint16_t delay)
{
	HAL_Delay(delay);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitUs
 * Description   : Waits for specified amount of microseconds.
 *
 *END**************************************************************************/


void BCC_MCU_WaitUs(uint32_t delay)
{
//	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(&htim1) < delay);  // wait for the counter to reach the us input in the parameter
	HAL_Delay(delay);
}
