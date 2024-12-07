/*
 * bcc_wait.h
 *
 *  Created on: Nov 12, 2024
 *      Author: LENOVO
 */

#ifndef BCC_WAIT_H_
#define BCC_WAIT_H_
#include <stdint.h>

//uint32_t BCC_MCU_GetSystemClockFreq(void);

/*!
 * @brief Waits for specified amount of seconds.
 *
 * @param delay Number of seconds to wait.
 */
void BCC_MCU_WaitSec(uint16_t delay);

/*!
 * @brief Waits for specified amount of milliseconds.
 *
 * @param delay Number of milliseconds to wait.
 */
void BCC_MCU_WaitMs(uint16_t delay);

/*!
 * @brief Waits for specified amount of microseconds.
 *
 * @param delay Number of microseconds to wait.
 */
void BCC_MCU_WaitUs(uint32_t delay);
/*! @} */

#endif /* BCC_WAIT_H_ */
