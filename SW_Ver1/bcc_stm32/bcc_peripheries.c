/*
 * bcc_peripheries.c
 *
 *  Created on: Nov 12, 2024
 *      Author: LENOVO
 */
#include "bcc_communication.h"
#include "bcc_peripheries.h"
#include "common.h"
#include "stm32f103xb.h"
#include "main.h"
#include "stm32f1xx_hal.h" // Thư viện HAL cho STM32

#define BMS_RST_Pin GPIO_PIN_1
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define BMS_RST_GPIO_Port GPIOB

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
// Trạng thái timeout
volatile bool s_timeoutExpired = false;
/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_StartTimeout
 * Description   : Starts a non-blocking timeout mechanism. After expiration of
 *                 the time passed as a parameter, function
 *                 BCC_MCU_TimeoutExpired should signalize an expired timeout.
 *
 *END**************************************************************************/
bcc_status_t BCC_MCU_StartTimeout(const uint32_t timeoutUs)
{
//    // Dừng Timer trước khi cấu hình lại
//        HAL_TIM_Base_Stop_IT(&htim3);
//
//        // Cấu hình chu kỳ (Period) của Timer
//        uint32_t timerClockFreq = HAL_RCC_GetPCLK1Freq(); // Tần số của bus APB1 (liên quan đến TIM3)
//        uint32_t prescaler = (timerClockFreq / 1000000) - 1; // Đặt prescaler để Timer đếm theo micro giây
//        uint32_t period = timeoutUs - 1; // Số lần đếm tương ứng với timeout
//
//        if (period > 0xFFFF) // Kiểm tra nếu vượt giới hạn bộ đếm 16-bit
//        {
//            return BCC_STATUS_TIMEOUT_START; // Lỗi: thời gian timeout quá lớn
//        }
//
//        // Cập nhật cấu hình Timer
//        htim3.Init.Prescaler = prescaler;
//        htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//        htim3.Init.Period = period;
//        htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

//        if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
//        {
//            return 1; // Lỗi khi khởi tạo Timer
//        }

        // Đặt trạng thái timeout chưa xảy ra
//        s_timeoutExpired = false;
//
//        // Bắt đầu Timer với ngắt
//        HAL_TIM_Base_Start_IT(&htim3);
//
//        return 0; // Thành công
    return BCC_STATUS_SUCCESS;
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM3) // Kiểm tra nếu ngắt đến từ TIM3
//    {
//        s_timeoutExpired = true; // Đặt trạng thái timeout
//        HAL_TIM_Base_Stop_IT(&htim3); // Dừng Timer sau khi hết hạn
//    }
//}



/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_WriteRstPin
 * Description   : Writes logic 0 or 1 to the RST pin.
 *
 *END**************************************************************************/
void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value)
{
	HAL_GPIO_WritePin(BMS_RST_GPIO_Port, BMS_RST_Pin, value);
}



/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_WriteCsbPin
 * Description   : Writes logic 0 or 1 to the CSB_TX pin.
 *
 *END**************************************************************************/
void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, value);
}



/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_MCU_TransferSpi
 * Description   : This function sends and receives data via SPI bus. Intended
 *                 for SPI mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[],
    uint8_t rxBuf[])
{
	char txmess[] = {0xF1};
	char rxmess[] = {0xF2};
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0);
//	HAL_SPI_TransmitReceive(&hspi1, &txBuf[0], &rxBuf[0], 1, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, &txBuf[1], &rxBuf[1], 1, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, &txBuf[2], &rxBuf[2], 1, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, &txBuf[3], &rxBuf[3], 1, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, &txBuf[4], &rxBuf[4], 1, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, &txBuf[5], &rxBuf[5], 1, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 6, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, 1);
//	HAL_UART_Transmit(&huart1, txmess, sizeof(txmess),HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart1, txBuf, 6,HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart1, rxmess, sizeof(rxmess),HAL_MAX_DELAY);
//	HAL_UART_Transmit(&huart1, rxBuf, 6,HAL_MAX_DELAY);
    return BCC_STATUS_SUCCESS;

}
