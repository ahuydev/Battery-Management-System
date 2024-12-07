/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bcc.h"
#include "common.h"
#include <stdio.h>
#include "monitoring.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Structure containing a register name and its address. */
typedef struct
{
    const uint8_t address;
    const uint16_t defaultVal;
    const uint16_t value;
} bcc_init_reg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Number of BCC configuration registers set during the initialization. */
#define MC33771C_INIT_CONF_REG_CNT     59U
#define MC33772C_INIT_CONF_REG_CNT     43U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


/* Initial configuration of Battery Cell Controller devices.
 * INIT: Initialized by the BCC driver.
 * SYS_CFG_GLOBAL, SYS_DIAG and TPL_CFG: Kept POR values. */


static const bcc_init_reg_t s_initRegsMc33772c[MC33772C_INIT_CONF_REG_CNT] = {
    {MC33772C_GPIO_CFG1_OFFSET, MC33772C_GPIO_CFG1_POR_VAL, MC33772C_GPIO_CFG1_VALUE},
    {MC33772C_GPIO_CFG2_OFFSET, MC33772C_GPIO_CFG2_POR_VAL, MC33772C_GPIO_CFG2_VALUE},
    {MC33772C_TH_ALL_CT_OFFSET, MC33772C_TH_ALL_CT_POR_VAL, MC33772C_TH_ALL_CT_VALUE},
    {MC33772C_TH_CT6_OFFSET, MC33772C_TH_CT6_POR_VAL, MC33772C_TH_CTX_VALUE},
    {MC33772C_TH_CT5_OFFSET, MC33772C_TH_CT5_POR_VAL, MC33772C_TH_CTX_VALUE},
    {MC33772C_TH_CT4_OFFSET, MC33772C_TH_CT4_POR_VAL, MC33772C_TH_CTX_VALUE},
    {MC33772C_TH_CT3_OFFSET, MC33772C_TH_CT3_POR_VAL, MC33772C_TH_CTX_VALUE},
    {MC33772C_TH_CT2_OFFSET, MC33772C_TH_CT2_POR_VAL, MC33772C_TH_CTX_VALUE},
    {MC33772C_TH_CT1_OFFSET, MC33772C_TH_CT1_POR_VAL, MC33772C_TH_CTX_VALUE},
    {MC33772C_TH_AN6_OT_OFFSET, MC33772C_TH_AN6_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN5_OT_OFFSET, MC33772C_TH_AN5_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN4_OT_OFFSET, MC33772C_TH_AN4_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN3_OT_OFFSET, MC33772C_TH_AN3_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN2_OT_OFFSET, MC33772C_TH_AN2_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN1_OT_OFFSET, MC33772C_TH_AN1_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN0_OT_OFFSET, MC33772C_TH_AN0_OT_POR_VAL, MC33772C_TH_ANX_OT_VALUE},
    {MC33772C_TH_AN6_UT_OFFSET, MC33772C_TH_AN6_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN5_UT_OFFSET, MC33772C_TH_AN5_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN4_UT_OFFSET, MC33772C_TH_AN4_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN3_UT_OFFSET, MC33772C_TH_AN3_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN2_UT_OFFSET, MC33772C_TH_AN2_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN1_UT_OFFSET, MC33772C_TH_AN1_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_AN0_UT_OFFSET, MC33772C_TH_AN0_UT_POR_VAL, MC33772C_TH_ANX_UT_VALUE},
    {MC33772C_TH_ISENSE_OC_OFFSET, MC33772C_TH_ISENSE_OC_POR_VAL, MC33772C_TH_ISENSE_OC_VALUE},
    {MC33772C_TH_COULOMB_CNT_MSB_OFFSET, MC33772C_TH_COULOMB_CNT_MSB_POR_VAL, MC33772C_TH_COULOMB_CNT_MSB_VALUE},
    {MC33772C_TH_COULOMB_CNT_LSB_OFFSET, MC33772C_TH_COULOMB_CNT_LSB_POR_VAL, MC33772C_TH_COULOMB_CNT_LSB_VALUE},
    {MC33772C_CB1_CFG_OFFSET, MC33772C_CB1_CFG_POR_VAL, MC33772C_CBX_CFG_VALUE},
    {MC33772C_CB2_CFG_OFFSET, MC33772C_CB2_CFG_POR_VAL, MC33772C_CBX_CFG_VALUE},
    {MC33772C_CB3_CFG_OFFSET, MC33772C_CB3_CFG_POR_VAL, MC33772C_CBX_CFG_VALUE},
    {MC33772C_CB4_CFG_OFFSET, MC33772C_CB4_CFG_POR_VAL, MC33772C_CBX_CFG_VALUE},
    {MC33772C_CB5_CFG_OFFSET, MC33772C_CB5_CFG_POR_VAL, MC33772C_CBX_CFG_VALUE},
    {MC33772C_CB6_CFG_OFFSET, MC33772C_CB6_CFG_POR_VAL, MC33772C_CBX_CFG_VALUE},
    {MC33772C_OV_UV_EN_OFFSET, MC33772C_OV_UV_EN_POR_VAL, MC33772C_OV_UV_EN_VALUE},
    {MC33772C_SYS_CFG1_OFFSET, MC33772C_SYS_CFG1_POR_VAL, MC33772C_SYS_CFG1_VALUE},
    {MC33772C_SYS_CFG2_OFFSET, MC33772C_SYS_CFG2_POR_VAL, MC33772C_SYS_CFG2_VALUE},
    {MC33772C_ADC_CFG_OFFSET, MC33772C_ADC_CFG_POR_VAL, MC33772C_ADC_CFG_VALUE},
    {MC33772C_ADC2_OFFSET_COMP_OFFSET, MC33772C_ADC2_OFFSET_COMP_POR_VAL, MC33772C_ADC2_OFFSET_COMP_VALUE},
    {MC33772C_FAULT_MASK1_OFFSET, MC33772C_FAULT_MASK1_POR_VAL, MC33772C_FAULT_MASK1_VALUE},
    {MC33772C_FAULT_MASK2_OFFSET, MC33772C_FAULT_MASK2_POR_VAL, MC33772C_FAULT_MASK2_VALUE},
    {MC33772C_FAULT_MASK3_OFFSET, MC33772C_FAULT_MASK3_POR_VAL, MC33772C_FAULT_MASK3_VALUE},
    {MC33772C_WAKEUP_MASK1_OFFSET, MC33772C_WAKEUP_MASK1_POR_VAL, MC33772C_WAKEUP_MASK1_VALUE},
    {MC33772C_WAKEUP_MASK2_OFFSET, MC33772C_WAKEUP_MASK2_POR_VAL, MC33772C_WAKEUP_MASK2_VALUE},
    {MC33772C_WAKEUP_MASK3_OFFSET, MC33772C_WAKEUP_MASK3_POR_VAL, MC33772C_WAKEUP_MASK3_VALUE},
};  // Genarate 43 Element = 43 Config Register of IC CellController

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static bcc_status_t initRegisters();
static bcc_status_t clearFaultRegs();
static void initDemo(bcc_status_t *bccError);
static bcc_status_t startApp(void);
void startDischarge_Charge();

bcc_status_t MeasuarenceApp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000);
    return len;
}



bcc_data_t g_bccData;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //  startDischarge_Charge();

    bcc_status_t bccError;

    initDemo(&bccError);

        if (bccError != BCC_STATUS_SUCCESS)
        {
            printf("An error occurred during BCC initialization: (0x%04x)\r\n", bccError);
            HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin,0);
        }
        else
        {
            printf("------------- BEGIN ---------------\r\n");
            printf("Success\n");
            if ((bccError = startApp()) != BCC_STATUS_SUCCESS)
            {
                printf("An error occurred (0x%04x)\r\n)", bccError);

            }

            bccError = BCC_Sleep(&g_bccData.drvConfig);
            if (bccError != BCC_STATUS_SUCCESS)
            {
                printf("SLEEP (0x%04x)\r\n)", bccError);

            }else{
          	  printf("SLEEPINGGGGG)");
            }

            printf("-------------- END ----------------\r\n");
        }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MeasuarenceApp();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOS_DISCHARGE_Pin|MOS_CHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_RST_GPIO_Port, BMS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PC13_Pin */
  GPIO_InitStruct.Pin = LED_PC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED_PC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOS_DISCHARGE_Pin MOS_CHARGE_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = MOS_DISCHARGE_Pin|MOS_CHARGE_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_RST_Pin */
  GPIO_InitStruct.Pin = BMS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Measure voltage of stack, cell 1, cell 2, cell 3, isense.
bcc_status_t MeasuarenceApp(void)
{
	uint8_t cid;
	bcc_status_t error;
	uint32_t valueVoltage;


	error = BCC_Meas_StartAndWait(&g_bccData.drvConfig, BCC_CID_DEV1, BCC_AVG_1);
	if (error != BCC_STATUS_SUCCESS)
	{
		return error;
	}

	for (cid = BCC_CID_DEV1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
	{
		sendNops();

		printf("------------- BEGIN ---------------\r\n");
		//	doMeasurements(1);
		error = BCC_Meas_GetStackVoltage(&g_bccData.drvConfig, BCC_CID_DEV1, &valueVoltage);
		if (error != BCC_STATUS_SUCCESS)
		{
			printf("Error (0x%04x)\r\n)", error);
		}
		else
		{
			printf("Stack: %d mV \r\n",valueVoltage/1000U);
		}

		sendNops();

		//	 get cell 1 voltage
		error = BCC_Meas_GetCellVoltage(&g_bccData.drvConfig, BCC_CID_DEV1, 0, &valueVoltage);
		if (error != BCC_STATUS_SUCCESS)
		{
			printf("Error (0x%04x)\r\n)", error);
		}
		else
		{
			printf("Cell 1: %d mV \r\n",valueVoltage/1000U);
		}

		sendNops();

		//	  get cell 1 voltage
		error = BCC_Meas_GetCellVoltage(&g_bccData.drvConfig, BCC_CID_DEV1, 1, &valueVoltage);
		if (error != BCC_STATUS_SUCCESS)
		{
			printf("Error (0x%04x)\r\n)", error);
		}else
		{
			printf("Cell 2: %d mV \r\n",valueVoltage/1000U);
		}

		sendNops();

		//	  get cell 1 voltage
		error = BCC_Meas_GetCellVoltage(&g_bccData.drvConfig, BCC_CID_DEV1, 5, &valueVoltage);
		if (error != BCC_STATUS_SUCCESS)
		{
			printf("Error (0x%04x)\r\n)", error);
		}else{
			printf("Cell 3: %d mV \r\n",valueVoltage/1000U);
		}

		sendNops();

		//	  get Isense voltage
		error = BCC_Meas_GetIsenseVoltage(&g_bccData.drvConfig, BCC_CID_DEV1, &valueVoltage);
		if (error != BCC_STATUS_SUCCESS)
		 {
			printf("Error (0x%04x)\r\n)", error);
		}else
		{
			printf("ISENSE: %d  uV \r\n", valueVoltage);
			printf("ISENSE: %d  mA \r\n", abs(valueVoltage)/10U);
		}

		sendNops();
		//		get temperature value
		if ((error = doGetTemp(cid)) != BCC_STATUS_SUCCESS)
		{
		    return error;
		}

		printf("-------------- END ----------------\r\n");
	}
}

static bcc_status_t initRegisters()
{
    uint8_t cid, i;
    bcc_status_t status;

    for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
    {
          for (i = 0; i < MC33772C_INIT_CONF_REG_CNT; i++)
          {
              if (s_initRegsMc33772c[i].value != s_initRegsMc33772c[i].defaultVal)
              {
                  status = BCC_Reg_Write(&g_bccData.drvConfig, (bcc_cid_t)cid,
                          s_initRegsMc33772c[i].address, s_initRegsMc33772c[i].value);
                  if (status != BCC_STATUS_SUCCESS)
                  {
                      return status;
                  }
              }
          }
  }
    return BCC_STATUS_SUCCESS;
}




static bcc_status_t clearFaultRegs()
{
    uint8_t cid;
    bcc_status_t status;

    for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
    {
        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_OV);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_UV);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_COMM);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT1);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT2);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }

        status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT3);
        if (status != BCC_STATUS_SUCCESS)
        {
            return status;
        }
    }

    return BCC_STATUS_SUCCESS;
}



static void initDemo(bcc_status_t *bccError)
{
    ntc_config_t ntcConfig;
    /* Initialize BCC driver configuration structure (g_bccData.drvConfig). */

    g_bccData.drvConfig.drvInstance = 0U;
    g_bccData.drvConfig.commMode = BCC_MODE_SPI;
    g_bccData.drvConfig.devicesCnt = 1U;
    g_bccData.drvConfig.device[0] = BCC_DEVICE_MC33772C;
    g_bccData.drvConfig.cellCnt[0] = 3U;


    /* Precalculate NTC look up table for fast temperature measurement. */
    ntcConfig.rntc = 6800U;               /* NTC pull-up 6.8kOhm */
    ntcConfig.refTemp = 25U;              /* NTC resistance 10kOhm at 25 degC */
    ntcConfig.refRes = 10000U;            /* NTC resistance 10kOhm at 25 degC */
    ntcConfig.beta = 3435U;
    fillNtcTable(&ntcConfig);

    /* Initialize BCC device. */
    *bccError = BCC_Init(&g_bccData.drvConfig);
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }

    /* Initialize BCC device registers. */
    *bccError = initRegisters();
    if (*bccError != BCC_STATUS_SUCCESS)
    {
        return;
    }

    /* Clear fault registers. */
    *bccError = clearFaultRegs();
}


static bcc_status_t startApp(void)
{
    uint8_t cid;
    bcc_status_t error;

    for (cid = BCC_CID_DEV1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
    {
        /* Send NOP command to all nodes in order to prevent communication timeout. */
        sendNops();

        /* Print values of the configurable registers. */
        if ((error = printInitialSettings(cid)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        /* Send NOP command to all nodes in order to prevent communication timeout. */
        sendNops();

        /* Do a measurement and print the measured values. */
        if ((error = doMeasurements(cid)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }

        /* Send NOP command to all nodes in order to prevent communication timeout. */
        sendNops();

        /* Print content of the fault registers. */
        if ((error = printFaultRegisters(cid)) != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    /* Send NOP command to all nodes in order to prevent communication timeout. */
    sendNops();

    return BCC_STATUS_SUCCESS;
}



void startDischarge_Charge(){
	HAL_GPIO_WritePin(MOS_DISCHARGE_GPIO_Port, MOS_DISCHARGE_Pin, 1);
	HAL_GPIO_WritePin(MOS_CHARGE_GPIO_Port, MOS_CHARGE_Pin, 1);
	HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin,1);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
