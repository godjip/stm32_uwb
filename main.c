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
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"
#include "DWM_functions.h"
//Code added for server(Below)
#include <stdlib.h>
#include <unistd.h>
#include "stdio.h"
#include "string.h"
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
//Code added for server(Below)
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Code added for server(Below)
volatile uint16_t uwADCxConvertedValue[3];

UART_HandleTypeDef *ESP8266_UART = &huart2;

#define RX_BUFFER_SIZE 100

volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxIndex = 0;
uint8_t singleByte;

#define Anchor 1 //Anchor number 1,2,3,4,5,6,7,8,9 ...
#define SERVER_IP "192.168.78.229" //"192.168.78.24"
#define SERVER_PORT "3000"

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//Code added for server(Below)
void ESP8266_SendCommand(const char* command);
HAL_StatusTypeDef ESP8266_WaitForResponse(const char* response, uint32_t timeout);
void ESP8266_SendDataToServer(char* jsonData);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Simple DS_TWR_Init-Resp

 extern void ss_init_main(void);
 extern void ss_init_start(void);
//sensor data from UWB
 extern float dist1,dist2;
 extern float temp1, temp2;
 extern uint8 wear_flag1, wear_flag2;
 extern uint8 sos_flag1, sos_flag2;
 extern uint8 server_shock_flag1, server_shock_flag2;

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
  HAL_InitTick(TICK_INT_PRIORITY);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Simple DS_TWR_Init-Resp
    /* Enable one of them */
     //ds_init_main();
  dwt_setinterrupt(SYS_STATUS_RXFCG, 1);

  ss_init_main();
  HAL_Delay(100*(Anchor - 1)); //anchor start delay ( anchor {n-1} * 100) //1,11,21, ... same delay
  HAL_TIM_Base_Start_IT(&htim2);

//    // Connect to Wi-Fi
//  ESP8266_SendCommand("AT+RST\r\n"); // Reset ESP8266
//  HAL_Delay(2000);
//  ESP8266_SendCommand("AT+CWJAP=\"DH\",\"1q2w3e4r\"\r\n");
//  HAL_Delay(2000);
   /******************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
	  // Create JSON object
	  char jsonData[100]; //buffer size
	  sprintf(jsonData, "{\"Anchor\":%2d, \"dist1\":%.2f,\"temp1\":%.2f,\"wear1\":%2d,\"sos1\":%2d,\"shock1\":%2d, \"dist2\":%.2f,\"temp2\":%.2f,\"wear2\":%2d,\"sos2\":%2d,\"shock2\":%2d}"
			  , Anchor, dist1, temp1, wear_flag1, sos_flag1, server_shock_flag1, dist2, temp2, wear_flag2, sos_flag2, server_shock_flag2);
	  // Send data to server
	  ESP8266_SendDataToServer(jsonData);

	  HAL_Delay(100);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, &singleByte, 1);
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DW_RESET_Pin|TCRT_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LedA_Pin|LedB_Pin|LedC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DW_IRQn_Pin */
  GPIO_InitStruct.Pin = DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DW_RESET_Pin DW_NSS_Pin TCRT_ON_Pin */
  GPIO_InitStruct.Pin = DW_RESET_Pin|DW_NSS_Pin|TCRT_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LedA_Pin LedB_Pin LedC_Pin */
  GPIO_InitStruct.Pin = LedA_Pin|LedB_Pin|LedC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance==TIM2){
		count++;
		ss_init_start();
		//server plus?

	}
}

void ESP8266_SendCommand(const char* command) {             //ESP8266 AT Order
  HAL_UART_Transmit(&huart2, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
}

HAL_StatusTypeDef ESP8266_WaitForResponse(const char* response, uint32_t timeout) {
  uint32_t tickstart = HAL_GetTick();

  while ((HAL_GetTick() - tickstart) < timeout) {
    if (strstr((char*)rxBuffer, response) != NULL) {
      return HAL_OK;
    }
    HAL_Delay(1);
  }

  return HAL_TIMEOUT;
}

void ESP8266_SendDataToServer(char* jsonData) {
    char command[100]; // 명령어를 저장할 충분한 크기의 버퍼

    // IP 주소와 포트를 포함한 명령어 구성
    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", SERVER_IP, SERVER_PORT);
    ESP8266_SendCommand(command);
    HAL_Delay(200);

    char dataLengthStr[11];
    sprintf(dataLengthStr, "%d", strlen(jsonData));
    ESP8266_SendCommand("AT+CIPSEND=");
    ESP8266_SendCommand(dataLengthStr);
    ESP8266_SendCommand("\r\n");
    HAL_Delay(500);

    ESP8266_SendCommand(jsonData);
    HAL_Delay(500);

    ESP8266_SendCommand("AT+CIPCLOSE\r\n");
}

//void ESP8266_SendCommand(const char* command) {             //ESP8266 AT Order
//  HAL_UART_Transmit(&huart2, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
//}
//
//HAL_StatusTypeDef ESP8266_WaitForResponse(const char* response, uint32_t timeout) {
//  uint32_t tickstart = HAL_GetTick();
//
//  while ((HAL_GetTick() - tickstart) < timeout) {
//
//    if (strstr((char*)rxBuffer, response) != NULL) {
//      return HAL_OK;
//    }
//    HAL_Delay(1);
//  }
//
//  return HAL_TIMEOUT;
//}
//
//void ESP8266_SendDataToServer(char* jsonData) {
//	//ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"192.168.78.24\",3000\r\n");
//	ESP8266_SendCommand("AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", SERVER_IP, SERVER_PORT);
//	HAL_Delay(200);
//
//	char dataLengthStr[11];
//	sprintf(dataLengthStr, "%d", strlen(jsonData));
//	ESP8266_SendCommand("AT+CIPSEND=");
//	ESP8266_SendCommand(dataLengthStr);
//	ESP8266_SendCommand("\r\n");
//	HAL_Delay(500);
//
//	ESP8266_SendCommand(jsonData);
//	HAL_Delay(500);
//
//	ESP8266_SendCommand("AT+CIPCLOSE\r\n");
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    rxBuffer[rxIndex++] = singleByte;

    if (rxIndex >= RX_BUFFER_SIZE) {
      rxIndex = 0;
    }

    HAL_UART_Receive_IT(&huart2, &singleByte, 1);
  }
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
