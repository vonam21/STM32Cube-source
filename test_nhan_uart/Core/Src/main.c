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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdio.h>
char receive[50] ;
char data_send[50];
int number_receive = 0;
int flag_check_err = 1;
int number_message = 0;
int stt_receive =0;
uint8_t chuoi_receive[30];
int count_uart =0;
int Init_SIMA7680 (void)
{
	flag_check_err = 1;
	do{
		number_receive = 0;
	}while(flag_check_err == 1);
}
int SendATCommand(void)
{
	unsigned char data1[] = "AT+CMQTTSTART\r\n";
	flag_check_err = 1;
	HAL_UART_Transmit(&huart1, data1, strlen((char *)data1), 10);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	do{
		number_receive =1;
		uint32_t counterValue = __HAL_TIM_GET_COUNTER(&htim1);
		if(counterValue > 3000)
		{
			return -1;
		}
	}while(flag_check_err == 1);


//	  unsigned char data2[] = "AT+CMQTTACCQ=0,\"clientID\"\r\n";
//	  flag_check_err =1;
//	  HAL_UART_Transmit(&huart1, data2, strlen((char *)data2), 10);
//	  __HAL_TIM_SET_COUNTER(&htim1,0);
//	  do{
//		  number_message = 2;
//		  uint32_t counterValue = __HAL_TIM_GET_COUNTER(&htim1);
//		  if(counterValue >3000)
//		  {
//			  return -1;
//		  }
//	  }while(flag_check_err == 1);
//	  return 1;
}
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  unsigned char data[] = "day la chuoi";
  HAL_UART_Transmit(&huart1, data, strlen((char*) data), 10);
  HAL_UART_Receive_IT(&huart1, receive, 1);
  Init_SIMA7680();
  unsigned char data11[] = "day la chuoi1";
 HAL_UART_Transmit(&huart1, data11, strlen((char*) data11), 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  unsigned char data22[] = "day la chuoi2";
	  int t = SendATCommand();
	  HAL_UART_Transmit(&huart1, data22, strlen((char*) data22), 10);
	  HAL_Delay(2000);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	 if(huart->Instance == huart1.Instance)
	 {
		 if((strcmp(receive,"\n")==0)||(strcmp(chuoi_receive,"OK")==0)||(strcmp(chuoi_receive,"+CMQTTSTART: 0")==0)||(strcmp(chuoi_receive,"PB DONE")==0))
		 {
			 unsigned char data33[] = "debug";
			HAL_UART_Transmit(&huart1, data33, strlen((char*) data33), 10);
			 if(strcmp(chuoi_receive,"PB DONE" == 0) && (number_receive ==0))
			 {
				 unsigned char data333[] = "debug1";
				HAL_UART_Transmit(&huart1, data333, strlen((char*) data333), 10);
				 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				 flag_check_err = 0;
			 }
			 if(strcmp(chuoi_receive,"+CMQTTSTART: 0" == 0) && (number_receive ==1))
			 {
				 unsigned char data3333[] = "debug2";
				HAL_UART_Transmit(&huart1, data3333, strlen((char*) data3333), 10);
				 flag_check_err = 0;
			 }
			 stt_receive = 0;
			 for(int i =0;i< sizeof(chuoi_receive);i++)
			 {
				 chuoi_receive[i] =0;
			 }
		 }else
		 {
			 chuoi_receive[stt_receive] = receive;
			 stt_receive ++;
		 }
		 count_uart = count_uart+2;
		 unsigned char data33333[30];
		 sprintf(data33333,"%d",count_uart++);
		HAL_UART_Transmit(&huart1, data33333, strlen((char*) data33333), 10);
		unsigned char data44[] = "debug tong";
		HAL_UART_Transmit(&huart1, data44, strlen((char*) data44), 10);
		  HAL_UART_Receive_IT(&huart1, receive, 1);
//		 if((number_message == 1) && (strcmp(receive,"OK")==0))
//		 {
//			 HAL_UART_Receive_IT(huart, receive, 2);
//			 flag_check_err = 0;
//			 number_receive = 2;
//			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		 }
//		 if((number_message == 2) && (strcmp(receive,"OK")==0))
//		 {
//			 HAL_UART_Receive_IT(huart, receive, 18);
//			 flag_check_err = 0;
//			 number_receive = 18;
//			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		 }
//		 unsigned char data33[] = "debug";
//		HAL_UART_Transmit(&huart1, data33, strlen((char*) data33), 10);
//		 HAL_UART_Receive_IT(huart, receive, number_receive);
	 }
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
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
