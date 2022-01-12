/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

uint8_t flag_btn = 1;
uint16_t btn = 0;
uint8_t flag_irq = 0;
uint32_t time_irq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void Task1(void *pvParameters);
void Task2(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(Task1, "Buttom read", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(Task2, "Leds work", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Task1 (void *pvParameters)
{
	for(;;)
	{
		/*if(HAL_GPIO_ReadPin(BTN1_GPIO_Port,BTN1_Pin)==GPIO_PIN_SET)
		{
			while(HAL_GPIO_ReadPin(BTN1_GPIO_Port,BTN1_Pin)==GPIO_PIN_SET)
			{
				vTaskDelay(200);
			}
			btn = 1;
		}

		if(HAL_GPIO_ReadPin(BTN1_GPIO_Port,BTN1_Pin)==GPIO_PIN_RESET)
		{
			while(HAL_GPIO_ReadPin(BTN1_GPIO_Port,BTN1_Pin)==GPIO_PIN_RESET)
			{
				vTaskDelay(200);
			}
			btn = 0;
		}
		osDelay(100);
		*/
		if(flag_irq && (HAL_GetTick() - time_irq) > 200)
		{
			__HAL_GPIO_EXTI_CLEAR_IT(BTN1_Pin);
			NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

			flag_irq = 0;
			flag_btn++;
			if (flag_btn > 7) flag_btn = 0;
		}
	}
}

void Task2 (void *pvParameters)
{
	for(;;)
	{
		switch(flag_btn)
		{
			case 0:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
				break;
			case 1:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
				break;
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BTN1_Pin)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		flag_irq = 1;
		time_irq = HAL_GetTick();
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
