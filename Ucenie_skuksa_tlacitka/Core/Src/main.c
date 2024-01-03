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
#define DEBOUNCE_TIME	100
#define LED_TIME_BLINK	300
#define LED_TIME_SHORT	100
#define LED_TIME_LONG	1000

#define BTN_TIME		40
#define DEBOUNCE_MS		5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/*void blikac() {
	static uint32_t DelayCnt = 0;

	if (HAL_GetTick() > (DelayCnt + LED_TIME_BLINK)) {
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		DelayCnt = HAL_GetTick();
	}
}
*/

/*void tlacitka() {
	static uint32_t old_s2;
	uint32_t new_s2 = HAL_GPIO_IsInputPinSet(S2_GPIO_Port, S2_Pin);
	uint32_t new_s1 = HAL_GPIO_IsInputPinSet(S1_GPIO_Port, S1_Pin);
	static uint32_t off_time, old_time, debounce_time;

	static uint16_t debounce = 0xFFFF;

	if (HAL_GetTick() > off_time) {
		HAL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
	}

	if (HAL_GetTick() > old_time + BTN_TIME) {
		old_time = HAL_GetTick();

		if (old_s2 && !new_s2) {
			off_time = HAL_GetTick() + LED_TIME_SHORT;
			HAL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin,1); //ToDO
		}
		old_s2 = new_s2;
	}

	if (HAL_GetTick() > debounce_time + DEBOUNCE_MS) {
		debounce_time = HAL_GetTick();

		debounce <<= 1;
		if (new_s1) debounce |= 0x0001;

		if (debounce == 0x7FFF) {
			off_time = HAL_GetTick() + LED_TIME_LONG;
			HAL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin,1);
		}
	}
}
*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t Tick;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	/*uint8_t s1_state;
	uint8_t s2_state;
	uint8_t old_state_S1;
	uint8_t old_state_S2;
	uint32_t s1_push_time;
	uint32_t s2_push_time;

	s1_push_time = 0;
	s1_state = 0;
	old_state_S1 = 0;
	s2_push_time = 0;
	s2_state = 0;
	old_state_S2 = 0;
	*/

  while (1)
  {
	  static uint32_t DelayCnt = 0;

	  if (HAL_GetTick() > (DelayCnt + LED_TIME_BLINK)) {
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  DelayCnt = HAL_GetTick();
	  }


	  static uint32_t old_s2;
	  uint32_t new_s2 = HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);
	  uint32_t new_s1 = HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
	  static uint32_t off_time, old_time, debounce_time;

	  static uint16_t debounce = 0xFFFF;

	  //if (HAL_GetTick() > off_time) {
		//  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,0);
	 // }

	  if (HAL_GetTick() > old_time + BTN_TIME) {
		  old_time = HAL_GetTick();

		  if (old_s2 && !new_s2) {
			  off_time = HAL_GetTick() + LED_TIME_SHORT;
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,1);//ToDO while time
		  }
		  old_s2 = new_s2;
	  }

	  if (HAL_GetTick() > debounce_time + DEBOUNCE_MS) {
		  debounce_time = HAL_GetTick();

		  debounce <<= 1;
		  if (new_s1) debounce |= 0x0001;

		  if (debounce == 0x7FFF) {
			  off_time = HAL_GetTick() + LED_TIME_LONG; //TODO without time
			  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		  }
	  }
  }
}
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
