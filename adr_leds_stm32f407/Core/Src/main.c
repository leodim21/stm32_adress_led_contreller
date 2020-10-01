/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static struct local_s
{
	uint8_t 	*data_buf[8];
	uint16_t  	data_len;
	uint16_t 	pulse[2];
	uint8_t 	bit_count;
	uint16_t 	byte_count;
	uint8_t 	bit_count_t4;
	uint16_t 	byte_count_t4;
	uint8_t		send_start;
}local=
{
	.pulse ={(uint16_t)NUL_SYG, (uint16_t)ONE_SYG}
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
LED_HandleTypeDef add_leds_buf1;
LED_HandleTypeDef add_leds_buf2;

uint8_t pState0 = 0;
uint8_t pState1 = 1;
uint8_t p4State0 = 0;
uint8_t p4State1 = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM1_Init( void );
static void MX_TIM8_Init( void );
static void TIM_SendEnd( void );
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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM1_Init();
  MX_TIM8_Init();
  memset( add_leds_buf1.data_buf, 0x55, PIXELS_NUM*24 );
  LED_SendData(&add_leds_buf1, PIXELS_NUM*24);
  HAL_SPI_Receive_DMA(&hspi1, add_leds_buf1.data_buf, PIXELS_NUM*24);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		{
			HAL_Delay(100);
			LED_SendData(&add_leds_buf1, PIXELS_NUM*24);
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_TIM1_Init( void )
{
	/*Configure GPIO pins : PA0 PA1 PA2 PA3
	 PA4 PA5 PA6 PA7 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	GPIOE->AFR[1] |=  (1 << GPIO_AFRH_AFSEL9_Pos)
					| (1 << GPIO_AFRH_AFSEL11_Pos)
					| (1 << GPIO_AFRH_AFSEL13_Pos)
					| (1 << GPIO_AFRH_AFSEL14_Pos);

	GPIOE->MODER |= GPIO_MODER_MODER9_1
					| GPIO_MODER_MODER11_1
					| GPIO_MODER_MODER13_1
					| GPIO_MODER_MODER14_1;

	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR9_1
					| GPIO_OSPEEDER_OSPEEDR11_0 | GPIO_OSPEEDER_OSPEEDR11_1
					| GPIO_OSPEEDER_OSPEEDR13_0 | GPIO_OSPEEDER_OSPEEDR13_1
					| GPIO_OSPEEDER_OSPEEDR14_0 | GPIO_OSPEEDER_OSPEEDR14_1;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->CR1 |= TIM_CR1_ARPE;

	TIM1->PSC = 0;
	TIM1->ARR = PERIOD_SYG;

	TIM1->CCR1 = local.pulse[pState0];
	TIM1->CCR2 = local.pulse[pState1];
	TIM1->CCR3 = local.pulse[pState0];
	TIM1->CCR4 = local.pulse[pState1];

	TIM1->CCER |= TIM_CCER_CC1E;   	// Capture/Compare 1 output enable
	TIM1->CCER |= TIM_CCER_CC2E;   	// Capture/Compare 2 output enable
	TIM1->CCER |= TIM_CCER_CC3E;   	// Capture/Compare 2 output enable
	TIM1->CCER |= TIM_CCER_CC4E;   	// Capture/Compare 2 output enable

//	GPIOA->ODR &= ~GPIO_ODR_OD8;

	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_0;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2; 	//110: PWM mode 1

	TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM1->CCMR1 &= ~TIM_CCMR1_OC2M_0;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_2; 	//110: PWM mode 1

	TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
	TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_0;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2; 	//110: PWM mode 1

	TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
	TIM1->CCMR2 &= ~TIM_CCMR2_OC4M_0;
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_2; 	//110: PWM mode 1

	NVIC_EnableIRQ( TIM1_UP_TIM10_IRQn );
	NVIC_EnableIRQ( TIM1_UP_TIM10_IRQn );
	TIM1->DIER |= TIM_DIER_UIE;

	TIM1->BDTR|= TIM_BDTR_MOE;     		//MOE: Main output enable
	TIM1->CR1 &= ~TIM_CR1_CEN; 			//Bit 0 CEN: Counter enable

}
static void MX_TIM8_Init( void )
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	GPIOC->AFR[0] |=  (3 << GPIO_AFRL_AFSEL6_Pos)
					| (3 << GPIO_AFRL_AFSEL7_Pos);
	GPIOC->AFR[1] |=  (3 << GPIO_AFRH_AFSEL8_Pos)
					| (3 << GPIO_AFRH_AFSEL9_Pos);

	GPIOC->MODER |= GPIO_MODER_MODER6_1
				  | GPIO_MODER_MODER7_1
				  | GPIO_MODER_MODER8_1
				  | GPIO_MODER_MODER9_1;

	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR6_1
					| GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR7_1
					| GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR8_1
					| GPIO_OSPEEDER_OSPEEDR9_0 | GPIO_OSPEEDER_OSPEEDR9_1;

	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

	TIM8->CR1 |= TIM_CR1_ARPE;

	TIM8->PSC = 0;
	TIM8->ARR = PERIOD_SYG;

	TIM8->CCR1 = local.pulse[p4State0];
	TIM8->CCR2 = local.pulse[p4State1];
	TIM8->CCR3 = local.pulse[p4State0];
	TIM8->CCR4 = local.pulse[p4State1];

	TIM8->CCER |= TIM_CCER_CC1E;   	// Capture/Compare 1 output enable
	TIM8->CCER |= TIM_CCER_CC2E;   	// Capture/Compare 2 output enable
	TIM8->CCER |= TIM_CCER_CC3E;   	// Capture/Compare 2 output enable
	TIM8->CCER |= TIM_CCER_CC4E;   	// Capture/Compare 2 output enable

	TIM8->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM8->CCMR1 &= ~TIM_CCMR1_OC1M_0;
	TIM8->CCMR1 |= TIM_CCMR1_OC1M_1;
	TIM8->CCMR1 |= TIM_CCMR1_OC1M_2; 	//110: PWM mode 1

	TIM8->CCMR1 |= TIM_CCMR1_OC2PE;
	TIM8->CCMR1 &= ~TIM_CCMR1_OC2M_0;
	TIM8->CCMR1 |= TIM_CCMR1_OC2M_1;
	TIM8->CCMR1 |= TIM_CCMR1_OC2M_2; 	//110: PWM mode 1

	TIM8->CCMR2 |= TIM_CCMR2_OC3PE;
	TIM8->CCMR2 &= ~TIM_CCMR2_OC3M_0;
	TIM8->CCMR2 |= TIM_CCMR2_OC3M_1;
	TIM8->CCMR2 |= TIM_CCMR2_OC3M_2; 	//110: PWM mode 1

	TIM8->CCMR2 |= TIM_CCMR2_OC4PE;
	TIM8->CCMR2 &= ~TIM_CCMR2_OC4M_0;
	TIM8->CCMR2 |= TIM_CCMR2_OC4M_1;
	TIM8->CCMR2 |= TIM_CCMR2_OC4M_2; 	//110: PWM mode 1

	TIM8->BDTR|= TIM_BDTR_MOE;     		//MOE: Main output enable
	TIM8->CR1 &= ~TIM_CR1_CEN; 			//Bit 0 CEN: Counter enable
}

void LED_SendData( LED_HandleTypeDef *data_buf, uint16_t data_len )
{
	if (!((TIM1->CR1 & TIM_CR1_CEN)||(TIM4->CR1 & TIM_CR1_CEN)))
	{
		local.data_buf[0] = data_buf->led_data.line1;
		local.data_buf[1] = data_buf->led_data.line2;
		local.data_buf[2] = data_buf->led_data.line3;
		local.data_buf[3] = data_buf->led_data.line4;
		local.data_buf[4] = data_buf->led_data.line5;
		local.data_buf[5] = data_buf->led_data.line6;
		local.data_buf[6] = data_buf->led_data.line7;
		local.data_buf[7] = data_buf->led_data.line8;

		local.data_len = data_len/8;
		local.send_start = 1;
		local.bit_count = 0;
		local.byte_count = 0;

		TIM1->CCR1 = local.pulse[(0x01 & local.data_buf[0][local.byte_count] >> local.bit_count)];
		TIM1->CCR2 = local.pulse[(0x01 & local.data_buf[1][local.byte_count] >> local.bit_count)];
		TIM1->CCR3 = local.pulse[(0x01 & local.data_buf[2][local.byte_count] >> local.bit_count)];
		TIM1->CCR4 = local.pulse[(0x01 & local.data_buf[3][local.byte_count] >> local.bit_count)];

		TIM8->CCR1 = local.pulse[(0x01 & local.data_buf[4][local.byte_count] >> local.bit_count)];
		TIM8->CCR2 = local.pulse[(0x01 & local.data_buf[5][local.byte_count] >> local.bit_count)];
		TIM8->CCR3 = local.pulse[(0x01 & local.data_buf[6][local.byte_count] >> local.bit_count)];
		TIM8->CCR4 = local.pulse[(0x01 & local.data_buf[7][local.byte_count] >> local.bit_count)];

		TIM1->CR1 |= TIM_CR1_CEN;
		TIM8->CR1 |= TIM_CR1_CEN;
	}
}
static void TIM_SendEnd( void )
{
	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM8->CR1 &= ~TIM_CR1_CEN;
	local.send_start = 0;
}
void TIM1_UPD_Servo_Set( void )
{
	if(local.bit_count++ > 6)
	{
		local.bit_count = 0;
		local.byte_count++;
	}
	TIM1->CCR1 = local.pulse[(0x01 & local.data_buf[0][local.byte_count] >> local.bit_count)];
	TIM1->CCR2 = local.pulse[(0x01 & local.data_buf[1][local.byte_count] >> local.bit_count)];
	TIM1->CCR3 = local.pulse[(0x01 & local.data_buf[2][local.byte_count] >> local.bit_count)];
	TIM1->CCR4 = local.pulse[(0x01 & local.data_buf[3][local.byte_count] >> local.bit_count)];

	TIM8->CCR1 = local.pulse[(0x01 & local.data_buf[4][local.byte_count] >> local.bit_count)];
	TIM8->CCR2 = local.pulse[(0x01 & local.data_buf[5][local.byte_count] >> local.bit_count)];
	TIM8->CCR3 = local.pulse[(0x01 & local.data_buf[6][local.byte_count] >> local.bit_count)];
	TIM8->CCR4 = local.pulse[(0x01 & local.data_buf[7][local.byte_count] >> local.bit_count)];

	( local.byte_count == local.data_len )?( TIM_SendEnd() ):( TIM1->CR1 |= TIM_CR1_CEN );
}
void HAL_SPI_RxCpltCallback( SPI_HandleTypeDef *hspi )
{
	if (hspi == &hspi1)
	{
		HAL_SPI_Receive_DMA(&hspi1, add_leds_buf1.data_buf, PIXELS_NUM*24);
		LED_SendData(&add_leds_buf1, PIXELS_NUM*24);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
