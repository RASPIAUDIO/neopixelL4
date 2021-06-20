/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
/* USER CODE BEGIN PFP */
void refreshLeds(uint32_t *t);
void shiftLeds(uint32_t *t);
void DMALeds_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
////////////////////////
// nombre de leds gerees
#define NBL 12
uint32_t t[NBL];
uint32_t rr[NBL*24*3];
#define DIon  0x00000002
#define DIoff 0x00020000
uint32_t ON = DIon ;
uint32_t OFF = DIoff;
uint16_t LOCK =0;
// la matrice de leds est connectee sur le GPIO PA1
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
	t[0]= 0x00FF00;t[1]=0x00FF00;t[2]=0x00FF00;
	t[3]= 0xFF0000;t[4]=0xFF0000;t[5]=0xFF0000;
	t[6]= 0x0000FF;t[7]=0x0000FF;t[8]=0x0000FF;
	t[9]= 0xFFFFFF;t[10]=0xFFFFFF;t[11]=0xFFFFFF;
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
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  refreshLeds(t);
	  shiftLeds(t);
	  HAL_Delay(300);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Neo_GPIO_Port, Neo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Neo_Pin */
  GPIO_InitStruct.Pin = Neo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Neo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void shiftLeds(uint32_t *t)
{
	uint32_t rr;
	int i;
	rr=t[0];
	for(i=0;i<(NBL-1);i++)t[i]=t[i+1];
	t[11]=rr;
}
// initialisation timer 1 et DMA1 pour les échanges DMA avec les neopixels (GPIO PA1)
void DMALeds_Init()
{
	RCC->AHB1ENR |= 1;                // DMA1 clock validee
	RCC->APB2ENR |= (1ul<<11);       // TIM1 clock validee
	//////////////////////////////////////////////////////////
	//init TIM1
	//////////////////////////////////////////////////////////
	TIM1->PSC   = 0;                   // (prescale =1   => tick = 12.5ns)
	TIM1->CR1   = 0;
	TIM1->CR2   = 0;
	TIM1->DIER  = 0x4E00;              //dma request + CC1
	TIM1->ARR   = 32;                  // T=400ns
	TIM1->CCR1  = 32;                  // CC1 à T=1248ns
	TIM1->CCMR1 = 0x0000;
	TIM1->EGR   = 0x004C;

	//////////////////////////////////////////////////////////////////////////////
	// init DMA
	//////////////////////////////////////////////////////////////////////////////
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	// init dma ch2 CC1
	DMA1_Channel2->CCR = 0;                            //arret DMA ch2
	DMA1_Channel2->CNDTR = NBL*72;                     //longueur échange
	DMA1_Channel2->CPAR = (uint32_t)&(GPIOA->BSRR);    //adresse peripherique
	DMA1_Channel2->CMAR = (uint32_t)rr;                //adresse memoire

	//taille 32b (PSIZE et MSIZE), priorite very high , incrementation mémoire, dir memoire-> periph, IT fin d'échange
	DMA1_Channel2->CCR = 0x3A92;
	DMA1_CSELR->CSELR |= 0x00000070;   //triggered by  TIM1_ch1
}


//////////////////////////////////////////////
// affichage des NBL leds (WS2812B)
//
// tt => tableau de NBL elements (uint32_t)
// chaque element contient sur 24 bits (3x8) la couleur (GRB) d'une led
//
/////////////////////////////////////
void refreshLeds(uint32_t *tt)
{
	uint32_t t,m;
	int i,j,l;
	if(LOCK != 0) return;
	LOCK = 1;
	i=0;

	//mise en forme du tableau des etats du signal (1 point/400ns) (prets pour chargt dans GPIOx->BSRR)
	for(l=0;l<NBL;l++)
	{
		t=tt[l]<<8;
		for(j=0;j<24;j++)
		{
			rr[i++] = DIon;
			m=t & 0x80000000;
			t = t<<1;
			if(m==0) rr[i++] = DIoff;else rr[i++] = DIon;
			rr[i++] = DIoff;
		}
	}


	DMALeds_Init();

	DMA1_Channel2->CCR |= 1;               //demarrage DMA Ch2

	HAL_Delay(1);
	TIM1->CR1 = 1;         //demarrage TIM1

}
void DMA1_Channel2_IRQHandler(void)
{
	TIM1->CR1 = 0;                                          //arret TIM1
    DMA1->IFCR = 0xF0FF0;                                  	//reset flags IT ch2 ch3 ch5
	LOCK = 0;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
