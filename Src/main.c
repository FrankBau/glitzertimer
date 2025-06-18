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
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "param.h"

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
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

  // WS2812B LED ring with 24 LEDs externally connected
  // DI is TIM1 CH1 at PA8 which is a FT pin.
  // SYSCLK 80 MHz TIM used no prescaler, period=100 -> 1.25 us
  // data sheet   -> PWM setting
  // T0H 0.4us      32
  // T1H 0.8us      64
  // RESET >50us  sequence of 50 PWM pulses with 0% duty (all zero)

#define L 32
#define H 64

// we now have two rings of 24 LEDs each
// the 2nd mirrors the 1st, but dimmed
#define RINGS 2
#define LEDs 24

// some sources say that v4 of those LEDs needs >280us reset, so we round up 
#define RESET 300

// Follow the order of GRB to sent data and the high bit sent at first

uint8_t pwm[24*RINGS*LEDs+RESET] = {
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,  L,L,L,L,L,L,L,L,
  // reset pulses are completely zero, 0% PWM
};

void SetLED(int nr, uint32_t rgb) {
  int pos = 24*nr;
  pwm[pos++] = rgb & (1<<15) ? H : L; 
  pwm[pos++] = rgb & (1<<14) ? H : L; 
  pwm[pos++] = rgb & (1<<13) ? H : L; 
  pwm[pos++] = rgb & (1<<12) ? H : L; 
  pwm[pos++] = rgb & (1<<11) ? H : L; 
  pwm[pos++] = rgb & (1<<10) ? H : L; 
  pwm[pos++] = rgb & (1<< 9) ? H : L; 
  pwm[pos++] = rgb & (1<< 8) ? H : L; 

  pwm[pos++] = rgb & (1<<23) ? H : L; 
  pwm[pos++] = rgb & (1<<22) ? H : L; 
  pwm[pos++] = rgb & (1<<21) ? H : L; 
  pwm[pos++] = rgb & (1<<20) ? H : L; 
  pwm[pos++] = rgb & (1<<19) ? H : L; 
  pwm[pos++] = rgb & (1<<18) ? H : L; 
  pwm[pos++] = rgb & (1<<17) ? H : L; 
  pwm[pos++] = rgb & (1<<16) ? H : L;
  
  pwm[pos++] = rgb & (1<<7) ? H : L; 
  pwm[pos++] = rgb & (1<<6) ? H : L; 
  pwm[pos++] = rgb & (1<<5) ? H : L; 
  pwm[pos++] = rgb & (1<<4) ? H : L; 
  pwm[pos++] = rgb & (1<<3) ? H : L; 
  pwm[pos++] = rgb & (1<<2) ? H : L; 
  pwm[pos++] = rgb & (1<<1) ? H : L; 
  pwm[pos++] = rgb & (1<<0) ? H : L; 
}

// set a pair of LEDs: one on each ring
// the audience ring (first) gets brighter colors 
void SetLEDs(int nr, uint32_t rgb) {
  SetLED(nr, rgb);                    // bright, to audience
  SetLED(LEDs+nr, rgb & 0x0F0F0F);    // dimmed, to stage
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (void*)ptr, len, HAL_MAX_DELAY);
  return len;
}

uint64_t timeout;

char uart_rx_buffer[64];
int uart_rx_buffer_len = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(isdigit(uart_rx_buffer[uart_rx_buffer_len])) {
    uart_rx_buffer_len++;
    if(uart_rx_buffer_len < sizeof(uart_rx_buffer)) {
      HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_buffer_len], 1);
    }
  } else {
    uart_rx_buffer[uart_rx_buffer_len] = '\0';
    timeout = strtoul(uart_rx_buffer, NULL, 10);
    if(10 <= timeout && timeout < 10000) {
      printf("writing new timeout %lu to permanent storage (flash) and reset...\r\n",  (uint32_t)timeout);
      param_set_u64(&timeout, 1);
    } else {
      printf("invalid input, ignored and reset...\r\n");
    }
    HAL_UART_DeInit(&huart2);
    NVIC_SystemReset();
  }
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  param_get_u64(&timeout, 1);
  if(timeout > 10000) {
    timeout = 120;
    printf("failed reading valid alarm timeout from flash, resetting to default %lu\r\n", (uint32_t)timeout);
  }

  uint32_t timeout_ms_per_led = (uint32_t)(1000LLU*timeout) / LEDs;
  printf("the alarm timeout is set to %lu seconds, you may enter new decimal value in seconds (range 10..9999):\r\n", (uint32_t)timeout);

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&uart_rx_buffer[uart_rx_buffer_len], 1);

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (void*)pwm, sizeof(pwm) );

  // all green
  for(int i=0; i<LEDs; ++i) {
    SetLEDs(i, 0x00FF00);
  }

  // turn them off slowly, one by one
  for(int i=0; i<LEDs; ++i) {
    HAL_Delay(timeout_ms_per_led);
    SetLEDs(i, 0x000000);
  }

  // red alaram
  for(int i=0; i<5; ++i) {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    for(int i=0; i<LEDs; ++i) {
      SetLEDs(i, 0xFF0000);
    }
    HAL_Delay(200);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    for(int i=0; i<LEDs; ++i) {
      SetLEDs(i, 0x000000);
    }
    HAL_Delay(100);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // the glitzer
    int i = rand() % LEDs;
    int r = rand() % 8;
    int g = rand() % 8;
    int b = rand() % 8;
    SetLEDs(i, (1<<(16+r))|(1<<(8+g))|1<<(b));
    HAL_Delay(20);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
