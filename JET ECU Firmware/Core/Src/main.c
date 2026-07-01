/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── Battery voltage two-point calibration (raw ADC count based) ─────────────
   Direct linear interpolation between two known (raw, voltage) points —
   no forced "0 V" anchor, since that raw value drifts whenever the sense
   line's ground reference changes (e.g. powering the board from the
   battery supply vs. from USB). Anchoring on two real mid/high-range
   measurements instead is immune to that shift.

   HOW TO RECALIBRATE (if the supply/ground path changes again):
     1. Apply a known low voltage, note "Raw: xxxx" → LO point.
     2. Apply a known high voltage, note "Raw: xxxx" → HI point.
        (Use the widest voltage separation you can — it minimizes error.)
     3. Set the four constants below from those two readings.

   Board measurements (2026-07-01, direct-supply powered):
     5.00 V → Raw 678  |  10.00 V → Raw 1447  |  20.00 V → Raw 2993  |  24.00 V → Raw 3608
   Anchored on the 5 V / 24 V pair (widest separation); cross-checked against
   the 10 V and 20 V points, both within ±0.02 V of measured.               */
#define VBATT_RAW_LO     678UL   /* raw count at VBATT_MV_LO actual voltage */
#define VBATT_MV_LO     5000UL   /* actual voltage at the low calibration point (mV) */
#define VBATT_RAW_HI    3608UL   /* raw count at VBATT_MV_HI actual voltage */
#define VBATT_MV_HI    24000UL   /* actual voltage at the high calibration point (mV) */
#define VBATT_RAW_FLOOR  250UL   /* below this, treat as no battery connected → 0.00 V */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* ── ADC DMA buffer ─────────────────────────────────────────────────────────
   [0] = Battery voltage raw (PA1 / CH1)
   [1] = MCU temperature raw (internal TEMPSENSOR / CH16)
   DMA runs in circular mode — just read these two values anytime.           */
static uint16_t adc_buf[2];

/* ── RC PWM input (PA0, GPIO polling via DWT — matches Arduino pulseIn) ─── */
static uint32_t rc_pw_min        = 1535U;     /* default 0 % position (µs)  */
static uint32_t rc_pw_max        = 1950U;     /* default 100 % position (µs) */
static uint32_t rc_last_pw       = 1500;      /* last accepted pulse width   */
static uint32_t rc_last_valid_ms = 0;         /* kernel tick of last valid PW */
#define RC_MAX_STEP   400U                    /* max µs jump between samples  */
#define RC_TIMEOUT_MS 500U                    /* signal-loss timeout          */

/* ── UART RX ────────────────────────────────────────────────────────────── */
#define RX_BUF_SIZE 64
static uint8_t rx_dma_buf[RX_BUF_SIZE];      /* DMA target (circular)       */
static char    rx_cmd[RX_BUF_SIZE];          /* command copied on IDLE event */

/* ── FET GPIO pin array (index 0=FET1 … 5=FET6, all GPIOB) ────────────── */
static const uint16_t FET_PIN[6] = {
    FET_1_Pin, FET_2_Pin, FET_3_Pin,
    FET_4_Pin, FET_5_Pin, FET_6_Pin
};

/* ── RTOS synchronisation objects ──────────────────────────────────────── */
static osMutexId_t     uart_tx_mutex;   /* one sender at a time              */
static osSemaphoreId_t uart_cmd_sem;    /* signalled by UART RX IDLE ISR     */

/* ── Task handles ───────────────────────────────────────────────────────── */
static osThreadId_t throttleTaskHandle;
static osThreadId_t voltageTaskHandle;
static osThreadId_t fetReportTaskHandle;
static osThreadId_t serialRxTaskHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void     uart_send(const char *str);
static uint32_t pulseIn_us(GPIO_TypeDef *port, uint16_t pin, uint32_t timeout_us);
static uint32_t sample_rc_pw(uint8_t n);
void ThrottleTask(void *arg);
void VoltageTask(void *arg);
void FETReportTask(void *arg);
void SerialRxTask(void *arg);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* ADC self-calibration — recommended for F103 before first DMA conversion */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Start continuous ADC→DMA: fills adc_buf[0]=BATT, [1]=MCU_TEMP forever  */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 2);

  /* Enable DWT cycle counter — 32-bit, 64 MHz, wraps every ~67 s. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* Start UART2 RX DMA with IDLE detection — fires callback on each command  */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); /* suppress half-transfer */

  /* Boot banner — blocking TX before RTOS scheduler starts                   */
  HAL_UART_Transmit(&huart2, (uint8_t *)"JET ECU V1.0 Ready\r\n", 20, 100);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  uart_tx_mutex = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  uart_cmd_sem = osSemaphoreNew(1, 0, NULL);   /* binary, starts locked */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 256×4=1024-byte stacks: snprintf+uart_send+FreeRTOS context save need ~500 bytes */
  static const osThreadAttr_t thr_attr = { .name="ThrottleTask",  .stack_size=256*4, .priority=osPriorityAboveNormal };
  static const osThreadAttr_t volt_attr= { .name="VoltageTask",   .stack_size=256*4, .priority=osPriorityNormal      };
  static const osThreadAttr_t fet_attr = { .name="FETReportTask", .stack_size=256*4, .priority=osPriorityNormal      };
  static const osThreadAttr_t rx_attr  = { .name="SerialRxTask",  .stack_size=256*4, .priority=osPriorityHigh        };

  throttleTaskHandle  = osThreadNew(ThrottleTask,  NULL, &thr_attr);
  voltageTaskHandle   = osThreadNew(VoltageTask,   NULL, &volt_attr);
  fetReportTaskHandle = osThreadNew(FETReportTask, NULL, &fet_attr);
  serialRxTaskHandle  = osThreadNew(SerialRxTask,  NULL, &rx_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, TC1_CS_Pin|TC2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FET_5_Pin|FET_2_Pin|FET_1_Pin|FET_6_Pin
                          |FET_4_Pin|FET_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RPM_IN_Pin */
  GPIO_InitStruct.Pin = RPM_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TC1_CS_Pin TC2_CS_Pin */
  GPIO_InitStruct.Pin = TC1_CS_Pin|TC2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FET_5_Pin FET_2_Pin FET_1_Pin FET_6_Pin
                           FET_4_Pin FET_3_Pin */
  GPIO_InitStruct.Pin = FET_5_Pin|FET_2_Pin|FET_1_Pin|FET_6_Pin
                          |FET_4_Pin|FET_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ── UART helpers ─────────────────────────────────────────────────────────
   uart_send() acquires a mutex (one sender at a time) then calls the
   blocking HAL_UART_Transmit.  At 115200 baud the longest message is ~4 ms;
   FreeRTOS will time-slice other tasks during that window.  The blocking
   approach is simpler and more reliable than DMA TX + semaphore because it
   avoids the race where DMA completes before osSemaphoreAcquire is reached
   and the risk of semaphore corruption from a stack overflow in the task.    */

static void uart_send(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;
    osMutexAcquire(uart_tx_mutex, osWaitForever);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, len, HAL_MAX_DELAY);
    osMutexRelease(uart_tx_mutex);
}

/* Called from DMA IRQ on IDLE line or buffer full.
   Copies received bytes to rx_cmd, signals SerialRxTask, re-arms DMA.       */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2 && Size > 0 && Size < RX_BUF_SIZE) {
        memcpy(rx_cmd, rx_dma_buf, Size);
        rx_cmd[Size] = '\0';
        osSemaphoreRelease(uart_cmd_sem);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
}

/* ── RC PWM — direct GPIO polling, equivalent to Arduino pulseIn() ────────
   Waits for a HIGH pulse on PA0 within timeout_us microseconds.
   Uses DWT cycle counter (enabled in USER CODE BEGIN 2, 64 cycles = 1 µs).
   A shared deadline across all three phases means total wait ≤ timeout_us.   */

static uint32_t pulseIn_us(GPIO_TypeDef *port, uint16_t pin, uint32_t timeout_us)
{
    uint32_t deadline = DWT->CYCCNT + timeout_us * 64U;
    /* if pin is already HIGH, wait for it to go LOW first (skip current pulse) */
    while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
        if ((int32_t)(DWT->CYCCNT - deadline) >= 0) return 0;
    /* wait for rising edge */
    while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
        if ((int32_t)(DWT->CYCCNT - deadline) >= 0) return 0;
    uint32_t rise = DWT->CYCCNT;
    /* measure until falling edge */
    while (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
        if ((int32_t)(DWT->CYCCNT - deadline) >= 0) return 0;
    return (DWT->CYCCNT - rise) / 64U;
}

/* ── RC calibration helper ────────────────────────────────────────────────
   Directly measures n pulses (25 ms timeout each) — matches Arduino samplePW().
   Called from SerialRxTask; blocks up to n × (25 ms + 50 ms) ≈ 1.5 s.       */

static uint32_t sample_rc_pw(uint8_t n)
{
    uint32_t sum = 0; uint8_t cnt = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint32_t pw = pulseIn_us(GPIOA, GPIO_PIN_0, 25000U);
        if (pw >= 900U && pw <= 2100U) { sum += pw; cnt++; }
        osDelay(50);
    }
    return cnt ? (sum / cnt) : 0U;
}

/* ── Task: ThrottleTask (200 ms) ──────────────────────────────────────────
   Spike filter: reject any jump > RC_MAX_STEP µs.
   Signal-loss: if no valid pulse for RC_TIMEOUT_MS, output 0 %.
   Output: THR: xxx\r\n                                                       */

void ThrottleTask(void *arg)
{
    char buf[32];
    for (;;) {
        uint32_t pw = pulseIn_us(GPIOA, GPIO_PIN_0, 50000U);

        if (pw >= 900U && pw <= 2100U) {
            uint32_t delta = (pw > rc_last_pw) ? (pw - rc_last_pw) : (rc_last_pw - pw);
            if (delta <= RC_MAX_STEP || rc_last_pw == 1500U) {
                rc_last_pw       = pw;
                rc_last_valid_ms = osKernelGetTickCount();
            }
        }

        if ((osKernelGetTickCount() - rc_last_valid_ms) > RC_TIMEOUT_MS)
            rc_last_pw = 1500U;

        uint32_t thr = 0;
        if (rc_pw_min < rc_pw_max) {
            if (rc_last_pw <= rc_pw_min) {
                thr = 0;
            } else if (rc_last_pw >= rc_pw_max) {
                thr = 100;
            } else {
                thr = (rc_last_pw - rc_pw_min) * 100U / (rc_pw_max - rc_pw_min);
            }
        }

        snprintf(buf, sizeof(buf), "THR: %lu\r\n", thr);
        uart_send(buf);
        osDelay(200);
    }
}

/* ── Task: VoltageTask (500 ms batt, 1000 ms MCU temp) ───────────────────
   ADC DMA buffer read directly — no lock needed (16-bit atomic read, and
   small timing skew is irrelevant at these sample rates).

   Integer arithmetic throughout — avoids needing -u _printf_float linker flag.

   Battery divider: R1=39kΩ, R2=5.6kΩ → ratio = 44600/5600
   Calibration factor: 9905/10000 (≈ 0.9905)
   All voltages in mV, formatted with manual decimal split.

   MCU temp: V25=1580 mV (chip-calibrated), slope=4.3 mV/°C
   temp×10 = ((1580 - vt_mv) × 100) / 43 + 250                              */

void VoltageTask(void *arg)
{
    char    buf[64];
    uint8_t mcu_cnt = 0;

    for (;;) {
        /* 256-sample average with 1 ms between reads — 4x the original 64-sample
           window, which halves report-to-report ADC jitter (~sqrt(4)) without
           adding any cross-report lag (no memory carried between reports, so a
           real voltage change still shows up on the very next report).
           Total: 256 ms sampling + osDelay(500) ≈ 756 ms per voltage report.  */
        uint32_t sum = 0;
        for (uint16_t i = 0; i < 256; i++) {
            sum += adc_buf[0];
            osDelay(1);
        }
        uint16_t raw  = (uint16_t)(sum / 256U);

        /* V_ADC at divider output — kept for display and future diagnostics */
        uint32_t vadc = ((uint32_t)raw * 3300U) / 4095U;

        /* Battery voltage via direct two-point linear interpolation between
           VBATT_RAW_LO/MV_LO and VBATT_RAW_HI/MV_HI (see USER CODE BEGIN PD).
           raw ≤ VBATT_RAW_FLOOR means no battery connected → clamp to 0.    */
        uint32_t vbat = 0;
        if (raw > VBATT_RAW_FLOOR) {
            int32_t v = (int32_t)VBATT_MV_LO
                      + ((int32_t)raw - (int32_t)VBATT_RAW_LO)
                        * ((int32_t)VBATT_MV_HI - (int32_t)VBATT_MV_LO)
                        / ((int32_t)VBATT_RAW_HI - (int32_t)VBATT_RAW_LO);
            vbat = (v > 0) ? (uint32_t)v : 0;
        }

        snprintf(buf, sizeof(buf),
                 "Raw: %u\tV_ADC: %lu.%03lu V\tV_BATT: %lu.%02lu V\r\n",
                 raw,
                 vadc / 1000UL, vadc % 1000UL,
                 vbat / 1000UL, (vbat % 1000UL) / 10UL);
        uart_send(buf);

        /* MCU internal temperature — every second (every 2nd cycle) */
        if (++mcu_cnt >= 2) {
            mcu_cnt = 0;
            uint16_t rt   = adc_buf[1];
            int32_t vt_mv = ((int32_t)rt * 3300L) / 4095L;
            int32_t t10   = (((1580L - vt_mv) * 100L) / 43L) + 250L;
            snprintf(buf, sizeof(buf), "MCU: %ld.%ld C\r\n",
                     t10 / 10L, (t10 < 0 ? -t10 : t10) % 10L);
            uart_send(buf);
        }

        osDelay(500);
    }
}

/* ── Task: FETReportTask (200 ms) ─────────────────────────────────────────
   Reads actual GPIO output state via HAL_GPIO_ReadPin (reads ODR, atomic).
   Output: FET:x,x,x,x,x,x\r\n                                               */

void FETReportTask(void *arg)
{
    char buf[24];
    for (;;) {
        snprintf(buf, sizeof(buf), "FET:%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, FET_PIN[0]),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, FET_PIN[1]),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, FET_PIN[2]),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, FET_PIN[3]),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, FET_PIN[4]),
                 (unsigned)HAL_GPIO_ReadPin(GPIOB, FET_PIN[5]));
        uart_send(buf);
        osDelay(200);
    }
}

/* ── Task: SerialRxTask (event-driven, high priority) ─────────────────────
   Waits on uart_cmd_sem released by HAL_UARTEx_RxEventCallback.

   Commands:
     FETn:v        set FET n (1-6) to v (0|1)
     CAL_CENTER    sample 20 readings at stick center → sets rc_pw_min
     CAL_FULL      sample 20 readings at full throw   → sets rc_pw_max
     CAL_RESET     clear calibration, throttle returns 0 until recal          */

void SerialRxTask(void *arg)
{
    char cmd[RX_BUF_SIZE];
    char resp[32];

    for (;;) {
        osSemaphoreAcquire(uart_cmd_sem, osWaitForever);

        strncpy(cmd, rx_cmd, RX_BUF_SIZE - 1);
        cmd[RX_BUF_SIZE - 1] = '\0';

        /* Strip CR/LF */
        for (int i = 0; cmd[i]; i++)
            if (cmd[i] == '\r' || cmd[i] == '\n') { cmd[i] = '\0'; break; }

        /* FET_ALL:v — set all 6 FETs HIGH (v=1) or LOW (v=0) */
        if (strncmp(cmd, "FET_ALL:", 8) == 0) {
            GPIO_PinState st = (cmd[8] == '1') ? GPIO_PIN_SET : GPIO_PIN_RESET;
            for (int i = 0; i < 6; i++)
                HAL_GPIO_WritePin(GPIOB, FET_PIN[i], st);
            uart_send(cmd[8]=='1' ? "FET_ALL:1\r\n" : "FET_ALL:0\r\n");
        }
        /* FETn:v — set single FET */
        else if (cmd[0]=='F' && cmd[1]=='E' && cmd[2]=='T' &&
            cmd[3] >= '1' && cmd[3] <= '6' && cmd[4] == ':') {
            int n = cmd[3] - '1';
            HAL_GPIO_WritePin(GPIOB, FET_PIN[n],
                              cmd[5] == '1' ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
        /* CAL_CENTER */
        else if (strncmp(cmd, "CAL_CENTER", 10) == 0) {
            uint32_t pw = sample_rc_pw(10);
            rc_pw_min = pw; rc_pw_max = 0;
            snprintf(resp, sizeof(resp), "CAL_CENTER:%lu\r\n", pw);
            uart_send(resp);
        }
        /* CAL_FULL */
        else if (strncmp(cmd, "CAL_FULL", 8) == 0) {
            uint32_t pw = sample_rc_pw(10);
            rc_pw_max = pw;
            snprintf(resp, sizeof(resp), "CAL_FULL:%lu\r\n", pw);
            uart_send(resp);
        }
        /* CAL_RESET */
        else if (strncmp(cmd, "CAL_RESET", 9) == 0) {
            rc_pw_min = 0U; rc_pw_max = 0U; rc_last_pw = 1500U;
            uart_send("CAL_RESET:OK\r\n");
        }
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for (;;)
      osDelay(1000);
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
