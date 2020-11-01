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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f429i_discovery_lcd.h"
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
DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
osThreadId grabI2CTaskHandle;
osThreadId readI2CTaskHandle;
/* USER CODE BEGIN PV */
uint8_t LCdev[4];
uint8_t LCcnt;
uint8_t bridgeValue[4 * 4];
uint16_t counts;
uint16_t errs;
uint16_t stale;
volatile unsigned int u1rc;
volatile unsigned int u1hrc;
volatile unsigned int u1tc;
volatile unsigned int u1htc;
volatile unsigned int u1ec;
volatile unsigned int u1ic;
unsigned char dbgBuf[256];
unsigned char input[64];
unsigned char u1tx[256];
unsigned char u1rx[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskI2C(void const * argument);
void readI2C(void const * argument);

/* USER CODE BEGIN PFP */
void my_printf(char *format, ...)
	_ATTRIBUTE ((__format__ (__printf__, 1, 2)));
HAL_StatusTypeDef powerup_and_read(I2C_HandleTypeDef *h, uint8_t d, uint8_t *dataBuf, const uint32_t I2C_Timeout);
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

  // !!!!!  Make sure MX_DMA_Init(); comes before the UARTS here below...
  // !!!!!  Get enough stack room for the tasks (set minimal stack size to 512 at this point)
  // !!!!!  Watch out about interrupt priorities for DMA and/or UARTs ... ??? not sure what is the real deal there
  //        but when 0 instead of 5 I get vTaskNotifyGiveFromISR freezing the system from the DMA completion callback...

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of grabI2CTask */
  osThreadDef(grabI2CTask, StartTaskI2C, osPriorityIdle, 0, 512);
  grabI2CTaskHandle = osThreadCreate(osThread(grabI2CTask), NULL);

  /* definition and creation of readI2CTask */
  osThreadDef(readI2CTask, readI2C, osPriorityNormal, 0, 512);
  readI2CTaskHandle = osThreadCreate(osThread(readI2CTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 165;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c3.Init.OwnAddress1 = 252;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin|Sensor_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin Sensor_PWR_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin|Sensor_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_HS_ID_Pin OTG_HS_DM_Pin OTG_HS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_HS_ID_Pin|OTG_HS_DM_Pin|OTG_HS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_HS_Pin */
  GPIO_InitStruct.Pin = VBUS_HS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_HS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin); // toggle red led

	if (huart->Instance == USART1)
	{
		u1tc += 1;
		/* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR(grabI2CTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1htc += 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1rc += 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1hrc += 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
		u1ec += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}

#if 0
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c3)
	{
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		memcpy(bridgeValue,dataBuf,2);
	}
}
#endif

int UART_Receive(unsigned char *dest, const unsigned char *rx, UART_HandleTypeDef *huart, unsigned int *uxcc, const unsigned int max)
{
	unsigned int cc = __HAL_DMA_GET_COUNTER(huart->hdmarx);
	if (*uxcc != cc)
	{
		HAL_UART_DMAPause(huart);
  	int len = 0;
		if (cc > *uxcc)
		{
			for (unsigned int i = max - *uxcc; i < max; i++)
				dest[len++] = rx[i];
			for (unsigned int i = 0; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		else
		{
			for (unsigned int i = max - *uxcc; i < max - cc; i++)
				dest[len++] = rx[i];
		}
		HAL_UART_DMAResume(huart);
  	*uxcc = cc;
  	return len;
	}
	return 0;
}

void my_printf(char *format, ...)
{
	va_list args;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
	uint32_t ulNotificationValue;
	HAL_StatusTypeDef res;
	va_start(args, format);
	int len = vsniprintf((char *) dbgBuf, 256, format, args);
	va_end(args);
	res = HAL_UART_Transmit_DMA(&huart1, dbgBuf, len);
	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
	if (res != HAL_OK || ulNotificationValue != 1)
	{
		/* Something went wrong during output... */
		//HAL_UART_DMAStop(&huart1);
		errs += 8;
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); // toggle green led
		len = snprintf((char *) dbgBuf, 256, "Timeout waiting on the DMA completion for %u ticks - seems bad\r\n", 200);
		HAL_UART_Transmit(&huart1, dbgBuf, len, xMaxBlockTime);
	}
}

HAL_StatusTypeDef read_cell(I2C_HandleTypeDef *h, uint8_t d, uint8_t a, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = a;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_I2C_Master_Transmit(h, d, dataBuf, 3, I2C_Timeout);
	memset(dataBuf, 0, 3);
	return HAL_I2C_Master_Receive(h, d | 1, dataBuf, 3, I2C_Timeout);
}

HAL_StatusTypeDef powerup_and_read(I2C_HandleTypeDef *h, uint8_t d, uint8_t *dataBuf, const uint32_t I2C_Timeout)
{
	dataBuf[0] = 0xA0;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(3);
	HAL_I2C_Master_Transmit(h, d, dataBuf, 3, I2C_Timeout);
	return read_cell(h, d, 2, dataBuf, I2C_Timeout);
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
	const uint32_t low = 950;
	uint16_t c = 0;
	BSP_LCD_Init();
	/* Initialize the LCD Layers */
	BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);
	/* Set LCD Foreground Layer  */
	BSP_LCD_SelectLayer(1);
	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
	/* Clear the LCD */
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	/* Set the LCD Text Color */
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);

	/* Display LCD messages */
	BSP_LCD_DisplayStringAt(0, 10, (uint8_t*)"STM32F429I BSP", CENTER_MODE);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)"Weighing Station", CENTER_MODE);
	BSP_LCD_SetFont(&Font20);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  /* Infinite loop */
	for(;;)
	{
		uint8_t msg[50];
		sprintf((char *)msg,"%u %u %u %u",c,errs,counts,stale);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2, msg, CENTER_MODE);
		sprintf((char *)msg,"%02x %02x %02x %02x",bridgeValue[0],bridgeValue[1],bridgeValue[2],bridgeValue[3]);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35, msg, CENTER_MODE);
		uint32_t weight = (bridgeValue[0] & 0x3f) * 256 + bridgeValue[1];
		sprintf((char *)msg,"  raw : %ld  ", weight);
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 75, msg, CENTER_MODE);
		if (weight < low)
			weight = 0;
		else
			weight -= low;
		weight *= 5000;
		weight /= 14000;
		sprintf((char *)msg,"W: %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 55, msg, CENTER_MODE);
		// Try this : Temperature = (Counts x 200 / 2048) - 50
		/*
		 * Temperature Output vs Counts
		 * Output oC  Digital Counts (decimal)  Digital  Counts (hex)
		 *   -50                0                    0 x 0000
		 *     0              512                    0 X 0200
		 *    10              614                    0 X 0266
		 *    25              767                    0 X 02FF
		 *    40              921                    0 X 0399
		 *    85             1381                    0 X 0565
		 *   150             2047                    0 X 07FF
		 */
		uint32_t temp = (bridgeValue[2] << 3) + (bridgeValue[3] >> 5);
		temp *= 2000;
		temp /= 2048; // just a guess at this point...
		temp -= 50;
		sprintf((char *)msg,"  T: %2d.%01d C  ", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
		BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 95, msg, CENTER_MODE);
		osDelay(200);
		c += 1;
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskI2C */
/**
* @brief Function implementing the grabI2CTask thread.
* @param argument: Not used
* @retval None
*/
/*
	* Status Bits (2 MSB of Output Data Packet) Definition
	* 00 - Normal Operation. Good Data Packet
	* 01 - Reserved
	* 10 - Stale Data. Data has been fetched since last measurement cycle.
	* 11 - Fault Detected
*/
/*
 *   | Action            | Byte 1            |  Byte 2   |   Byte 3    |  Byte 4    | Notes
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 *   | Put sensor into   | [7 bit address *] |           |             |            | Data must be
 * 1 | command mode      |        +          |   0xA0    |    0x00     |   0x00     | sent within 6ms
 *   |                   | [Write bit = 0]   |           |             |            | of power up
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 *   | Command to read   | [7 bit address *] |           |             |            |
 * 2 | EEPROM word 02    |        +          |   0x02    |    0x00     |   0x00     |
 *   | from sensor       | [Write bit = 0]   |           |             |            |
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 *   | Fetch EEPROM      | [7 bit address *] |   0x5A    |   Word 02   |  Word 02   |
 * 3 | word 02           |        +          | (response | [bits 15:8] | [bits 7:0] |
 *   |                   | [Read bit = 1]    |  byte)    |             |            |
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 *   | Modify Word 02    |                   |           |             |            | Bits [9:3]: I2C address
 * 4 | in user           |                   |           |             |            | required Bits [12:10]:
 *   | software          |                   |           |             |            | 011 (communication lock)
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 *   | Write new version | [7 bit address *] |           |   Word 02   |  Word 02   |
 * 5 | of Word 02 to     |        +          |   0x42    | [bits 15:8] | [bits 7:0] |
 *   | sensor EEPROM     | [Write bit = 0]   |           |             |            |
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 *   | Exit command mode | [7 bit address *] |           |             |            |
 * 6 | & start normal    |        +          |   0x80    |    0x00     |   0x00     |
 *   | operating mode    | [Write bit = 0]   |           |             |            |
 * --+-------------------+-------------------+-----------+-------------+------------+-------------------------
 */
/*
 * I2C Address
 * Code Device Address (hex)
 *  X    Analog Output*
 *  0    0x28*
 *  1    0x36
 *  2    0x46
 *  3    0x48
 *  4    0x51
 */
/*
 * Original value in word 02 : 0x5A 0x0D     0x46
 *                             0x5A 00001101 01000110
 *                                  000                - EEPROM lock
 *                                     011             - communication address lock
 *                                        01 01000     - address 7 bits
 *                                                11   - diagnostic config
 *                                                  0  - SPI clock polarity
 */
/* USER CODE END Header_StartTaskI2C */
void StartTaskI2C(void const * argument)
{
  /* USER CODE BEGIN StartTaskI2C */
	HAL_UART_Receive_DMA(&huart1, u1rx, 64);
	unsigned int u1cc = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	const int nbAddress = 5;
	const uint8_t address[5] = { 0x28, 0x36, 0x46, 0x48, 0x51 };
	HAL_StatusTypeDef res;
	uint8_t detected = 0;
	uint8_t counted = 0;
	uint8_t dataBuf[4];
	int inLen = 0;
	int hb = 0;
	counts = 0;
	errs = 0;
	memset(bridgeValue,0,2);
	my_printf("\r\nStarting Run on %s\r\n# ", tskKERNEL_VERSION_NUMBER);
	// Now see what we do with cell
	for(;;)
	{
		int len = UART_Receive(input + inLen, u1rx, &huart1, &u1cc, 64);
		if (len > 0)
		{
			my_printf("%.*s", len, input + inLen);
			inLen += len;
			if (input[inLen - 1] == '\r')
			{
				int cmdLen = inLen - 1;
				my_printf("\nReceived command '%.*s'\r\n", cmdLen, input);
				inLen = 0;
				if (strncmp((char *) input, "UART", cmdLen) == 0)
					my_printf("u1rc = %u u1hrc = %u u1tc = %u u1htc = %u u1ec = %u u1ic = %u u1cc = %u\r\n# ", u1rc, u1hrc, u1tc, u1htc, u1ec, u1ic, u1cc);
				else if (strncmp((char *) input, "done", cmdLen) == 0)
				{
					my_printf("\nSetup done - now releasing read task\r\n# ");
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_SET);
					HAL_Delay(10);
					// Indicate to the readI2C task that we are done
					if (counted == 0)
					{
						LCdev[0] = 0x28 << 1;
						LCcnt = 1;
					}
					else
						LCcnt = counted;
					break;
				}
				else if (strncmp((char *) input, "scan", cmdLen) == 0)
				{
					/* Power up load cell and detect it */
					detected = 0;
					counted = 0;
					for (unsigned int i = 0; i < nbAddress; i++)
					{
						uint8_t d = address[i] << 1;
						res = powerup_and_read(&hi2c3, d, dataBuf, I2C_Timeout);
						HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
						if (res == HAL_OK && dataBuf[0] == 0x5A)
						{
							uint8_t a = (dataBuf[1] >> 2) & 3;
							uint8_t b = ((dataBuf[1] & 3) << 5) | (dataBuf[2] >> 3);
							my_printf("Scan 0x%02X - received 0x%02X 0x%02X 0x%02X => %u 0x%02X\r\n", address[i], dataBuf[0], dataBuf[1], dataBuf[2], a, b);
							detected |= 1 << i;
							LCdev[counted++] = d;
						} else {
							my_printf("Scan 0x%02X - empty\r\n", address[i]);
						}
						HAL_Delay(10);
					}
					my_printf("Scan completed - found %u device%s - flags 0x%02X\r\n# ", counted, counted > 1 ? "s" : "", detected);
				}
				else if (strncmp((char *) input, "set ", 4) == 0 && cmdLen == 6)
				{
					/* WARNING - writing bad data to the EEPROM can render the sensor unusable...  */
					/*
					 * From ZSC31014 datasheet
					 *
					 *  Command Byte    Third and                                                                    Processing
					 * 8 Command Bits  Fourth Bytes    Description       Time ††
					 *    (Hex)          16 Data                          4MHz/1MHz
					 *                  Bits(Hex)
					 * 00HEX to 13HEX  0000HEX       EEPROM Read of addresses 00HEX to 13HEX.
					 *                               After this command has been sent and executed, a data
					 *                               fetch of three bytes must be performed. The first byte will be     10μs
					 *                               a response byte, which should be a 5AHEX, and then the next
					 *                               two bytes will be the EEPROM data.
					 * 40HEX to 53HEX  YYYYHEX       Write to EEPROM addresses 00HEX to 13HEX.
					 *                 (Y= data)     If the command is an EEPROM write, then the 16 bits of             15ms
					 *                               data sent will be written to the address specified in the 6
					 *                               LSBs of the command byte.
					 * 80HEX           0000HEX       Start_NOM => Ends Command Mode and transitions to
					 *                               Normal Operation Mode. When a Start_NOM command is              15ms if EEPROM
					 *                               executed, a flag is checked to see if EEPROM was                signature is
					 *                               programmed during Command Mode. If so, the device will          updated;
					 *                               regenerate the checksum and update the signature                10μs otherwise
					 *                               EEPROM word.
					 * A0HEX           0000HEX       Start_CM => Start Command Mode; used to enter
					 *                               Command Mode. Start_CM is only valid during the power-on           10μs
					 *                               command window.
					 */
					/* Set I2C address to new value */
					int old = input[4] - '0';
					int new = input[5] - '0';
					if (old < 0 || old > 4 || new < 0 || new > 4 || (detected & (1 << old)) == 0)
						my_printf("Something is wrong... old = %d, new = %d, detected = %d\r\n", old, new, detected);
					else
					{
						uint8_t d = address[old] << 1;
						res = powerup_and_read(&hi2c3, d, dataBuf, I2C_Timeout);
						uint8_t a = (dataBuf[1] >> 2) & 3;
						uint8_t b = ((dataBuf[1] & 3) << 5) | (dataBuf[2] >> 3);
						my_printf("Got %d 0x%02X 0x%02X 0x%02X %u 0x%02X\r\n", res, dataBuf[0], dataBuf[1], dataBuf[2], a, b);
						if (res == HAL_OK && dataBuf[0] == 0x5A)
						{
							dataBuf[0] = 0x42;
							dataBuf[1] = 0x0C | (address[new] >> 5);
							dataBuf[2] = 0x06 | ((address[new] << 3) & 0xff);
							my_printf("Sending 0x%02X 0x%02X 0x%02X\r\n", dataBuf[0], dataBuf[1], dataBuf[2]);
							res = HAL_I2C_Master_Transmit(&hi2c3, d, dataBuf, 3, I2C_Timeout);
							my_printf("HAL res after send new address = %d\r\n", res);
							HAL_Delay(15); // wait 15 ms according to DS
							dataBuf[0] = 0x80;
							dataBuf[1] = 0;
							dataBuf[2] = 0;
							res = HAL_I2C_Master_Transmit(&hi2c3, d, dataBuf, 3, I2C_Timeout);
							my_printf("HAL res after send exit command mode = %d\r\n", res);
							HAL_Delay(15); // wait another 15 ms to update EEPROM signature
							osDelay(50);
							HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
							my_printf("Will now pause 10 seconds\r\n");
							osDelay(10000);
							HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_SET);
							HAL_Delay(10);
							d = address[new] << 1;
							memset(dataBuf,0,4);
							res = HAL_I2C_Master_Receive(&hi2c3, d | 1, dataBuf, 4, I2C_Timeout);
							my_printf("Retrieved 0x%02X 0x%02X 0x%02X 0x%02X %d with new address 0x%02X\r\n", dataBuf[0], dataBuf[1], dataBuf[2], dataBuf[3], res, address[new]);
							osDelay(50);
							memset(dataBuf,0,4);
							res = HAL_I2C_Master_Receive(&hi2c3, d | 1, dataBuf, 4, I2C_Timeout);
							my_printf("Retrieved 0x%02X 0x%02X 0x%02X 0x%02X %d with new address 0x%02X\r\n", dataBuf[0], dataBuf[1], dataBuf[2], dataBuf[3], res, address[new]);
							if (res == HAL_OK)
								my_printf("Set new I2C address completed successfully - 0x%02X -> 0x%02X\r\n", address[old], address[new]);
							else
								my_printf("Set new I2C address failed : %d\r\n", res);
						} else {
							my_printf("Power up check failed : %d 0x%02X 0x%02X 0x%02X %u 0x%02X\r\n", res, dataBuf[0], dataBuf[1], dataBuf[2], a, b);
						}
						HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
						detected = 0;
						counted = 0;
					}
					my_printf("# ");
				}
				else if (strncmp((char *) input, "reset", cmdLen) == 0)
				{
					/*
					 * 3.2.10 General call address
					 * 0000 0110 (06h) — Reset and write programmable part of slave address by hardware.
					 * On receiving this 2-byte sequence, all devices designed to respond to the general call
					 * address reset and take in the programmable part of their address.
					 */
					dataBuf[0] = 0x06;
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_SET);
					HAL_Delay(3);
					res = HAL_I2C_Master_Transmit(&hi2c3, 0, dataBuf, 1, I2C_Timeout);
					HAL_Delay(10);
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
					my_printf("General call with reset done - res = %d\r\n# ", res);
				}
				else if (strncmp((char *) input, "read", cmdLen) == 0)
				{
					/* read EEPROM values */
					uint8_t d = LCdev[0];
					res = powerup_and_read(&hi2c3, d, dataBuf, I2C_Timeout);
					uint8_t a = (dataBuf[1] >> 2) & 3;
					uint8_t b = ((dataBuf[1] & 3) << 5) | (dataBuf[2] >> 3);
					my_printf("Reading from 0x%02X\r\nGot %d 0x%02X 0x%02X 0x%02X %u 0x%02X\r\n", d >> 1, res, dataBuf[0], dataBuf[1], dataBuf[2], a, b);
					if (res == HAL_OK && dataBuf[0] == 0x5A)
					{
						for (uint8_t a = 0; a < 0x14; a++)
						{
							res = read_cell(&hi2c3, d, a, dataBuf, I2C_Timeout);
							my_printf(" 0x%02X : 0x%02X 0x%02X 0x%02X %d\r\n", a, dataBuf[0], dataBuf[1], dataBuf[2], res);
							if (res != HAL_OK || dataBuf[0] != 0x5A)
								break;
						}
					}
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
					my_printf("# ");
				}
				else if (strncmp((char *) input, "probe", cmdLen) == 0)
				{
					uint8_t d = LCdev[0];
					my_printf("\nProbing twice\r\n");
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_SET);
					HAL_Delay(10);
					for (unsigned int i = 0; i < 4; i++)
					{
						memset(dataBuf,0,4);
						res = HAL_I2C_Master_Receive(&hi2c3, d | 1, dataBuf, 4, I2C_Timeout);
						my_printf("Read %d at 0x%02X (%d) :", i, d >> 1, res);
						int status = dataBuf[0] >> 6;
						int w = ((dataBuf[0] & 0x3f) << 8) | dataBuf[1];
						int t = (dataBuf[2] << 3) | (dataBuf[3] >> 5);
						my_printf(" status %u - weight %u temperature %u\r\n", status, w, t);
						osDelay(1000);
					}
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
					osDelay(50);
					res = powerup_and_read(&hi2c3, d, dataBuf, I2C_Timeout);
					uint8_t a = (dataBuf[1] >> 2) & 3;
					uint8_t b = ((dataBuf[1] & 3) << 5) | (dataBuf[2] >> 3);
					my_printf("Got %d 0x%02X 0x%02X 0x%02X %u 0x%02X\r\n", res, dataBuf[0], dataBuf[1], dataBuf[2], a, b);
					if (res == HAL_OK && dataBuf[0] == 0x5A)
					{
						dataBuf[0] = 0x80;
						dataBuf[1] = 0;
						dataBuf[2] = 0;
						res = HAL_I2C_Master_Transmit(&hi2c3, d, dataBuf, 3, I2C_Timeout);
						my_printf("HAL res after send exit command mode = %d\r\n", res);
						for (unsigned int i = 0; i < 4; i++)
						{
							memset(dataBuf,0,4);
							res = HAL_I2C_Master_Receive(&hi2c3, d | 1, dataBuf, 4, I2C_Timeout);
							my_printf("Read %d at 0x%02X (%d) :", i, d >> 1, res);
							int status = dataBuf[0] >> 6;
							int w = ((dataBuf[0] & 0x3f) << 8) | dataBuf[1];
							int t = (dataBuf[2] << 3) | (dataBuf[3] >> 5);
							my_printf(" status %u - weight %u temperature %u\r\n", status, w, t);
							osDelay(1000);
						}
					}
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
					my_printf("# ");
				}
				else if (strncmp((char *) input, "ID", cmdLen) == 0)
				{
					/* does not seem to work on my device... */
					/*
					 * The Device ID is read-only, hard-wired in the device and can be accessed as follows:
					 * 1. START condition
					 * 2. The master sends the Reserved Device ID I2C-bus address followed by the R/W bit
					 *    set to ‘0’ (write): ‘1111 1000’.
					 * 3. The master sends the I2C-bus slave address of the slave device it must identify. The
					 *    LSB is a ‘Don’t care’ value. Only one device must acknowledge this byte (the one that
					 *    has the I2C-bus slave address).
					 * 4. The master sends a Re-START condition.
					 *    Remark: A STOP condition followed by a START condition resets the slave state
					 *    machine and the Device ID Read cannot be performed. Also, a STOP condition or a
					 *    Re-START condition followed by an access to another slave device resets the slave
					 *    state machine and the Device ID Read cannot be performed.
					 * 5. The master sends the Reserved Device ID I2C-bus address followed by the R/W bit
					 *    set to ‘1’ (read): ‘1111 1001’.
					 * 6. The Device ID Read can be done, starting with the 12 manufacturer bits (first byte +
					 *    four MSBs of the second byte), followed by the nine part identification bits (four LSBs
					 *    of the second byte + five MSBs of the third byte), and then the three die revision bits
					 *    (three LSBs of the third byte).
					 * 7. The master ends the reading sequence by NACKing the last byte, thus resetting the
					 *    slave device state machine and allowing the master to send the STOP condition.
					 */
					my_printf("\nGetting Device ID\r\n");
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_SET);
					HAL_Delay(10);
					for (unsigned int i = 0; i < 5; i++)
					{
						int d = address[i] << 1;
						memset(dataBuf,0,3);
						res = HAL_I2C_Mem_Read(&hi2c3, 0xF8, d, 1, dataBuf, 3, I2C_Timeout);
						my_printf("%u - 0x%02X : 0x%02X 0x%02X 0x%02X %d\r\n", i, d >> 1, dataBuf[0], dataBuf[1], dataBuf[2], res);
						if (res == HAL_OK)
						{
							int m = (dataBuf[0] << 4) | (dataBuf[1] >> 4);
							int p = ((int) dataBuf[1] << 5) | (dataBuf[2] >> 3);
							my_printf("  Manufacturer %u (0x%03X) Part %u (0x%03X) Die revision %u\r\n", m, m, p, p, dataBuf[2] & 0x7);
						}
					}
					HAL_GPIO_WritePin(GPIOC, Sensor_PWR_Pin, GPIO_PIN_RESET);
					my_printf("# ");
				}
				else
					my_printf("Unknown command\r\n# ");
			}
		}
		osDelay(50);
	}
#if 0
	dataBuf[0] = 0x80;
	dataBuf[1] = 0;
	dataBuf[2] = 0;
	HAL_I2C_Master_Transmit(&hi2c3, dev, dataBuf, 3, I2C_Timeout);
#endif
	/* Infinite loop */
	for(;;)
	{
		hb += 1;
		if (hb > 20)
		{
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); // toggle green led as heartbeat
			hb = 0;
		}
		int len = UART_Receive(input + inLen, u1rx, &huart1, &u1cc, 64);
		if (len > 0)
		{
			my_printf("%.*s", len, input + inLen);
			inLen += len;
			if (input[inLen - 1] == '\r')
			{
				int cmdLen = inLen - 1;
				my_printf("\nReceived command '%.*s'\r\n# ", cmdLen, input);
				inLen = 0;
				if (strncmp((char *) input, "UART", cmdLen) == 0)
					my_printf("u1rc = %u u1hrc = %u u1tc = %u u1htc = %u u1ec = %u u1ic = %u u1cc = %u\r\n# ", u1rc, u1hrc, u1tc, u1htc, u1ec, u1ic, u1cc);
			}
		}
		osDelay(50);
	}
  /* USER CODE END StartTaskI2C */
}

/* USER CODE BEGIN Header_readI2C */
/**
* @brief Function implementing the readI2CTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readI2C */
void readI2C(void const * argument)
{
  /* USER CODE BEGIN readI2C */
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	/* Infinite loop */
	for(;;)
	{
		if (LCcnt > 0)
		{
			// all on the same bus... need to read sequentially
			// maybe reorganize to send update request followed by actual read of data
			//  currently alternates between stale and valid data
			for (unsigned int i = 0; i < LCcnt; i++)
			{
				memset(dataBuf, 0, 4);
				res = HAL_I2C_Master_Receive(&hi2c3, LCdev[i] | 1, dataBuf, 4, I2C_Timeout);
				if (res != HAL_OK)
					errs += 1;
				else
				{
					uint8_t status = (dataBuf[0] >> 6) & 0x3;
					if (status == 0)
					{
						counts += 1;
						memcpy(bridgeValue + i * 4, dataBuf, 4);
					} else if (status == 2)
						stale += 1;
					else
						bridgeValue[3] = status;
				}
			}
			osDelay(50);
		}
		else
			osDelay(1000);
	}
  /* USER CODE END readI2C */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
