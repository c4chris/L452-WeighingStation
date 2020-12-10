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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "app_display.h"
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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c4_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId WriteLineTaskHandle;
osThreadId ReadI2C1TaskHandle;
osThreadId ReadI2C2TaskHandle;
osThreadId ReadI2C3TaskHandle;
osThreadId ReadI2C4TaskHandle;
osThreadId ShowWeightTaskHandle;
osThreadId DebugTaskHandle;
osMessageQId LineQueueHandle;
/* USER CODE BEGIN PV */
volatile unsigned int u2rc;
volatile unsigned int u2hrc;
volatile unsigned int u2tc;
volatile unsigned int u2htc;
volatile unsigned int u2ec;
volatile unsigned int u2ic;
volatile unsigned int errs1, errs2, errs3, errs4;
volatile unsigned int counts1, counts2, counts3, counts4;
volatile unsigned int stale1, stale2, stale3, stale4;
volatile unsigned int badstatus1, badstatus2, badstatus3, badstatus4;
volatile uint8_t bridgeValue1[4], bridgeValue2[4], bridgeValue3[4], bridgeValue4[4];
unsigned char dbgBuf[256];
unsigned char input[64];
unsigned char u2tx[256];
unsigned char u2rx[64];
volatile uint8_t setZero1;
volatile uint8_t setZero2;
volatile uint8_t setZero3;
volatile uint8_t setZero4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
void StartDefaultTask(void const * argument);
void StartWriteLineTask(void const * argument);
void StartReadI2C1Task(void const * argument);
void StartReadI2C2Task(void const * argument);
void StartReadI2C3Task(void const * argument);
void StartReadI2C4Task(void const * argument);
void StartShowWeightTask(void const * argument);
void StartDebugTask(void const * argument);

/* USER CODE BEGIN PFP */
void my_printf(char *format, ...) _ATTRIBUTE ((__format__ (__printf__, 1, 2)));
HAL_StatusTypeDef read_cell(I2C_HandleTypeDef *h, uint8_t d, uint8_t a, uint8_t *dataBuf, const uint32_t I2C_Timeout);
HAL_StatusTypeDef powerup_and_read(I2C_HandleTypeDef *h, uint8_t d, uint8_t *dataBuf, const uint32_t I2C_Timeout);
HAL_StatusTypeDef write_word(I2C_HandleTypeDef *, uint8_t, uint8_t *, uint8_t, uint8_t, uint8_t, const uint32_t);
HAL_StatusTypeDef exit_command_mode(I2C_HandleTypeDef *, uint8_t, uint8_t *, const uint32_t);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
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

  /* Create the queue(s) */
  /* definition and creation of LineQueue */
  osMessageQDef(LineQueue, 16, uint32_t);
  LineQueueHandle = osMessageCreate(osMessageQ(LineQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of WriteLineTask */
  osThreadDef(WriteLineTask, StartWriteLineTask, osPriorityIdle, 0, 1024);
  WriteLineTaskHandle = osThreadCreate(osThread(WriteLineTask), NULL);

  /* definition and creation of ReadI2C1Task */
  osThreadDef(ReadI2C1Task, StartReadI2C1Task, osPriorityIdle, 0, 128);
  ReadI2C1TaskHandle = osThreadCreate(osThread(ReadI2C1Task), NULL);

  /* definition and creation of ReadI2C2Task */
  osThreadDef(ReadI2C2Task, StartReadI2C2Task, osPriorityIdle, 0, 128);
  ReadI2C2TaskHandle = osThreadCreate(osThread(ReadI2C2Task), NULL);

  /* definition and creation of ReadI2C3Task */
  osThreadDef(ReadI2C3Task, StartReadI2C3Task, osPriorityIdle, 0, 128);
  ReadI2C3TaskHandle = osThreadCreate(osThread(ReadI2C3Task), NULL);

  /* definition and creation of ReadI2C4Task */
  osThreadDef(ReadI2C4Task, StartReadI2C4Task, osPriorityIdle, 0, 128);
  ReadI2C4TaskHandle = osThreadCreate(osThread(ReadI2C4Task), NULL);

  /* definition and creation of ShowWeightTask */
  osThreadDef(ShowWeightTask, StartShowWeightTask, osPriorityIdle, 0, 128);
  ShowWeightTaskHandle = osThreadCreate(osThread(ShowWeightTask), NULL);

  /* definition and creation of DebugTask */
  osThreadDef(DebugTask, StartDebugTask, osPriorityIdle, 0, 1024);
  DebugTaskHandle = osThreadCreate(osThread(DebugTask), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00702991;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hi2c3.Init.Timing = 0x00702991;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
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
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00702991;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);

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
  HAL_GPIO_WritePin(DISP_NRESET_GPIO_Port, DISP_NRESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_DCX_GPIO_Port, SPI1_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_NCS_Pin|SPI2_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DISP_TE_Pin */
  GPIO_InitStruct.Pin = DISP_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DISP_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DISP_NRESET_Pin */
  GPIO_InitStruct.Pin = DISP_NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DISP_NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_CENTER_Pin JOY_LEFT_Pin JOY_DOWN_Pin JOY_RIGHT_Pin
                           JOY_UP_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin|JOY_LEFT_Pin|JOY_DOWN_Pin|JOY_RIGHT_Pin
                          |JOY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_DCX_Pin */
  GPIO_InitStruct.Pin = SPI1_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_DCX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_NCS_Pin SPI2_NCS_Pin */
  GPIO_InitStruct.Pin = SPI1_NCS_Pin|SPI2_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART2)
	{
		u2tc += 1;
		/* Notify the task that the transmission is complete. */
		// FIXME - should use a variable with the task currently waiting...
    vTaskNotifyGiveFromISR(DebugTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2htc += 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2rc += 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2hrc += 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
		u2ec += 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		setZero1 = 1;
		setZero2 = 1;
		setZero3 = 1;
		setZero4 = 1;
		//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hspi->Instance == SPI1)
	{
		/* Notify the task that the transmission is complete. */
		// FIXME - should use a variable with the task currently waiting...
    vTaskNotifyGiveFromISR(WriteLineTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
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
	res = HAL_UART_Transmit_DMA(&huart2, dbgBuf, len);
	ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
	if (res != HAL_OK || ulNotificationValue != 1)
	{
		/* Something went wrong during output... */
		//HAL_UART_DMAStop(&huart2);
		len = snprintf((char *) dbgBuf, 256, "Timeout waiting on the DMA completion for %u ticks - seems bad\r\n", 200);
		HAL_UART_Transmit(&huart2, dbgBuf, len, xMaxBlockTime);
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
  	// put some kind of heartbeat here
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartWriteLineTask */
/**
* @brief Function implementing the WriteLineTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWriteLineTask */
void StartWriteLineTask(void const * argument)
{
  /* USER CODE BEGIN StartWriteLineTask */
	uint32_t low1 = 950;
	uint32_t low2 = 950;
	uint32_t low3 = 950;
	uint32_t low4 = 950;
	uint8_t msg[50];
	unsigned int c1 = 0, c2 = 0, c3 = 0, c4 = 0;
  MX_DISPLAY_Init();
	BSP_LCD_SetBackColor(0);
  /* Infinite loop */
  for(;;)
  {
  	uint32_t c = osKernelSysTick() / pdMS_TO_TICKS(1000);
  	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  	BSP_LCD_SetFont(&Font24);
		sprintf((char *)msg,"WS %lu",c);
		BSP_LCD_DisplayStringAtLine(0, msg, CENTER_MODE);
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		sprintf((char *)msg,"%u %u %u %u",errs1,badstatus1,errs2,badstatus2);
		BSP_LCD_DisplayStringAtLine(3, msg, CENTER_MODE);
		sprintf((char *)msg,"%u %u %u %u",errs3,badstatus3,errs4,badstatus4);
		BSP_LCD_DisplayStringAtLine(4, msg, CENTER_MODE);
		if (c1 != counts1)
		{
			uint32_t weight = (bridgeValue1[0] & 0x3f) * 256 + bridgeValue1[1];
			//sprintf((char *)msg,"  raw : %ld  ", weight);
			//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 75, msg, CENTER_MODE);
			if (setZero1)
			{
				low1 = weight;
				setZero1 = 0;
			}
			if (weight < low1)
				weight = 0;
			else
				weight -= low1;
			weight *= 5000;
			weight /= 14000;
			sprintf((char *)msg,"1: %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
			BSP_LCD_SetFont(&Font24);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
			BSP_LCD_DisplayStringAtLine(4, msg, CENTER_MODE);
			uint32_t temp = (bridgeValue1[2] << 3) + (bridgeValue1[3] >> 5);
			temp *= 2000;
			temp /= 2048; // just a guess at this point...
			temp -= 500;
			sprintf((char *)msg,"  T: %2d.%01d C  ", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
			BSP_LCD_DisplayStringAtLine(5, msg, CENTER_MODE);
			c1 = counts1;
		}
		if (c2 != counts2)
		{
			uint32_t weight = (bridgeValue2[0] & 0x3f) * 256 + bridgeValue2[1];
			//sprintf((char *)msg,"  raw : %ld  ", weight);
			//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 75, msg, CENTER_MODE);
			if (setZero2)
			{
				low2 = weight;
				setZero2 = 0;
			}
			if (weight < low2)
				weight = 0;
			else
				weight -= low2;
			weight *= 5000;
			weight /= 14000;
			sprintf((char *)msg,"2: %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
			BSP_LCD_SetFont(&Font24);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTYELLOW);
			BSP_LCD_DisplayStringAtLine(6, msg, CENTER_MODE);
			uint32_t temp = (bridgeValue2[2] << 3) + (bridgeValue2[3] >> 5);
			temp *= 2000;
			temp /= 2048; // just a guess at this point...
			temp -= 500;
			sprintf((char *)msg,"  T: %2d.%01d C  ", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
			BSP_LCD_DisplayStringAtLine(7, msg, CENTER_MODE);
			c2 = counts2;
		}
		if (c3 != counts3)
		{
			uint32_t weight = (bridgeValue3[0] & 0x3f) * 256 + bridgeValue3[1];
			//sprintf((char *)msg,"  raw : %ld  ", weight);
			//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 75, msg, CENTER_MODE);
			if (setZero3)
			{
				low3 = weight;
				setZero3 = 0;
			}
			if (weight < low3)
				weight = 0;
			else
				weight -= low3;
			weight *= 5000;
			weight /= 14000;
			sprintf((char *)msg,"3: %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
			BSP_LCD_SetFont(&Font24);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
			BSP_LCD_DisplayStringAtLine(8, msg, CENTER_MODE);
			uint32_t temp = (bridgeValue3[2] << 3) + (bridgeValue3[3] >> 5);
			temp *= 2000;
			temp /= 2048; // just a guess at this point...
			temp -= 500;
			sprintf((char *)msg,"  T: %2d.%01d C  ", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
			BSP_LCD_DisplayStringAtLine(9, msg, CENTER_MODE);
			c3 = counts3;
		}
		if (c4 != counts4)
		{
			uint32_t weight = (bridgeValue4[0] & 0x3f) * 256 + bridgeValue4[1];
			//sprintf((char *)msg,"  raw : %ld  ", weight);
			//BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 75, msg, CENTER_MODE);
			if (setZero4)
			{
				low4 = weight;
				setZero4 = 0;
			}
			if (weight < low4)
				weight = 0;
			else
				weight -= low4;
			weight *= 5000;
			weight /= 14000;
			sprintf((char *)msg,"4: %2d.%02d kg", (uint16_t)(weight / 100), (uint16_t)(weight % 100));
			BSP_LCD_SetFont(&Font24);
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
			BSP_LCD_DisplayStringAtLine(10, msg, CENTER_MODE);
			uint32_t temp = (bridgeValue4[2] << 3) + (bridgeValue4[3] >> 5);
			temp *= 2000;
			temp /= 2048; // just a guess at this point...
			temp -= 500;
			sprintf((char *)msg,"  T: %2d.%01d C  ", (uint16_t)(temp / 10), (uint16_t)(temp % 10));
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
			BSP_LCD_DisplayStringAtLine(11, msg, CENTER_MODE);
			c4 = counts4;
		}
    osDelay(333);
  }
  /* USER CODE END StartWriteLineTask */
}

/* USER CODE BEGIN Header_StartReadI2C1Task */
/**
* @brief Function implementing the ReadI2C1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadI2C1Task */
void StartReadI2C1Task(void const * argument)
{
  /* USER CODE BEGIN StartReadI2C1Task */
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	/* Infinite loop */
	for(;;)
	{
		memset(dataBuf, 0, 4);
		res = HAL_I2C_Master_Receive(&hi2c1, (0x28 << 1) | 1, dataBuf, 4, I2C_Timeout);
		if (res != HAL_OK)
		{
			errs1 += 1;
			HAL_I2C_DeInit(&hi2c1);
			osDelay(500);
			HAL_I2C_Init(&hi2c1);
			osDelay(500);
		}
		else
		{
			uint8_t status = (dataBuf[0] >> 6) & 0x3;
			if (status == 0)
			{
				counts1 += 1;
				memcpy((void *) bridgeValue1, dataBuf, 4);
			} else if (status == 2)
				stale1 += 1;
			else
				badstatus1 += 1;
		}
		osDelay(50);
	}
  /* USER CODE END StartReadI2C1Task */
}

/* USER CODE BEGIN Header_StartReadI2C2Task */
/**
* @brief Function implementing the ReadI2C2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadI2C2Task */
void StartReadI2C2Task(void const * argument)
{
  /* USER CODE BEGIN StartReadI2C2Task */
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	/* Infinite loop */
	for(;;)
	{
		memset(dataBuf, 0, 4);
		res = HAL_I2C_Master_Receive(&hi2c2, (0x28 << 1) | 1, dataBuf, 4, I2C_Timeout);
		if (res != HAL_OK)
		{
			errs2 += 1;
			HAL_I2C_DeInit(&hi2c3);
			osDelay(500);
			HAL_I2C_Init(&hi2c3);
			osDelay(500);
		}
		else
		{
			uint8_t status = (dataBuf[0] >> 6) & 0x3;
			if (status == 0)
			{
				counts2 += 1;
				memcpy((void *) bridgeValue2, dataBuf, 4);
			} else if (status == 2)
				stale2 += 1;
			else
				badstatus2 += 1;
		}
		osDelay(50);
	}
  /* USER CODE END StartReadI2C2Task */
}

/* USER CODE BEGIN Header_StartReadI2C3Task */
/**
* @brief Function implementing the ReadI2C3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadI2C3Task */
void StartReadI2C3Task(void const * argument)
{
  /* USER CODE BEGIN StartReadI2C3Task */
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	/* Infinite loop */
	for(;;)
	{
		memset(dataBuf, 0, 4);
		res = HAL_I2C_Master_Receive(&hi2c3, (0x28 << 1) | 1, dataBuf, 4, I2C_Timeout);
		if (res != HAL_OK)
		{
			errs3 += 1;
			HAL_I2C_DeInit(&hi2c3);
			osDelay(500);
			HAL_I2C_Init(&hi2c3);
			osDelay(500);
		}
		else
		{
			uint8_t status = (dataBuf[0] >> 6) & 0x3;
			if (status == 0)
			{
				counts3 += 1;
				memcpy((void *) bridgeValue3, dataBuf, 4);
			} else if (status == 2)
				stale3 += 1;
			else
				badstatus3 += 1;
		}
		osDelay(50);
	}
  /* USER CODE END StartReadI2C3Task */
}

/* USER CODE BEGIN Header_StartReadI2C4Task */
/**
* @brief Function implementing the ReadI2C4Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadI2C4Task */
void StartReadI2C4Task(void const * argument)
{
  /* USER CODE BEGIN StartReadI2C4Task */
	const uint32_t I2C_Timeout = I2Cx_TIMEOUT_MAX;
	HAL_StatusTypeDef res;
	uint8_t dataBuf[4];
	/* Infinite loop */
	for(;;)
	{
		memset(dataBuf, 0, 4);
		res = HAL_I2C_Master_Receive(&hi2c4, (0x28 << 1) | 1, dataBuf, 4, I2C_Timeout);
		if (res != HAL_OK)
		{
			errs4 += 1;
			HAL_I2C_DeInit(&hi2c4);
			osDelay(500);
			HAL_I2C_Init(&hi2c4);
			osDelay(500);
		}
		else
		{
			uint8_t status = (dataBuf[0] >> 6) & 0x3;
			if (status == 0)
			{
				counts4 += 1;
				memcpy((void *) bridgeValue4, dataBuf, 4);
			} else if (status == 2)
				stale4 += 1;
			else
				badstatus4 += 1;
		}
		osDelay(50);
	}
  /* USER CODE END StartReadI2C4Task */
}

/* USER CODE BEGIN Header_StartShowWeightTask */
/**
* @brief Function implementing the ShowWeightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShowWeightTask */
void StartShowWeightTask(void const * argument)
{
  /* USER CODE BEGIN StartShowWeightTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
  }
  /* USER CODE END StartShowWeightTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
* @brief Function implementing the DebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void const * argument)
{
  /* USER CODE BEGIN StartDebugTask */
	HAL_UART_Receive_DMA(&huart2, u2rx, 64);
	unsigned int u2cc = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
	//HAL_StatusTypeDef res;
	int inLen = 0;
	my_printf("\r\nStarting Run on %s\r\n# ", tskKERNEL_VERSION_NUMBER);
	/* Infinite loop */
	for(;;)
	{
		int len = UART_Receive(input + inLen, u2rx, &huart2, &u2cc, 64);
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
					my_printf("u2rc = %u u2hrc = %u u2tc = %u u2htc = %u u2ec = %u u2ic = %u u2cc = %u\r\n# ", u2rc, u2hrc, u2tc, u2htc, u2ec, u2ic, u2cc);
			}
		}
		osDelay(50);
	}
  /* USER CODE END StartDebugTask */
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
