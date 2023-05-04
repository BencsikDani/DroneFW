/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "MPU9250.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 6 // My TX only has 6 channels, no point in polling the rest

#define THROTTLE_CHANNEL 3
#define YAW_CHANNEL 4
#define PITCH_CHANNEL 2
#define ROLL_CHANNEL 1

#define MPU9250_SPI hspi2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

osThreadId TaskSensorDataHandle;
osThreadId TaskControllerHandle;
osThreadId TaskRemoteHandle;
osThreadId TaskMotorHandle;
osThreadId TaskPowerHandle;
osThreadId TaskDiagnosticsHandle;
osSemaphoreId BinarySemHandle;
/* USER CODE BEGIN PV */

volatile int speed = 50;
volatile int16_t AccData[3] = { 0 };
volatile float TempData = 0;
volatile int16_t GyroData[3] = { 0 };
volatile int16_t MagData[3] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
void StartTaskSensorData(void const * argument);
void StartTaskController(void const * argument);
void StartTaskRemote(void const * argument);
void StartTaskMotor(void const * argument);
void StartTaskPower(void const * argument);
void StartTaskDiagnostics(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	MPU9250_Init();

	TIM3->CCR1 = (uint32_t) speed;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	HAL_GPIO_WritePin(IMU_CSBM_GPIO_Port, IMU_CSBM_Pin, GPIO_PIN_SET);
	MPU9250_Calibrate();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem */
  osSemaphoreDef(BinarySem);
  BinarySemHandle = osSemaphoreCreate(osSemaphore(BinarySem), 1);

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
  /* definition and creation of TaskSensorData */
  osThreadDef(TaskSensorData, StartTaskSensorData, osPriorityRealtime, 0, 512);
  TaskSensorDataHandle = osThreadCreate(osThread(TaskSensorData), NULL);

  /* definition and creation of TaskController */
  osThreadDef(TaskController, StartTaskController, osPriorityHigh, 0, 128);
  TaskControllerHandle = osThreadCreate(osThread(TaskController), NULL);

  /* definition and creation of TaskRemote */
  osThreadDef(TaskRemote, StartTaskRemote, osPriorityAboveNormal, 0, 512);
  TaskRemoteHandle = osThreadCreate(osThread(TaskRemote), NULL);

  /* definition and creation of TaskMotor */
  osThreadDef(TaskMotor, StartTaskMotor, osPriorityNormal, 0, 128);
  TaskMotorHandle = osThreadCreate(osThread(TaskMotor), NULL);

  /* definition and creation of TaskPower */
  osThreadDef(TaskPower, StartTaskPower, osPriorityBelowNormal, 0, 128);
  TaskPowerHandle = osThreadCreate(osThread(TaskPower), NULL);

  /* definition and creation of TaskDiagnostics */
  osThreadDef(TaskDiagnostics, StartTaskDiagnostics, osPriorityLow, 0, 1024);
  TaskDiagnosticsHandle = osThreadCreate(osThread(TaskDiagnostics), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	osSemaphoreRelease(BinarySemHandle);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 320-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.Mode = UART_MODE_RX;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_CSIMU_Pin|IMU_CSBM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_CSIMU_Pin IMU_CSBM_Pin */
  GPIO_InitStruct.Pin = IMU_CSIMU_Pin|IMU_CSBM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskSensorData */
/**
 * @brief  Function implementing the TaskSensorData thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskSensorData */
void StartTaskSensorData(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	while (1)
	{
		if (osSemaphoreWait(BinarySemHandle, osWaitForever) == osOK)
		{
			MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);
			osSemaphoreRelease(BinarySemHandle);
		}

		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskController */
/**
 * @brief Function implementing the TaskController thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskController */
void StartTaskController(void const * argument)
{
  /* USER CODE BEGIN StartTaskController */
	/* Infinite loop */
	while (1)
	{
		osDelay(10);
	}
  /* USER CODE END StartTaskController */
}

/* USER CODE BEGIN Header_StartTaskRemote */
/**
 * @brief Function implementing the TaskRemote thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskRemote */
void StartTaskRemote(void const * argument)
{
  /* USER CODE BEGIN StartTaskRemote */
	static uint8_t ibusIndex = 0;	// Current position in the ibus packet
	static uint8_t ibusData[IBUS_BUFFSIZE] = { 0 };	// Ibus packet buffer
	static uint16_t rcValue[IBUS_MAXCHANNELS];	// Output values of the channels (1000 ... 2000)

	/* Infinite loop */
	while (1)
	{
		uint8_t val = 0;
		HAL_UART_Receive(&huart2, &val, 1, HAL_MAX_DELAY);		// Read one bite at a time

		if (ibusIndex == IBUS_BUFFSIZE)
		{
			ibusIndex = 0;	// Reset the index variable

			// And cycle through the raw data and convert it to actual integer values
			// ibus pattern example:
			// i=0  1     2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21  22 23 24  25  26 27  28 28  30 31
			//   20 40    DB 5  DC 5  54 5  DC 5  E8 3  D0 7  D2 5  E8 3  DC 5  DC 5   DC 5   DC 5   DC 5   DC 5   DA F3
			// | Header | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 | CH9 | CH10 | CH11 | CH12 | CH13 | CH14 | Checksum |
			for (int i = 0; i < IBUS_MAXCHANNELS; i++)
				rcValue[i] = (ibusData[3 + 2*i] << 8) + ibusData[2 + 2*i];

			// Setting the speed
			if (osSemaphoreWait(BinarySemHandle, osWaitForever) == osOK)
			{
				if (AccData[2] > 160)
					speed = rcValue[2] / 20;
				else
					speed = 50;

				osSemaphoreRelease(BinarySemHandle);
			}

			// Setting PWM speed
			TIM3->CCR1 = (uint32_t) speed;

			osDelay(10);
		}
		// If we are just getting the header bytes or the actual data
		else if ((ibusIndex == 0 && val == 0x20) || (ibusIndex == 1 && val == 0x40) || (ibusIndex > 1))
		{
			ibusData[ibusIndex] = val;
			ibusIndex++;
		}
	}
  /* USER CODE END StartTaskRemote */
}

/* USER CODE BEGIN Header_StartTaskMotor */
/**
 * @brief Function implementing the TaskMotor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskMotor */
void StartTaskMotor(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotor */
	/* Infinite loop */
	while (1)
	{
		osDelay(10);
	}
  /* USER CODE END StartTaskMotor */
}

/* USER CODE BEGIN Header_StartTaskPower */
/**
 * @brief Function implementing the TaskPower thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskPower */
void StartTaskPower(void const * argument)
{
  /* USER CODE BEGIN StartTaskPower */
	/* Infinite loop */
	while (1)
	{
		osDelay(10);
	}
  /* USER CODE END StartTaskPower */
}

/* USER CODE BEGIN Header_StartTaskDiagnostics */
/**
* @brief Function implementing the TaskDiagnostics thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDiagnostics */
void StartTaskDiagnostics(void const * argument)
{
  /* USER CODE BEGIN StartTaskDiagnostics */
	char str[100];

	/* Infinite loop */
	while (1)
	{
		if (osSemaphoreWait(BinarySemHandle, osWaitForever) == osOK)
		{
			//HAL_UART_Transmit(&huart5, "DEB2\r\n", strlen("DEB2\r\n"), HAL_MAX_DELAY);

			// Setting up UART log info
			sprintf(str,
					"Speed: %d\r\nTemp: %.4f\r\nAcc:  %5d ; %5d ; %5d\r\nGyro: %5d ; %5d ; %5d\r\nMagn: %5d ; %5d ; %5d\r\n\r\n",
					speed, TempData, AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2], MagData[0],
					MagData[1], MagData[2]);

			osSemaphoreRelease(BinarySemHandle);
		}

		// Sending UART log info
		HAL_UART_Transmit(&huart5, str, strlen(str), HAL_MAX_DELAY);

		osDelay(100);
	}
  /* USER CODE END StartTaskDiagnostics */
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