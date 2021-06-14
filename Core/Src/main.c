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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "stdio.h"
#include "math.h"
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId bpmTaskHandle;
osThreadId stepTaskHandle;
osThreadId displayTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define true 1
#define false 0
int bpm;                   // used to hold the pulse rate
int signal;                // holds the incoming raw data
int IBI = 600;             // holds the time between beats, must be seeded!
unsigned char pulse = false;     // true when pulse wave is high, false when it's low
unsigned char beat_detected = false;        // becomes true when stm finds a beat.
int rate[10];                    // array to hold last ten IBI values
unsigned long sampleCounter = 0;          // used to determine pulse timing
unsigned long lastBeatTime = 0;           // used to find IBI
int P =512;                      // used to find peak in pulse wave
int T = 512;                     // used to find trough in pulse wave
int thresh = 512;                // used to find instant moment of heart beat
int amp = 100;                   // used to hold amplitude of pulse waveform
int num;
unsigned char firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
unsigned char secondBeat = false;      // used to seed rate array so we startup with reasonable BPM


#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t acc_x_raw = 0;
int16_t acc_y_raw = 0;
int16_t acc_z_raw = 0;

int16_t gyro_x_raw = 0;
int16_t gyro_y_raw = 0;
int16_t gyro_z_raw = 0;

float ax, ay, az, gx, gy, gz;

float prev_vec;
float curr_vec;
float total_vec;
int step_count = 0;

char bpm_buf[10] = {0};
char step_buf[10] = {0};


void MPU6050_Init (void)
{
	uint8_t is_operating;
	uint8_t config_data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &is_operating, 1, 1000);

	if (is_operating == 0x68)  // 0x68 will be returned by the sensor if everything goes well
	{
		config_data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&config_data, 1, 1000);

		config_data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &config_data, 1, 1000);

		config_data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &config_data, 1, 1000);

		config_data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &config_data, 1, 1000);
	}

}


void MPU6050_Read_Acc (void)
{
	uint8_t data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 1000);

	acc_x_raw = (int16_t)(data[0] << 8 | data [1]);
	acc_y_raw = (int16_t)(data[2] << 8 | data [3]);
	acc_z_raw = (int16_t)(data[4] << 8 | data [5]);

	ax = acc_x_raw/16384.0;
	ay = acc_y_raw/16384.0;
	az = acc_z_raw/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, data, 6, 1000);

	gyro_x_raw = (int16_t)(data[0] << 8 | data [1]);
	gyro_y_raw = (int16_t)(data[2] << 8 | data [3]);
	gyro_z_raw = (int16_t)(data[4] << 8 | data [5]);

	gx = gyro_x_raw/131.0;
	gy = gyro_y_raw/131.0;
	gz = gyro_z_raw/131.0;
}

void accelometer_task_func()
{
	while(1)
	{
		MPU6050_Read_Acc();
		curr_vec = sqrt((acc_x_raw*acc_x_raw)+(acc_y_raw*acc_y_raw)+(acc_z_raw*acc_z_raw));
		total_vec = curr_vec - prev_vec;
		if(total_vec > 625)
			step_count += 1;
		prev_vec = curr_vec;
		osDelay(1000);
	}
}

void bpm_task_func()
{

	while(1)
	{
		if(beat_detected)
		{
			beat_detected = false;
			osDelay(20);
		}
	}
}

void display_task_func()
{

	while(1)
	{
		sprintf(bpm_buf, "%-5d", bpm);
		sprintf(step_buf, "%-5d", step_count);
		SSD1306_GotoXY(0, 40);
		SSD1306_Puts(bpm_buf, &Font_11x18, 1);
		SSD1306_GotoXY(70, 40);
		SSD1306_Puts(step_buf, &Font_11x18, 1);
		SSD1306_UpdateScreen();
		osDelay(1000);
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
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();
  SSD1306_Init();
  HAL_TIM_Base_Start_IT(&htim3);
	SSD1306_GotoXY(0, 0);
	SSD1306_Puts("BPM", &Font_11x18, 1);

	SSD1306_GotoXY(70, 0);
	SSD1306_Puts("STEP", &Font_11x18, 1);

	SSD1306_GotoXY(58, 0);
	SSD1306_Puts("|", &Font_11x18, 1);
	SSD1306_GotoXY(58, 18);
	SSD1306_Puts("|", &Font_11x18, 1);
	SSD1306_GotoXY(58, 36);
	SSD1306_Puts("|", &Font_11x18, 1);
	SSD1306_GotoXY(58, 54);
	SSD1306_Puts("|", &Font_11x18, 1);
	SSD1306_GotoXY(58, 72);
	SSD1306_Puts("|", &Font_11x18, 1);
	SSD1306_UpdateScreen();
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

  /* definition and creation of bpmTask */
  osThreadDef(bpmTask, bpm_task_func, osPriorityIdle, 0, 128);
  bpmTaskHandle = osThreadCreate(osThread(bpmTask), NULL);

  /* definition and creation of stepTask */
  osThreadDef(stepTask, accelometer_task_func, osPriorityNormal, 0, 128);
  stepTaskHandle = osThreadCreate(osThread(stepTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, display_task_func, osPriorityNormal, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	unsigned int runningTotal;
	if(htim->Instance==htim3.Instance)
	{
		signal=HAL_ADC_GetValue(&hadc2)>>2;					 // read the Pulse Sensor
		sampleCounter += 2;                         // keep track of the time in mS with this variable
		num = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
		HAL_ADC_Start(&hadc2);									//restart ADC conversion

		//  find the peak and trough of the pulse wave
		if(signal < thresh && num > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
			if (signal < T){                        // T is the trough
				T = signal;                         // keep track of lowest point in pulse wave
			}
		}

		if(signal > thresh && signal > P){          // thresh condition helps avoid noise
			P = signal;                             // P is the peak
		}                                        // keep track of highest point in pulse wave

		//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
		// signal surges up in value every time there is a pulse
		if (num > 250){                                   // avoid high frequency noise
			if ( (signal > thresh) && (pulse == false) && (num > (IBI/5)*3) ){
				pulse = true;                               // set the Pulse flag when we think there is a pulse
				IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
				lastBeatTime = sampleCounter;               // keep track of time for next pulse

				if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
					secondBeat = false;                  // clear secondBeat flag
					for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
						rate[i] = IBI;
					}
				}

				if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
					firstBeat = false;                   // clear firstBeat flag
					secondBeat = true;                   // set the second beat flag
					return;                              // IBI value is unreliable so discard it
				}


				// keep a running total of the last 10 IBI values
				runningTotal = 0;                  // clear the runningTotal variable

				for(int i=0; i<=8; i++){                // shift data in the rate array
					rate[i] = rate[i+1];                  // and drop the oldest IBI value
					runningTotal += rate[i];              // add up the 9 oldest IBI values
				}

				rate[9] = IBI;                          // add the latest IBI to the rate array
				runningTotal += rate[9];                // add the latest IBI to runningTotal
				runningTotal /= 10;                     // average the last 10 IBI values
				bpm = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
				beat_detected = true;                              // set beat_detected flag
			}
		}

		if (signal < thresh && pulse == true){   // when the values are going down, the beat is over
			pulse = false;                         // reset the Pulse flag so we can do it again
			amp = P - T;                           // get amplitude of the pulse wave
			thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
			P = thresh;                            // reset these for next time
			T = thresh;
		}

		if (num > 2500){                           // if 2.5 seconds go by without a beat
			thresh = 512;                          // set thresh default
			P = 512;                               // set P default
			T = 512;                               // set T default
			lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
			firstBeat = true;                      // set these to avoid noise
			secondBeat = false;                    // when we get the heartbeat back
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

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the bpmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the stepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
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
