
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include "WS2812_Lib.h"
#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_accelerometer.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId controlTaskHandle;
osThreadId commTaskHandle;
osThreadId idleTaskHandle;
osThreadId ledTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x
#define PI     3.14159

#define COMPASS_X_MAX 702
#define COMPASS_X_MIN -358
#define COMPASS_Y_MAX 293
#define COMPASS_Y_MIN -829
#define COMPASS_Z_MAX 845
#define COMPASS_Z_MIN -282

static volatile uint32_t ADC_Buf[6];
static volatile uint32_t ADC_Values[6];

static uint16_t mot1Cntr, mot1CntrPrev = 0; //L1
static uint16_t mot2Cntr, mot2CntrPrev = 0; //L2
static uint16_t mot3Cntr, mot3CntrPrev = 0; //L3
static uint16_t mot4Cntr, mot4CntrPrev = 0; //R1
static uint16_t mot5Cntr, mot5CntrPrev = 0; //R2
static uint16_t mot6Cntr, mot6CntrPrev = 0; //R3
static volatile uint8_t mot1Dir = 0; //L1
static volatile uint8_t mot2Dir = 0; //L2
static volatile uint8_t mot3Dir = 0; //L3
static volatile uint8_t mot4Dir = 0; //R1
static volatile uint8_t mot5Dir = 0; //R2
static volatile uint8_t mot6Dir = 0; //R3

static volatile uint32_t sen1Cntr, sen1Start, sen1End, sen1Length = 0; //Sensors
static volatile uint32_t sen2Cntr, sen2Start, sen2End, sen2Length = 0; //Sensors
static volatile uint32_t sen3Cntr, sen3Start, sen3End, sen3Length = 0; //Sensors
static volatile uint32_t sen4Cntr, sen4Start, sen4End, sen4Length = 0; //Sensors
static volatile float sen1Distance = 0;
static volatile float sen2Distance = 0;
static volatile float sen3Distance = 0;
static volatile float sen4Distance = 0;

static volatile float batteryVoltage1 = 0;
static volatile float batteryVoltage2 = 0;

static volatile uint16_t sen1AnalogValue = 0;
static volatile uint16_t sen2AnalogValue = 0;
static volatile uint16_t sen3AnalogValue = 0;
static volatile uint16_t sen4AnalogValue = 0;

static volatile uint32_t internalCounter = 0;

static volatile float roll,pitch,heading = 0;
static volatile float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f;
static volatile float accX,accY,accZ = 0;
static volatile float accXFiltered,accYFiltered,accZFiltered = 0;
static volatile float compassX,compassY,compassZ = 0;

static volatile float wheelL1Odometer_m, wheelL1Odometer_m_prev = 0;
static volatile float wheelL2Odometer_m, wheelL2Odometer_m_prev = 0;
static volatile float wheelL3Odometer_m, wheelL3Odometer_m_prev = 0;
static volatile float wheelR1Odometer_m, wheelR1Odometer_m_prev = 0;
static volatile float wheelR2Odometer_m, wheelR2Odometer_m_prev = 0;
static volatile float wheelR3Odometer_m, wheelR3Odometer_m_prev = 0;
static volatile float wheelL1Angle_deg = 0;
static volatile float wheelL2Angle_deg = 0;
static volatile float wheelL3Angle_deg = 0;
static volatile float wheelR1Angle_deg = 0;
static volatile float wheelR2Angle_deg = 0;
static volatile float wheelR3Angle_deg = 0;

static volatile float wheelL1RawVelocity_mps, wheelL1FiltVelocity_mps = 0;
static volatile float wheelL2RawVelocity_mps, wheelL2FiltVelocity_mps = 0;
static volatile float wheelL3RawVelocity_mps, wheelL3FiltVelocity_mps = 0;
static volatile float wheelR1RawVelocity_mps, wheelR1FiltVelocity_mps = 0;
static volatile float wheelR2RawVelocity_mps, wheelR2FiltVelocity_mps = 0;
static volatile float wheelR3RawVelocity_mps, wheelR3FiltVelocity_mps = 0;

static volatile float wheelSize_m = 0.20; // 20cm circumference
static volatile uint16_t encoderSignalsPerRound = 360; // 3 counts per motor round and a 120:1 gearbox
static volatile uint16_t speedSensingTaskCycle_ms = 10; // 10ms task cycle time

static volatile float referenceSpeedLeft = 0;
static volatile float referenceSpeedRight = 0;

static volatile uint32_t timeStamp =0;
static volatile uint32_t timeOutGuard =0;
static uint8_t receiveBuffer[3]={0};
static volatile float referenceSpeed = 0;
static volatile float referenceAngle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void const * argument);
void StartSensorTask(void const * argument);
void StartControlTask(void const * argument);
void StartCommTask(void const * argument);
void StartIdleTask(void const * argument);
void StartLedTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  
  if(hadc->Instance==ADC1){
    ADC_Values[0] = ADC_Buf[0];
    ADC_Values[1] = ADC_Buf[1];
    ADC_Values[2] = ADC_Buf[2];
    ADC_Values[3] = ADC_Buf[3];
    ADC_Values[4] = ADC_Buf[4];
    ADC_Values[5] = ADC_Buf[5];
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  // 6*PWM signals for the 6 DC motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_3);
  // 6*PWM signals for the 6 servos
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim5, TIM_CHANNEL_2);
  // 1*PWM signal for buzzer
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim9, TIM_CHANNEL_1);
  //Start internal timer for 10us time measurement
  HAL_TIM_Base_Start(&htim10);
  
  // Test code START
  TIM1->CCR1 = 10*4800/100; //LEFT1
  TIM1->CCR2 = 10*4800/100; //LEFT2
  TIM1->CCR3 = 10*4800/100; //LEFT3
  TIM1->CCR4 = 10*4800/100; //RIGHT1
  TIM2->CCR1 = 10*4800/100; //RIGHT2
  TIM2->CCR3 = 10*4800/100; //RIGHT3
  
  TIM3->CCR1 = 5*20000/100; //SERVO1 - L1
  TIM3->CCR2 = 5*20000/100; //SERVO2 - L2
  TIM3->CCR3 = 5*20000/100; //SERVO3 - L3
  TIM3->CCR4 = 5*20000/100; //SERVO4 - L4
  TIM5->CCR1 = 5*20000/100; //SERVO5 - L5
  TIM5->CCR2 = 5*20000/100; //SERVO6 - L6
  
  TIM2->CCR3 = 40*4800/100; //Buzzer
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //LEFT1 FW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); //LEFT1 RW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //LEFT2 FW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); //LEFT2 RW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); //LEFT3 FW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); //LEFT3 RW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_RESET); //RIGHT1 FW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,  GPIO_PIN_RESET); //RIGHT1 RW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_RESET); //RIGHT2 FW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //RIGHT2 RW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //RIGHT3 FW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //RIGHT3 RW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  GPIO_PIN_RESET); //SEN1 OUT
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,  GPIO_PIN_RESET); //SEN2 OUT
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,  GPIO_PIN_RESET); //SEN3 OUT
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,  GPIO_PIN_RESET); //SEN4 OUT
  // Test code END
  
  BSP_ACCELERO_Init();
  MAGNET_Init();
  
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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityHigh, 0, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of commTask */
  osThreadDef(commTask, StartCommTask, osPriorityAboveNormal, 0, 128);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  /* definition and creation of idleTask */
  osThreadDef(idleTask, StartIdleTask, osPriorityLow, 0, 128);
  idleTaskHandle = osThreadCreate(osThread(idleTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, StartLedTask, osPriorityNormal, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 119;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 95;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4800;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 960;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB_OTG_FS init function */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEFT2_FW_Pin|LEFT2_RW_Pin|SEN3_OUT_Pin|SEN4_OUT_Pin 
                          |LEFT3_FW_Pin|LEFT3_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEN1_OUT_Pin|SEN2_OUT_Pin|RIGHT1_FW_Pin|RIGHT1_RW_Pin 
                          |RIGHT2_FW_Pin|RIGHT2_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEFT1_FW_GPIO_Port, LEFT1_FW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LEFT1_RW_Pin|RIGHT3_FW_Pin|RIGHT3_RW_Pin|LD5_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEFT2_FW_Pin LEFT2_RW_Pin SEN3_OUT_Pin SEN4_OUT_Pin 
                           LEFT3_FW_Pin LEFT3_RW_Pin */
  GPIO_InitStruct.Pin = LEFT2_FW_Pin|LEFT2_RW_Pin|SEN3_OUT_Pin|SEN4_OUT_Pin 
                          |LEFT3_FW_Pin|LEFT3_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEN1_OUT_Pin SEN2_OUT_Pin RIGHT1_FW_Pin RIGHT1_RW_Pin 
                           RIGHT2_FW_Pin RIGHT2_RW_Pin */
  GPIO_InitStruct.Pin = SEN1_OUT_Pin|SEN2_OUT_Pin|RIGHT1_FW_Pin|RIGHT1_RW_Pin 
                          |RIGHT2_FW_Pin|RIGHT2_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT1_ENC_B_Pin RIGHT2_ENC_B_Pin */
  GPIO_InitStruct.Pin = RIGHT1_ENC_B_Pin|RIGHT2_ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT1_ENC_B_Pin LEFT2_ENC_B_Pin LEFT3_ENC_B_Pin */
  GPIO_InitStruct.Pin = LEFT1_ENC_B_Pin|LEFT2_ENC_B_Pin|LEFT3_ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT1_FW_Pin */
  GPIO_InitStruct.Pin = LEFT1_FW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEFT1_FW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEN3_EXTI_Pin SEN4_EXTI_Pin SEN1_EXTI_Pin SEN2_EXTI_Pin */
  GPIO_InitStruct.Pin = SEN3_EXTI_Pin|SEN4_EXTI_Pin|SEN1_EXTI_Pin|SEN2_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT3_ENC_B_Pin */
  GPIO_InitStruct.Pin = RIGHT3_ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RIGHT3_ENC_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT1_RW_Pin RIGHT3_FW_Pin RIGHT3_RW_Pin LD5_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = LEFT1_RW_Pin|RIGHT3_FW_Pin|RIGHT3_RW_Pin|LD5_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT1_EXTI0_Pin LEFT2_EXTI1_Pin LEFT3_EXTI2_Pin RIGHT1_EXTI3_Pin 
                           RIGHT2_EXTI4_Pin RIGHT3_EXTI5_Pin */
  GPIO_InitStruct.Pin = LEFT1_EXTI0_Pin|LEFT2_EXTI1_Pin|LEFT3_EXTI2_Pin|RIGHT1_EXTI3_Pin 
                          |RIGHT2_EXTI4_Pin|RIGHT3_EXTI5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  switch ( GPIO_Pin ) {
  case GPIO_PIN_0:
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) mot1Cntr--;
    else mot1Cntr++;
    break;
  case GPIO_PIN_1:
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) mot2Cntr--;
    else mot2Cntr++;
    break;
  case GPIO_PIN_2:
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) mot3Cntr--;
    else mot3Cntr++;
    break;
  case GPIO_PIN_3:
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)) mot4Cntr--;
    else mot4Cntr++;
    break;
  case GPIO_PIN_4:
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)) mot5Cntr--;
    else mot5Cntr++;
    break;
  case GPIO_PIN_5:
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)) mot6Cntr--;
    else mot6Cntr++;
    break;
  case GPIO_PIN_6:
    sen1Cntr++;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6)) sen1Start = TIM10->CNT;
    else {
      sen1End = TIM10->CNT;
      if (sen1End > sen1Start) sen1Length = sen1End-sen1Start;
      else sen1Length = (sen1End+65535)-sen1Start;
    }
    break; 
  case GPIO_PIN_7:
    sen2Cntr++;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)) sen2Start = TIM10->CNT;
    else {
      sen2End = TIM10->CNT;
      if (sen2End > sen2Start) sen2Length = sen2End-sen2Start;
      else sen2Length = (sen2End+65535)-sen2Start;
    }
    break; 
  case GPIO_PIN_8:
    sen3Cntr++;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)) sen3Start = TIM10->CNT;
    else {
      sen3End = TIM10->CNT;
      if (sen3End > sen3Start) sen3Length = sen3End-sen3Start;
      else sen3Length = (sen3End+65535)-sen3Start;
    }
    break; 
  case GPIO_PIN_9:
    sen4Cntr++;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)) sen4Start = TIM10->CNT;
    else {
      sen4End = TIM10->CNT;
      if (sen4End > sen4Start) sen4Length = sen4End-sen4Start;
      else sen4Length = (sen4End+65535)-sen4Start;
    }
    break; 
  default:
    break;
  }
}

float filter2 (float avg, float input, float alpha) {
  avg = (alpha * input) + (1.0 - alpha) * avg;
  return avg;
}

void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

void saturateVolatileFloat(volatile float* i, float min, float max) {
  float val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

void calculateLRSpeeds(float r, float phi) {

  phi = phi * PI / 180.0;
  
  if ((phi >= 0) && (phi < PI / 2.0)) //1st quadrant
  {
      referenceSpeedLeft = r;
      referenceSpeedRight = -r * cosf(2 * phi);
  }
  else if ((phi >= PI / 2.0) && (phi < PI)) //2nd quadrant
  {
      referenceSpeedLeft = -r * cosf(2 * phi);
      referenceSpeedRight = r;
  }
  else if ((phi >= PI) && (phi < 3 * PI / 2.0)) //3rd quadrant
  {
      referenceSpeedLeft = -r;
      referenceSpeedRight = r * cosf(2 * phi);
  }
  else if ((phi >= 3 * PI / 2.0) && (phi <= 2 * PI)) //4th quadrant
  {
      referenceSpeedLeft = r * cosf(2 * phi);
      referenceSpeedRight = -r;
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

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint32_t cycleCounter = 0;
  uint32_t targetTick;
  float compassBuffer[3];
  float accelerometerBuffer[3] = {0};
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buf,6);
  /* Infinite loop */
  for(;;)
  {
    //test code START
    mot1Dir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    mot2Dir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    mot3Dir = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    mot4Dir = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
    mot5Dir = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);
    mot6Dir = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
    internalCounter = TIM10->CNT;
    //test code END
    osDelay(speedSensingTaskCycle_ms);
    cycleCounter++;
    sen1Distance = sen1Length * 0.34/2;
    sen2Distance = sen2Length * 0.34/2;
    sen3Distance = sen3Length * 0.34/2;
    sen4Distance = sen4Length * 0.34/2;
    batteryVoltage1 = ADC_Values[0]*0.005; // ToDo adjust gain
    batteryVoltage2 = ADC_Values[1]*0.005; // ToDo adjust gain
    sen1AnalogValue = ADC_Values[2];
    sen2AnalogValue = ADC_Values[3];
    sen3AnalogValue = ADC_Values[4];
    sen4AnalogValue = ADC_Values[5];
    
    if ((mot1Cntr - mot1CntrPrev) > 100){
      wheelL1Odometer_m += (mot1Cntr - 65535 - mot1CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL1Angle_deg += (mot1Cntr - 65535 - mot1CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else if ((mot1Cntr - mot1CntrPrev) < -100){
      wheelL1Odometer_m += (mot1Cntr + 65535 - mot1CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL1Angle_deg += (mot1Cntr + 65535 - mot1CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else {
      wheelL1Odometer_m += (mot1Cntr - mot1CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL1Angle_deg += (mot1Cntr - mot1CntrPrev) * 360 / encoderSignalsPerRound;
    }
    
    if ((mot2Cntr - mot2CntrPrev) > 100){
      wheelL2Odometer_m += (mot2Cntr - 65535 - mot2CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL2Angle_deg += (mot2Cntr - 65535 - mot2CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else if ((mot2Cntr - mot2CntrPrev) < -100){
      wheelL2Odometer_m += (mot2Cntr + 65535 - mot2CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL2Angle_deg += (mot2Cntr + 65535 - mot2CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else {
      wheelL2Odometer_m += (mot2Cntr - mot2CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL2Angle_deg += (mot2Cntr - mot2CntrPrev) * 360 / encoderSignalsPerRound;
    }
    
    if ((mot3Cntr - mot3CntrPrev) > 100){
      wheelL3Odometer_m += (mot3Cntr - 65535 - mot3CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL3Angle_deg += (mot3Cntr - 65535 - mot3CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else if ((mot3Cntr - mot3CntrPrev) < -100){
      wheelL3Odometer_m += (mot3Cntr + 65535 - mot3CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL3Angle_deg += (mot3Cntr + 65535 - mot3CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else {
      wheelL3Odometer_m += (mot3Cntr - mot3CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelL3Angle_deg += (mot3Cntr - mot3CntrPrev) * 360 / encoderSignalsPerRound;
    }
    
    if ((mot4Cntr - mot4CntrPrev) > 100){
      wheelR1Odometer_m += (mot4Cntr - 65535 - mot4CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR1Angle_deg += (mot4Cntr - 65535 - mot4CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else if ((mot4Cntr - mot4CntrPrev) < -100){
      wheelR1Odometer_m += (mot4Cntr + 65535 - mot4CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR1Angle_deg += (mot4Cntr + 65535 - mot4CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else {
      wheelR1Odometer_m += (mot4Cntr - mot4CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR1Angle_deg += (mot4Cntr - mot4CntrPrev) * 360 / encoderSignalsPerRound;
    }
    
    if ((mot5Cntr - mot5CntrPrev) > 100){
      wheelR2Odometer_m += (mot5Cntr - 65535 - mot5CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR2Angle_deg += (mot5Cntr - 65535 - mot5CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else if ((mot5Cntr - mot5CntrPrev) < -100){
      wheelR2Odometer_m += (mot5Cntr + 65535 - mot5CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR2Angle_deg += (mot5Cntr + 65535 - mot5CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else {
      wheelR2Odometer_m += (mot5Cntr - mot5CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR2Angle_deg += (mot5Cntr - mot5CntrPrev) * 360 / encoderSignalsPerRound;
    }
    
    if ((mot6Cntr - mot6CntrPrev) > 100){
      wheelR3Odometer_m += (mot6Cntr - 65535 - mot6CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR3Angle_deg += (mot6Cntr - 65535 - mot6CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else if ((mot6Cntr - mot6CntrPrev) < -100){
      wheelR3Odometer_m += (mot6Cntr + 65535 - mot6CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR3Angle_deg += (mot6Cntr + 65535 - mot6CntrPrev) * 360 / encoderSignalsPerRound;
    }
    else {
      wheelR3Odometer_m += (mot6Cntr - mot6CntrPrev) * wheelSize_m / encoderSignalsPerRound;
      wheelR3Angle_deg += (mot6Cntr - mot6CntrPrev) * 360 / encoderSignalsPerRound;
    }
    
    wheelL1RawVelocity_mps = (wheelL1Odometer_m - wheelL1Odometer_m_prev) * 1000.0 / speedSensingTaskCycle_ms;
    wheelL2RawVelocity_mps = (wheelL2Odometer_m - wheelL2Odometer_m_prev) * 1000.0 / speedSensingTaskCycle_ms;
    wheelL3RawVelocity_mps = (wheelL3Odometer_m - wheelL3Odometer_m_prev) * 1000.0 / speedSensingTaskCycle_ms;
    wheelR1RawVelocity_mps = (wheelR1Odometer_m - wheelR1Odometer_m_prev) * 1000.0 / speedSensingTaskCycle_ms;
    wheelR2RawVelocity_mps = (wheelR2Odometer_m - wheelR2Odometer_m_prev) * 1000.0 / speedSensingTaskCycle_ms;
    wheelR3RawVelocity_mps = (wheelR3Odometer_m - wheelR3Odometer_m_prev) * 1000.0 / speedSensingTaskCycle_ms;

    wheelL1FiltVelocity_mps = filter2(wheelL1FiltVelocity_mps, wheelL1RawVelocity_mps, 0.35);
    wheelL2FiltVelocity_mps = filter2(wheelL2FiltVelocity_mps, wheelL2RawVelocity_mps, 0.35);
    wheelL3FiltVelocity_mps = filter2(wheelL3FiltVelocity_mps, wheelL3RawVelocity_mps, 0.35);
    wheelR1FiltVelocity_mps = filter2(wheelR1FiltVelocity_mps, wheelR1RawVelocity_mps, 0.35);
    wheelR2FiltVelocity_mps = filter2(wheelR2FiltVelocity_mps, wheelR2RawVelocity_mps, 0.35);
    wheelR3FiltVelocity_mps = filter2(wheelR3FiltVelocity_mps, wheelR3RawVelocity_mps, 0.35);
    
    wheelL1Odometer_m_prev = wheelL1Odometer_m;
    wheelL2Odometer_m_prev = wheelL2Odometer_m;
    wheelL3Odometer_m_prev = wheelL3Odometer_m;
    wheelR1Odometer_m_prev = wheelR1Odometer_m;
    wheelR2Odometer_m_prev = wheelR2Odometer_m;
    wheelR3Odometer_m_prev = wheelR3Odometer_m;
    mot1CntrPrev = mot1Cntr;
    mot2CntrPrev = mot2Cntr;
    mot3CntrPrev = mot3Cntr;
    mot4CntrPrev = mot4Cntr;
    mot5CntrPrev = mot5Cntr;
    mot6CntrPrev = mot6Cntr;
    
    if (cycleCounter % 10 == 0){
      if (TIM10->CNT < 65534){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //SEN1 trigger
        targetTick = TIM10->CNT+1;
        while (TIM10->CNT <= targetTick);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //SEN1 trigger
      }
    }
    
    if (cycleCounter % 10 == 2){
      if (TIM10->CNT < 65534){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //SEN2 trigger
        targetTick = TIM10->CNT+1;
        while (TIM10->CNT <= targetTick);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SEN2 trigger
      }
    }
    
    if (cycleCounter % 10 == 4){
      if (TIM10->CNT < 65534){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); //SEN3 trigger
        targetTick = TIM10->CNT+1;
        while (TIM10->CNT <= targetTick);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); //SEN3 trigger
      }
    }
    
    if (cycleCounter % 10 == 6){
      if (TIM10->CNT < 65534){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); //SEN4 trigger
        targetTick = TIM10->CNT+1;
        while (TIM10->CNT <= targetTick);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //SEN4 trigger
      }
    }
    
    if (cycleCounter % 4 == 0){
      /* Read Acceleration*/
      BSP_ACCELERO_GetXYZ(accelerometerBuffer);

      accX = accelerometerBuffer[0];
      accY = accelerometerBuffer[1];
      accZ = accelerometerBuffer[2];
      
      accXFiltered = filter2(accXFiltered, accX, 0.2);
      accYFiltered = filter2(accYFiltered, accY, 0.2);
      accZFiltered = filter2(accZFiltered, accZ, 0.2);
     
      fNormAcc = sqrt((accXFiltered*accXFiltered)+(accYFiltered*accYFiltered)+(accZFiltered*accZFiltered));
        
      fSinRoll = accYFiltered/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = accXFiltered/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
        
      if ( fSinRoll >0) {
        if (fCosRoll>0) roll = acos(fCosRoll)*180/PI;
        else            roll = acos(fCosRoll)*180/PI + 180;
      }
      else {
        if (fCosRoll>0) roll = acos(fCosRoll)*180/PI + 360;
        else            roll = acos(fCosRoll)*180/PI + 180;
      }
       
      if ( fSinPitch >0) {
        if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI;
        else             pitch = acos(fCosPitch)*180/PI + 180;
      }
      else {
        if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI + 360;
        else             pitch = acos(fCosPitch)*180/PI + 180;
      }

      if (roll >=360)  roll = 360 - roll;
      if (pitch >=360) pitch = 360 - pitch;
      
      /* Read Magnetometer*/
      LSM303DLHC_MagReadXYZ(compassBuffer);
      compassX = compassBuffer[0];
      compassY = compassBuffer[1];
      compassZ = compassBuffer[2];
    }
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Receive_IT(&huart6,receiveBuffer,3);
    
    if ((receiveBuffer[0] == 0xFF) && (receiveBuffer[1] <=100) && (receiveBuffer[2] <=180)){
      //referenceSpeedOrig = receiveBuffer[1] / 10.0;
      //referenceSpeed = calcSpdRef_LUT(referenceSpeedOrig);
      referenceSpeed = receiveBuffer[1] / 25.0;
      referenceAngle = receiveBuffer[2] * 2.0;
      calculateLRSpeeds(referenceSpeed, referenceAngle);
      timeStamp = HAL_GetTick();
    }    
    
    timeOutGuard = HAL_GetTick() - timeStamp;
    
    if ((timeOutGuard) > 3000){
      referenceSpeed = 0;
      referenceAngle = 0;
    }
    
    osDelay(20);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartIdleTask */
/**
* @brief Function implementing the idleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIdleTask */
void StartIdleTask(void const * argument)
{
  /* USER CODE BEGIN StartIdleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
  }
  /* USER CODE END StartIdleTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  WS2812_One_RGB(3,(WS2812_RGB_t){0,0,10},1);
  /* Infinite loop */
  for(;;)
  {
    osDelay(250);
    WS2812_Rotate_Left(1);

  }
  /* USER CODE END StartLedTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
