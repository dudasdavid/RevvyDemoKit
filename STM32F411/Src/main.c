/* USER CODE BEGIN Header */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "WS2812_Lib.h"
#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_accelerometer.h"
#include "FreeRTOS.h"
#include "task.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi5_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId controlTaskHandle;
osThreadId commTaskHandle;
osThreadId idleTaskHandle;
osThreadId ledTaskHandle;
osThreadId applicationTaskHandle;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define COMPASS_X_MAX 702
#define COMPASS_X_MIN -358
#define COMPASS_Y_MAX 293
#define COMPASS_Y_MIN -829
#define COMPASS_Z_MAX 845
#define COMPASS_Z_MIN -282

#define NORMALVOLTAGE_MOT 1.4
#define LOWVOLTAGE_MOT 1.18

#define NORMALVOLTAGE_LOGIC 1.1
#define LOWVOLTAGE_LOGIC 0.8

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

typedef struct {
    const GPIO_TypeDef* port;
    const uint16_t pin;
} GPIO_t;

#define GPIO_PIN(__port__, __pin__) {   \
    .port = GPIO ## __port__,           \
    .pin  = GPIO_PIN_ ## __pin__        \
}

typedef struct
{
    const GPIO_t control;
    volatile uint32_t * const delayRegister;

    uint32_t counter;
    uint32_t start;
    uint32_t end;
    uint32_t length;

    uint16_t debounceCounter;
    float rawDistance;
    float previousDistance;
    float filteredDistance;
} Ultrasound_t;

#define DEFINE_ULTRASOUND(ctrlPort, ctrlPin, timer) \
{                                                               \
    .control = GPIO_PIN(ctrlPort, ctrlPin),                     \
    .delayRegister = &(timer),                                  \
    .counter = 0u,                                              \
    .start = 0u,                                                \
    .end = 0u,                                                  \
    .length = 0u,                                               \
    .debounceCounter = 1000u,                                   \
    .rawDistance = 0.0f,                                        \
    .previousDistance = 0.0f,                                   \
    .filteredDistance = 0.0f                                    \
}

static Ultrasound_t sensors[4] =
{
    DEFINE_ULTRASOUND(A, 4, TIM10->CNT),
    DEFINE_ULTRASOUND(A, 5, TIM10->CNT),
    DEFINE_ULTRASOUND(C, 4, TIM10->CNT),
    DEFINE_ULTRASOUND(C, 5, TIM10->CNT)
};

static void ultrasound_onSenseStart(Ultrasound_t* sensor, uint32_t counter)
{
    sensor->counter++;
    sensor->start = counter;
}

static void ultrasound_onSenseEnd(Ultrasound_t* sensor, uint32_t counter)
{
    sensor->counter++;
    sensor->end = counter;
    if (counter > sensor->start)
    {
        sensor->length = counter - sensor->start;
    }
    else
    {
        sensor->length = (counter + 65536u) - sensor->start;
    }
}

static void ultrasound_startMeasurement(uint8_t sensorIdx)
{
    uint32_t currentTimerCounter = *(sensors[sensorIdx].delayRegister);
    uint32_t targetCounter = currentTimerCounter + 2u;  /* +2 so the delay is between 10 and 20us */

    /* Generate a trigger pulse between 10 and 20us (timer resolution limit) */
    HAL_GPIO_WritePin(sensors[sensorIdx].control.port, sensors[sensorIdx].control.pin, GPIO_PIN_SET);
    while (*(sensors[sensorIdx].delayRegister) != targetCounter);
    HAL_GPIO_WritePin(sensors[sensorIdx].control.port, sensors[sensorIdx].control.pin, GPIO_PIN_RESET);
}

typedef struct {
    const GPIO_t ctrlA;
    const GPIO_t ctrlB;
    volatile uint32_t * const speedControlRegister;
} MotorControl_t;

#define MOTOR_LEFT1 (0u)
#define MOTOR_LEFT2 (1u)
#define MOTOR_LEFT3 (2u)

#define MOTOR_RIGHT1 (3u)
#define MOTOR_RIGHT2 (4u)
#define MOTOR_RIGHT3 (5u)

static const MotorControl_t motors[] =
{
    /* Left side motors */
    {
        .ctrlA = GPIO_PIN(B, 15),
        .ctrlB = GPIO_PIN(D, 11),
        .speedControlRegister = &(TIM1->CCR1)
    },
    {
        .ctrlA = GPIO_PIN(C, 13),
        .ctrlB = GPIO_PIN(C, 14),
        .speedControlRegister = &(TIM1->CCR2)
    },
    {
        .ctrlA = GPIO_PIN(C, 10),
        .ctrlB = GPIO_PIN(C, 11),
        .speedControlRegister = &(TIM1->CCR3)
    },

    /* Right side motors */
    {
        .ctrlA = GPIO_PIN(A, 6),
        .ctrlB = GPIO_PIN(A, 7),
        .speedControlRegister = &(TIM1->CCR4)
    },
    {
        .ctrlA = GPIO_PIN(A,  9),
        .ctrlB = GPIO_PIN(A, 10),
        .speedControlRegister = &(TIM2->CCR1)
    },
    {
        .ctrlA = GPIO_PIN(D, 12),
        .ctrlB = GPIO_PIN(D, 13),
        .speedControlRegister = &(TIM2->CCR2)
    }
};

static void MotorControl_SetSpeed(const MotorControl_t* motor, int32_t speed)
{
    const uint32_t motorScaling = 4800 / 100;

    if (speed >= 0)
    {
        HAL_GPIO_WritePin(motor->ctrlA.port, motor->ctrlA.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->ctrlB.port, motor->ctrlB.pin, GPIO_PIN_RESET);
        *(motor->speedControlRegister) = speed * motorScaling;
    }
    else
    {
        HAL_GPIO_WritePin(motor->ctrlA.port, motor->ctrlA.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->ctrlB.port, motor->ctrlB.pin, GPIO_PIN_SET);
        *(motor->speedControlRegister) = -speed * motorScaling;
    }
}

typedef struct
{
    float P;
    float I;
    float D;

    float umax;
    float umin;

    float integrator;
    float previousError;

    float unsaturatedControlSignal;
    float saturatedControlSignal;
} PID_t;

#define DEFINE_PID(p, i, d, min, max)   \
{                                       \
    .P = p,                             \
    .I = i,                             \
    .D = d,                             \
    .umax = max,                        \
    .umin = min,                        \
    .integrator = 0.0f,                 \
    .previousError = 0.0f,              \
    .unsaturatedControlSignal = 0.0f,   \
    .saturatedControlSignal = 0.0f      \
}

static PID_t speedControllers[] =
{
    /* Left side motors */
    DEFINE_PID(750.0f, 10.0f, 0.0f, -100.0f, 100.0f),
    DEFINE_PID(750.0f, 10.0f, 0.0f, -100.0f, 100.0f),
    DEFINE_PID(750.0f, 10.0f, 0.0f, -100.0f, 100.0f),

    /* Right side motors */
    DEFINE_PID(750.0f, 10.0f, 0.0f, -100.0f, 100.0f),
    DEFINE_PID(750.0f, 10.0f, 0.0f, -100.0f, 100.0f),
    DEFINE_PID(750.0f, 10.0f, 0.0f, -100.0f, 100.0f),
};

static PID_t positionController = DEFINE_PID(0.01f, 0.0f, 0.0f, -FLT_MAX, FLT_MAX);

static float saturateFloat(float i, float min, float max);

static float Controller_PID(PID_t* context, float reference, float feedback)
{
    float error = reference - feedback;

    /* Proportional */
    float p = context->P * error;

    /* Differential */
    float d = context->D * (context->previousError - error);
    context->previousError = error;

    /* Integral - using the FOXBORO structure */
    float i = context->integrator;

    float unsaturated = p + i + d;
    float saturated = saturateFloat(unsaturated, context->umin, context->umax);

    context->integrator = (1.0f - context->I) * context->integrator + context->I * saturated;

    /* Debug variables */
    context->unsaturatedControlSignal = unsaturated;
    context->saturatedControlSignal = saturated;

    return saturated;
}

static float Controller_P(PID_t* context, float reference, float feedback)
{
    float error = reference - feedback;

    float p = context->P * error;

    return saturateFloat(p, context->umin, context->umax);
}

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

static volatile float wheelL1RawVelocity_mps, wheelL1FiltVelocity_mps, wheelL1FiltVelocity_rpm = 0;
static volatile float wheelL2RawVelocity_mps, wheelL2FiltVelocity_mps, wheelL2FiltVelocity_rpm = 0;
static volatile float wheelL3RawVelocity_mps, wheelL3FiltVelocity_mps, wheelL3FiltVelocity_rpm = 0;
static volatile float wheelR1RawVelocity_mps, wheelR1FiltVelocity_mps, wheelR1FiltVelocity_rpm = 0;
static volatile float wheelR2RawVelocity_mps, wheelR2FiltVelocity_mps, wheelR2FiltVelocity_rpm = 0;
static volatile float wheelR3RawVelocity_mps, wheelR3FiltVelocity_mps, wheelR3FiltVelocity_rpm = 0;

static volatile float wheelSize_m = 0.20f; // 20cm circumference [m]
static volatile uint16_t encoderSignalsPerRound = 360; // 3 counts per motor round and a 120:1 gearbox
static volatile uint16_t speedSensingTaskCycle_ms = 10; // 10ms task cycle time
static volatile float distancePerImpulse = 0.20f / 360; // wheelSize_m / encoderSignalsPerRound, [m / tick]
static volatile float anglePerImpulse = 360 / 360; // 360 / encoderSignalsPerRound, [� / tick]

static volatile float referenceSpeedLeft = 0;
static volatile float referenceSpeedRight = 0;

static volatile uint32_t timeStamp =0;
static volatile uint32_t timeOutGuard, timeOutGuardMax =0;
static uint8_t receiveBuffer[10]={0};
static volatile float referenceSpeed = 0;
static volatile float referenceAngle = 0;
static uint8_t messageCounter, messageCounterPrev = 0;
static volatile uint8_t buttonState = 0;

static volatile float nextSpeedLeft, nextSpeedRight = 0;
static volatile float reqSpeedLeft, reqSpeedRight, currSpeedLeft, currSpeedRight = 0;
static volatile float speedRampRate = 0.05;

static volatile float wheelControlSignal[6];

static volatile float positionErrorL3_deg = 0;
static volatile float referenceAngleL3_deg = 0;

static volatile uint16_t ultrasonicDebounceLimit = 50;
static volatile uint8_t ultrasonicActive = 0;
static volatile uint8_t bumperActive = 0;
static volatile uint8_t autonomousActive = 0;
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
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI5_Init(void);
void StartDefaultTask(void const * argument);
void StartSensorTask(void const * argument);
void StartControlTask(void const * argument);
void StartCommTask(void const * argument);
void StartIdleTask(void const * argument);
void StartLedTask(void const * argument);
void StartApplicationTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_SPI5_Init();
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
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim9, TIM_CHANNEL_2);
  
  //Start internal timer for 10us time measurement
  HAL_TIM_Base_Start(&htim10);
  
  // Test code START
  TIM1->CCR1 = 0*4800/100; //LEFT1
  TIM1->CCR2 = 0*4800/100; //LEFT2
  TIM1->CCR3 = 0*4800/100; //LEFT3
  TIM1->CCR4 = 0*4800/100; //RIGHT1
  TIM2->CCR1 = 0*4800/100; //RIGHT2
  TIM2->CCR3 = 0*4800/100; //RIGHT3
 
  TIM3->CCR1 = 5*20000/100; //SERVO1 - L1
  TIM3->CCR2 = 5*20000/100; //SERVO2 - L2
  TIM3->CCR3 = 5*20000/100; //SERVO3 - L3
  TIM3->CCR4 = 5*20000/100; //SERVO4 - L4
  TIM5->CCR1 = 5*20000/100; //SERVO5 - L5
  TIM5->CCR2 = 5*20000/100; //SERVO6 - L6
  
  TIM9->CCR2 = 0*2400/100; //Buzzer
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //LEFT1 FW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); //LEFT1 RW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //LEFT2 FW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); //LEFT2 RW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); //LEFT3 FW
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); //LEFT3 RW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,  GPIO_PIN_RESET); //RIGHT1 FW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,  GPIO_PIN_SET); //RIGHT1 RW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_RESET); //RIGHT2 FW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //RIGHT2 RW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //RIGHT3 FW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //RIGHT3 RW
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

  /* definition and creation of applicationTask */
  osThreadDef(applicationTask, StartApplicationTask, osPriorityNormal, 0, 128);
  applicationTaskHandle = osThreadCreate(osThread(applicationTask), NULL);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
    Error_Handler();
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
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 119;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 95;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2400;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 960;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

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
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) mot1Cntr++;
    else mot1Cntr--;
    break;
  case GPIO_PIN_1:
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) mot2Cntr++;
    else mot2Cntr--;
    break;
  case GPIO_PIN_2:
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) mot3Cntr++;
    else mot3Cntr--;
    break;
  case GPIO_PIN_3:
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)) mot4Cntr++;
    else mot4Cntr--;
    break;
  case GPIO_PIN_4:
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)) mot5Cntr++;
    else mot5Cntr--;
    break;
  case GPIO_PIN_5:
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)) mot6Cntr++;
    else mot6Cntr--;
    break;
  case GPIO_PIN_6:
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6))
    {
        ultrasound_onSenseStart(&sensors[0], TIM10->CNT);
    }
    else
    {
        ultrasound_onSenseEnd(&sensors[0], TIM10->CNT);
    }
    break; 
  case GPIO_PIN_7:
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7))
    {
        ultrasound_onSenseStart(&sensors[1], TIM10->CNT);
    }
    else
    {
        ultrasound_onSenseEnd(&sensors[1], TIM10->CNT);
    }
    break; 
  case GPIO_PIN_8:
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8))
      {
          ultrasound_onSenseStart(&sensors[2], TIM10->CNT);
      }
      else
      {
          ultrasound_onSenseEnd(&sensors[2], TIM10->CNT);
      }
    break; 
  case GPIO_PIN_9:
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9))
      {
          ultrasound_onSenseStart(&sensors[3], TIM10->CNT);
      }
      else
      {
          ultrasound_onSenseEnd(&sensors[3], TIM10->CNT);
      }
    break; 
  default:
    break;
  }
}

float filter2 (float avg, float input, float alpha) {
  return (alpha * input) + (1.0f - alpha) * avg;
}

void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

static float saturateFloat(float i, float min, float max)
{
    float val;

    if (i < min)
    {
      val = min;
    }
    else if (i > max)
    {
      val = max;
    }
    else
    {
      val = i;
    }

    return val;
}

void calculateLRSpeeds(float r, float phiDeg) {

  float phiRad = phiDeg * (M_PI / 180.0f);
  
  if ((phiRad >= 0) && (phiRad < M_PI_2)) //1st quadrant
  {
      referenceSpeedLeft = r;
      referenceSpeedRight = -r * cosf(2 * phiRad);
  }
  else if ((phiRad >= M_PI_2) && (phiRad < M_PI)) //2nd quadrant
  {
      referenceSpeedLeft = -r * cosf(2 * phiRad);
      referenceSpeedRight = r;
  }
  else if ((phiRad >= M_PI) && (phiRad < 3 * M_PI_2)) //3rd quadrant
  {
      referenceSpeedLeft = -r;
      referenceSpeedRight = r * cosf(2 * phiRad);
  }
  else if ((phiRad >= 3 * M_PI_2) && (phiRad < M_2_PI)) //4th quadrant
  {
      referenceSpeedLeft = r * cosf(2 * phiRad);
      referenceSpeedRight = -r;
  }
}

void processButtons(uint8_t buttons){
  if ((buttons & 0x01) == 0x01){
    referenceAngleL3_deg += 1;
  }
  if ((buttons & 0x04) == 0x04){
    referenceAngleL3_deg -= 1;
  }
  
  if ((buttons & 0x10) == 0x10){
    autonomousActive = 1;
  }
  else {
    autonomousActive = 0;
  }

  //else{
  //  //referenceAngleL3_deg = wheelL3Angle_deg;
  //}
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
    vTaskSuspend(NULL);
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
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buf,6);
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
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
    vTaskDelayUntil(&xLastWakeTime, speedSensingTaskCycle_ms);
    cycleCounter++;
    
    for (uint8_t i = 0u; i < sizeof(sensors) / sizeof(sensors[0]); i++)
    {
        const float ultrasoundSmoothingCoeff = 0.2f;
        const float ultrasoundTimeToDistanceCoeff = 0.34f / 2;
        sensors[i].rawDistance = sensors[i].length * ultrasoundTimeToDistanceCoeff;

        if (sensors[i].rawDistance == sensors[i].previousDistance)
        {
            if (sensors[i].debounceCounter < 1000u)
            {
                sensors[i].debounceCounter++;
            }
        }
        else
        {
            sensors[i].debounceCounter = 0u;
        }
        sensors[i].previousDistance = sensors[i].rawDistance;
        sensors[i].filteredDistance = filter2(sensors[i].filteredDistance, sensors[i].rawDistance, ultrasoundSmoothingCoeff);
    }
    
    batteryVoltage1 = ADC_Values[0]*3.0*10/4095;
    batteryVoltage2 = ADC_Values[1]*3.0*10/4095;
    sen1AnalogValue = ADC_Values[2];
    sen2AnalogValue = ADC_Values[3];
    sen3AnalogValue = ADC_Values[4];
    sen4AnalogValue = ADC_Values[5];

    /* Update wheel 1 */
    int32_t dcntr1 = mot1Cntr - mot1CntrPrev;
    /* Assume that an overflow has happened and fix it */
    if (dcntr1 > 32000)
    {
        dcntr1 -= 65536;
    }
    else if (dcntr1 < -32000)
    {
        dcntr1 += 65536;
    }
    wheelL1Odometer_m += dcntr1 * distancePerImpulse;
    wheelL1Angle_deg += dcntr1 * anglePerImpulse;

    /* Update wheel 2 */
    int32_t dcntr2 = mot2Cntr - mot2CntrPrev;
    /* Assume that an overflow has happened and fix it */
    if (dcntr2 > 32000)
    {
        dcntr2 -= 65536;
    }
    else if (dcntr2 < -32000)
    {
        dcntr2 += 65536;
    }
    wheelL2Odometer_m += dcntr2 * distancePerImpulse;
    wheelL2Angle_deg += dcntr2 * anglePerImpulse;

    /* Update wheel 3 */
    int32_t dcntr3 = mot3Cntr - mot3CntrPrev;
    /* Assume that an overflow has happened and fix it */
    if (dcntr3 > 32000)
    {
        dcntr3 -= 65536;
    }
    else if (dcntr3 < -32000)
    {
        dcntr3 += 65536;
    }
    wheelL3Odometer_m += dcntr3 * distancePerImpulse;
    wheelL3Angle_deg += dcntr3 * anglePerImpulse;

    /* Update wheel 4 */
    int32_t dcntr4 = mot4Cntr - mot4CntrPrev;
    /* Assume that an overflow has happened and fix it */
    if (dcntr4 > 32000)
    {
        dcntr4 -= 65536;
    }
    else if (dcntr4 < -32000)
    {
        dcntr4 += 65536;
    }
    wheelR1Odometer_m += dcntr4 * distancePerImpulse;
    wheelR1Angle_deg += dcntr4 * anglePerImpulse;

    /* Update wheel 5 */
    int32_t dcntr5 = mot5Cntr - mot5CntrPrev;
    /* Assume that an overflow has happened and fix it */
    if (dcntr5 > 32000)
    {
        dcntr5 -= 65536;
    }
    else if (dcntr5 < -32000)
    {
        dcntr5 += 65536;
    }
    wheelR2Odometer_m += dcntr5 * distancePerImpulse;
    wheelR2Angle_deg += dcntr5 * anglePerImpulse;

    /* Update wheel 6 */
    int32_t dcntr6 = mot6Cntr - mot6CntrPrev;
    /* Assume that an overflow has happened and fix it */
    if (dcntr6 > 32000)
    {
        dcntr6 -= 65536;
    }
    else if (dcntr6 < -32000)
    {
        dcntr6 += 65536;
    }
    wheelR3Odometer_m += dcntr6 * distancePerImpulse;
    wheelR3Angle_deg += dcntr6 * anglePerImpulse;
    
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
    
    wheelL1FiltVelocity_rpm = wheelL1FiltVelocity_mps / wheelSize_m * 360 / 6;
    wheelL2FiltVelocity_rpm = wheelL2FiltVelocity_mps / wheelSize_m * 360 / 6;
    wheelL3FiltVelocity_rpm = wheelL3FiltVelocity_mps / wheelSize_m * 360 / 6;
    wheelR1FiltVelocity_rpm = wheelR1FiltVelocity_mps / wheelSize_m * 360 / 6;
    wheelR2FiltVelocity_rpm = wheelR2FiltVelocity_mps / wheelSize_m * 360 / 6;
    wheelR3FiltVelocity_rpm = wheelR3FiltVelocity_mps / wheelSize_m * 360 / 6;
    
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
    
    if (cycleCounter <= 6 && (cycleCounter % 2) == 0)
    {
        uint8_t sensorIdx = cycleCounter / 2;

        ultrasound_startMeasurement(sensorIdx);
    }

    if (cycleCounter % 4 == 0){
      /* Read Acceleration*/
      float accelerometerBuffer[3];
      BSP_ACCELERO_GetXYZ(accelerometerBuffer);

      accX = accelerometerBuffer[0];
      accY = accelerometerBuffer[1];
      accZ = accelerometerBuffer[2];
      
      accXFiltered = filter2(accXFiltered, accX, 0.2);
      accYFiltered = filter2(accYFiltered, accY, 0.2);
      accZFiltered = filter2(accZFiltered, accZ, 0.2);
     
      fNormAcc = sqrtf((accXFiltered*accXFiltered)+(accYFiltered*accYFiltered)+(accZFiltered*accZFiltered));
        
      fSinRoll = accYFiltered/fNormAcc;
      fCosRoll = sqrtf(1.0-(fSinRoll * fSinRoll));
      fSinPitch = accXFiltered/fNormAcc;
      fCosPitch = sqrtf(1.0-(fSinPitch * fSinPitch));
        
      if ( fSinRoll >0) {
        if (fCosRoll>0) roll = acosf(fCosRoll)*180/M_PI;
        else            roll = acosf(fCosRoll)*180/M_PI + 180;
      }
      else {
        if (fCosRoll>0) roll = acosf(fCosRoll)*180/M_PI + 360;
        else            roll = acosf(fCosRoll)*180/M_PI + 180;
      }
       
      if ( fSinPitch >0) {
        if (fCosPitch>0) pitch = acosf(fCosPitch)*180/M_PI;
        else             pitch = acosf(fCosPitch)*180/M_PI + 180;
      }
      else {
        if (fCosPitch>0) pitch = acosf(fCosPitch)*180/M_PI + 360;
        else             pitch = acosf(fCosPitch)*180/M_PI + 180;
      }

      if (roll >=360)  roll = 360 - roll;
      if (pitch >=360) pitch = 360 - pitch;
      
      /* Read Magnetometer*/
      float compassBuffer[3];
      LSM303DLHC_MagReadXYZ(compassBuffer);
      compassX = compassBuffer[0];
      compassY = compassBuffer[1];
      compassZ = compassBuffer[2];
    }

    if (cycleCounter % 2 == 0)
    {
        /* Calculate speed reference signals */
        reqSpeedLeft = -referenceSpeedLeft;
        reqSpeedRight = referenceSpeedRight;

        nextSpeedLeft = saturateFloat(reqSpeedLeft, currSpeedLeft - speedRampRate, currSpeedLeft + speedRampRate);
        nextSpeedRight = saturateFloat(reqSpeedRight, currSpeedRight - speedRampRate, currSpeedRight + speedRampRate);

        /* Update position controller */
        float l3Speed = Controller_P(&positionController, referenceAngleL3_deg, wheelL3Angle_deg);

        float nextSpeeds[] =
        {
            nextSpeedLeft,
            nextSpeedLeft,
            l3Speed,
            nextSpeedRight,
            nextSpeedRight,
            nextSpeedRight
        };

        currSpeedLeft = nextSpeedLeft;
        currSpeedRight = nextSpeedRight;

        /* Update speed controllers */
        wheelControlSignal[MOTOR_LEFT1] = Controller_PID(&speedControllers[MOTOR_LEFT1], nextSpeeds[MOTOR_LEFT1], wheelL1FiltVelocity_mps);
        wheelControlSignal[MOTOR_LEFT2] = Controller_PID(&speedControllers[MOTOR_LEFT2], nextSpeeds[MOTOR_LEFT2], wheelL2FiltVelocity_mps);
        wheelControlSignal[MOTOR_LEFT3] = Controller_PID(&speedControllers[MOTOR_LEFT3], nextSpeeds[MOTOR_LEFT3], wheelL3FiltVelocity_mps);

        wheelControlSignal[MOTOR_RIGHT1] = Controller_PID(&speedControllers[MOTOR_RIGHT1], nextSpeeds[MOTOR_RIGHT1], wheelR1FiltVelocity_mps);
        wheelControlSignal[MOTOR_RIGHT2] = Controller_PID(&speedControllers[MOTOR_RIGHT2], nextSpeeds[MOTOR_RIGHT2], wheelR2FiltVelocity_mps);
        wheelControlSignal[MOTOR_RIGHT3] = Controller_PID(&speedControllers[MOTOR_RIGHT3], nextSpeeds[MOTOR_RIGHT3], wheelR3FiltVelocity_mps);

        /* Set output signals */
        MotorControl_SetSpeed(&motors[MOTOR_LEFT1], (int32_t) wheelControlSignal[MOTOR_LEFT1]);
        MotorControl_SetSpeed(&motors[MOTOR_LEFT2], (int32_t) wheelControlSignal[MOTOR_LEFT2]);
        MotorControl_SetSpeed(&motors[MOTOR_LEFT3], (int32_t) wheelControlSignal[MOTOR_LEFT3]);

        MotorControl_SetSpeed(&motors[MOTOR_RIGHT1], (int32_t) wheelControlSignal[MOTOR_RIGHT1]);
        MotorControl_SetSpeed(&motors[MOTOR_RIGHT2], (int32_t) wheelControlSignal[MOTOR_RIGHT2]);
        MotorControl_SetSpeed(&motors[MOTOR_RIGHT3], (int32_t) wheelControlSignal[MOTOR_RIGHT3]);
    }

    if (cycleCounter % 2 == 0)
    {
        HAL_UART_Receive_IT(&huart6,receiveBuffer,5);

        if ((receiveBuffer[0] == 0xFF) && (receiveBuffer[1] <=100) && (receiveBuffer[2] <=180))
        {
            //referenceSpeedOrig = receiveBuffer[1] / 10.0;
            //referenceSpeed = calcSpdRef_LUT(referenceSpeedOrig);
            referenceSpeed = receiveBuffer[1] / 667.0;
            referenceAngle = receiveBuffer[2] * 2.0;
            buttonState = receiveBuffer[3];
            messageCounter = receiveBuffer[4];
            if (messageCounter != messageCounterPrev)
            {
                timeStamp = HAL_GetTick();
                messageCounterPrev = messageCounter;
            }
        }

        timeOutGuard = HAL_GetTick() - timeStamp;
        if (timeOutGuard > timeOutGuardMax)
        {
            timeOutGuardMax = timeOutGuard;
        }

        if (timeOutGuard > 1000)
        {
            referenceSpeed = 0;
            referenceAngle = 0;
            buttonState = 0;
        }
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
    vTaskSuspend(NULL);
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
    vTaskSuspend(NULL);
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
  uint8_t cycleCounter=0;
  uint32_t roundCounter = 0;
  int8_t bluetoothBlinkCounter = 0;
  int8_t battery1BlinkCounter = 0;
  int8_t battery2BlinkCounter = 0;
  
  float voltageFullRange_Mot = NORMALVOLTAGE_MOT - LOWVOLTAGE_MOT;
  float voltageFullRange_Logic = NORMALVOLTAGE_LOGIC - LOWVOLTAGE_LOGIC;
  float voltageInRange = 0;
  float voltageRatio = 0;
  float cellVoltage = 0;
  
  WS2812_One_RGB(0,(WS2812_RGB_t){0,10,0},0);
  WS2812_One_RGB(1,(WS2812_RGB_t){10,10,10},0);
  WS2812_One_RGB(2,(WS2812_RGB_t){10,0,0},0);
  WS2812_One_RGB(3,(WS2812_RGB_t){10,10,0},0);
  WS2812_One_RGB(4,(WS2812_RGB_t){0,0,10},1);
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    cycleCounter++;
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    
    if (bumperActive){
      WS2812_All_RGB((WS2812_RGB_t){10,0,0},0);
    }
    else if (ultrasonicActive){
      WS2812_All_RGB((WS2812_RGB_t){0,10,0},0);
    }
    else {
      WS2812_All_RGB((WS2812_RGB_t){0,0,0},0);
      WS2812_One_RGB(((roundCounter%12)+4),(WS2812_RGB_t){0,0,10},0);
      roundCounter++;
      WS2812_Revvy_Shift_Right(0);
    }
    
    
    
    WS2812_One_RGB(0,(WS2812_RGB_t){30,30,30},0);

    if ((timeOutGuard) > 1000){
      if ((bluetoothBlinkCounter >= 0) && (bluetoothBlinkCounter < 3)) WS2812_One_RGB(1,(WS2812_RGB_t){0,0,30},0);
      else if ((bluetoothBlinkCounter >= 3) && (bluetoothBlinkCounter < 7)) WS2812_One_RGB(1,(WS2812_RGB_t){0,0,0},0);
      if (bluetoothBlinkCounter == 6) bluetoothBlinkCounter = -1;
      bluetoothBlinkCounter++;
    }
    else{
      WS2812_One_RGB(1,(WS2812_RGB_t){0,0,30},0);
    }
    
    cellVoltage = batteryVoltage1 / 4;
    if (cellVoltage >= NORMALVOLTAGE_LOGIC){
      WS2812_One_RGB(2,(WS2812_RGB_t){0,30,0},0);
    }
    else if ((cellVoltage < NORMALVOLTAGE_LOGIC) && (cellVoltage >= LOWVOLTAGE_LOGIC)){
      voltageInRange = cellVoltage - LOWVOLTAGE_LOGIC;
      voltageRatio = voltageInRange / voltageFullRange_Logic;
      WS2812_One_RGB(2,(WS2812_RGB_t){(int)(30*(1-voltageRatio)),(int)(30*(voltageRatio)),0},0);
    }
    else if (cellVoltage < LOWVOLTAGE_LOGIC){
      if (battery1BlinkCounter == 0) WS2812_One_RGB(2,(WS2812_RGB_t){30,0,0},0);
      else if (battery1BlinkCounter == 2) WS2812_One_RGB(2,(WS2812_RGB_t){0,0,0},0);
      else if (battery1BlinkCounter == 4) battery1BlinkCounter = -1;
      battery1BlinkCounter++;
    }
    else {
      WS2812_One_RGB(2,(WS2812_RGB_t){30,30,30},0);
    }
    
    cellVoltage = batteryVoltage2 / 4;
    if (cellVoltage >= NORMALVOLTAGE_MOT){
      WS2812_One_RGB(3,(WS2812_RGB_t){0,30,0},0);
    }
    else if ((cellVoltage < NORMALVOLTAGE_MOT) && (cellVoltage >= LOWVOLTAGE_MOT)){
      voltageInRange = cellVoltage - LOWVOLTAGE_MOT;
      voltageRatio = voltageInRange / voltageFullRange_Mot;
      WS2812_One_RGB(3,(WS2812_RGB_t){(int)(30*(1-voltageRatio)),(int)(30*(voltageRatio)),0},0);
    }
    else if (cellVoltage < LOWVOLTAGE_MOT){
      if (battery2BlinkCounter == 0) WS2812_One_RGB(3,(WS2812_RGB_t){30,0,0},0);
      else if (battery2BlinkCounter == 2) WS2812_One_RGB(3,(WS2812_RGB_t){0,0,0},0);
      else if (battery2BlinkCounter == 4) battery2BlinkCounter = -1;
      battery2BlinkCounter++;
    }
    else {
      WS2812_One_RGB(3,(WS2812_RGB_t){30,30,30},0);
    }
    
    WS2812_Refresh();
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

    vTaskDelayUntil(&xLastWakeTime, 150u);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartApplicationTask */
/**
* @brief Function implementing the applicationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartApplicationTask */
void StartApplicationTask(void const * argument)
{
  /* USER CODE BEGIN StartApplicationTask */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    if (autonomousActive){
      referenceAngle = 90;
      referenceSpeed = 0.08;
    }
    
    if (((sensors[0].filteredDistance < 20) && (sensors[0].debounceCounter < ultrasonicDebounceLimit)) 
        || ((sensors[1].filteredDistance < 20) && (sensors[1].debounceCounter < ultrasonicDebounceLimit)) 
        || ((sensors[2].filteredDistance < 20) && (sensors[2].debounceCounter < ultrasonicDebounceLimit)) 
        || ((sensors[3].filteredDistance < 20) && (sensors[3].debounceCounter < ultrasonicDebounceLimit))){
      ultrasonicActive = 1;
      referenceAngle = 180;
      referenceSpeed = 0.1;
    }
    else {
      ultrasonicActive = 0;
    }
    
    
    if ((sen1AnalogValue > 100) || (sen2AnalogValue > 100) || (sen3AnalogValue > 100) || (sen4AnalogValue > 100)){
      bumperActive = 1;
      referenceAngle = 90;
      referenceSpeed = 0.05;
    }
    else {
      bumperActive = 0;
    }
    
    calculateLRSpeeds(referenceSpeed, referenceAngle);
    processButtons(buttonState);

    vTaskDelayUntil(&xLastWakeTime, 10u);
  }
  /* USER CODE END StartApplicationTask */
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
  * @retval None
  */
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
