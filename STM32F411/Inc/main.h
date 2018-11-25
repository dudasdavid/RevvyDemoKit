/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_PWM_Pin GPIO_PIN_6
#define BUZZER_PWM_GPIO_Port GPIOE
#define LEFT2_FW_Pin GPIO_PIN_13
#define LEFT2_FW_GPIO_Port GPIOC
#define LEFT2_RW_Pin GPIO_PIN_14
#define LEFT2_RW_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define SEN1_Pin GPIO_PIN_0
#define SEN1_GPIO_Port GPIOC
#define SEN2_Pin GPIO_PIN_1
#define SEN2_GPIO_Port GPIOC
#define SEN3_Pin GPIO_PIN_2
#define SEN3_GPIO_Port GPIOC
#define SEN4_Pin GPIO_PIN_3
#define SEN4_GPIO_Port GPIOC
#define SERVO5_Pin GPIO_PIN_0
#define SERVO5_GPIO_Port GPIOA
#define SERVO6_Pin GPIO_PIN_1
#define SERVO6_GPIO_Port GPIOA
#define BAT1_Pin GPIO_PIN_2
#define BAT1_GPIO_Port GPIOA
#define BAT2_Pin GPIO_PIN_3
#define BAT2_GPIO_Port GPIOA
#define SEN1_OUT_Pin GPIO_PIN_4
#define SEN1_OUT_GPIO_Port GPIOA
#define SEN2_OUT_Pin GPIO_PIN_5
#define SEN2_OUT_GPIO_Port GPIOA
#define RIGHT1_FW_Pin GPIO_PIN_6
#define RIGHT1_FW_GPIO_Port GPIOA
#define RIGHT1_RW_Pin GPIO_PIN_7
#define RIGHT1_RW_GPIO_Port GPIOA
#define SEN3_OUT_Pin GPIO_PIN_4
#define SEN3_OUT_GPIO_Port GPIOC
#define SEN4_OUT_Pin GPIO_PIN_5
#define SEN4_OUT_GPIO_Port GPIOC
#define SERVO3_Pin GPIO_PIN_0
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_1
#define SERVO4_GPIO_Port GPIOB
#define RIGHT1_ENC_B_Pin GPIO_PIN_7
#define RIGHT1_ENC_B_GPIO_Port GPIOE
#define RIGHT2_ENC_B_Pin GPIO_PIN_8
#define RIGHT2_ENC_B_GPIO_Port GPIOE
#define LEFT1_PWM_Pin GPIO_PIN_9
#define LEFT1_PWM_GPIO_Port GPIOE
#define LEFT2_PWM_Pin GPIO_PIN_11
#define LEFT2_PWM_GPIO_Port GPIOE
#define LEFT3_PWM_Pin GPIO_PIN_13
#define LEFT3_PWM_GPIO_Port GPIOE
#define RIGHT1_PWM_Pin GPIO_PIN_14
#define RIGHT1_PWM_GPIO_Port GPIOE
#define RIGHT3_PWM_Pin GPIO_PIN_10
#define RIGHT3_PWM_GPIO_Port GPIOB
#define LEFT1_ENC_B_Pin GPIO_PIN_12
#define LEFT1_ENC_B_GPIO_Port GPIOB
#define LEFT2_ENC_B_Pin GPIO_PIN_13
#define LEFT2_ENC_B_GPIO_Port GPIOB
#define LEFT3_ENC_B_Pin GPIO_PIN_14
#define LEFT3_ENC_B_GPIO_Port GPIOB
#define LEFT1_FW_Pin GPIO_PIN_15
#define LEFT1_FW_GPIO_Port GPIOB
#define SEN3_EXTI_Pin GPIO_PIN_8
#define SEN3_EXTI_GPIO_Port GPIOD
#define SEN3_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define SEN4_EXTI_Pin GPIO_PIN_9
#define SEN4_EXTI_GPIO_Port GPIOD
#define SEN4_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define RIGHT3_ENC_B_Pin GPIO_PIN_10
#define RIGHT3_ENC_B_GPIO_Port GPIOD
#define LEFT1_RW_Pin GPIO_PIN_11
#define LEFT1_RW_GPIO_Port GPIOD
#define RIGHT3_FW_Pin GPIO_PIN_12
#define RIGHT3_FW_GPIO_Port GPIOD
#define RIGHT3_RW_Pin GPIO_PIN_13
#define RIGHT3_RW_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define HM_10_Pin GPIO_PIN_6
#define HM_10_GPIO_Port GPIOC
#define HM_10C7_Pin GPIO_PIN_7
#define HM_10C7_GPIO_Port GPIOC
#define SENSOR_SDA_Pin GPIO_PIN_9
#define SENSOR_SDA_GPIO_Port GPIOC
#define SENSOR_SCL_Pin GPIO_PIN_8
#define SENSOR_SCL_GPIO_Port GPIOA
#define RIGHT2_FW_Pin GPIO_PIN_9
#define RIGHT2_FW_GPIO_Port GPIOA
#define RIGHT2_RW_Pin GPIO_PIN_10
#define RIGHT2_RW_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RIGHT2_PWM_Pin GPIO_PIN_15
#define RIGHT2_PWM_GPIO_Port GPIOA
#define LEFT3_FW_Pin GPIO_PIN_10
#define LEFT3_FW_GPIO_Port GPIOC
#define LEFT3_RW_Pin GPIO_PIN_11
#define LEFT3_RW_GPIO_Port GPIOC
#define LEFT1_EXTI0_Pin GPIO_PIN_0
#define LEFT1_EXTI0_GPIO_Port GPIOD
#define LEFT1_EXTI0_EXTI_IRQn EXTI0_IRQn
#define LEFT2_EXTI1_Pin GPIO_PIN_1
#define LEFT2_EXTI1_GPIO_Port GPIOD
#define LEFT2_EXTI1_EXTI_IRQn EXTI1_IRQn
#define LEFT3_EXTI2_Pin GPIO_PIN_2
#define LEFT3_EXTI2_GPIO_Port GPIOD
#define LEFT3_EXTI2_EXTI_IRQn EXTI2_IRQn
#define RIGHT1_EXTI3_Pin GPIO_PIN_3
#define RIGHT1_EXTI3_GPIO_Port GPIOD
#define RIGHT1_EXTI3_EXTI_IRQn EXTI3_IRQn
#define RIGHT2_EXTI4_Pin GPIO_PIN_4
#define RIGHT2_EXTI4_GPIO_Port GPIOD
#define RIGHT2_EXTI4_EXTI_IRQn EXTI4_IRQn
#define RIGHT3_EXTI5_Pin GPIO_PIN_5
#define RIGHT3_EXTI5_GPIO_Port GPIOD
#define RIGHT3_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define SEN1_EXTI_Pin GPIO_PIN_6
#define SEN1_EXTI_GPIO_Port GPIOD
#define SEN1_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define SEN2_EXTI_Pin GPIO_PIN_7
#define SEN2_EXTI_GPIO_Port GPIOD
#define SEN2_EXTI_EXTI_IRQn EXTI9_5_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_4
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_5
#define SERVO2_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define WS2812_PWM_Pin GPIO_PIN_7
#define WS2812_PWM_GPIO_Port GPIOB
#define WS2812_Pin GPIO_PIN_8
#define WS2812_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
