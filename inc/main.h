/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <arm_math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define _U_DC 208
#define _U_IN 120
#define _U_OUT 100
#define _SQRT3 1.73205081
#define _PIdiv3 1.04719755
#define _PIdiv6 0.523598775598
#define _5PIdiv6 2.61799387799
#define _PI 3.14159265
#define _PI330 5.7595865315
#define _PI210 3.66519142919
#define _PI_90 1.57079632679
#define _DEBUG_FLAG 0


extern float32_t ThetaOut;
extern float32_t ThetaA, ThetaB, ThetaC;
extern float32_t ThetaAB, ThetaBC, ThetaCA;
extern float32_t cosinevalue;
extern float32_t sin_ab, sin_bc, sin_ca;
extern float32_t cos_a, cos_b, cos_c;
extern float32_t DFreq;
extern float32_t V_AB, V_BC, V_CA;
extern float32_t v_ab;
extern float32_t v_ac;
extern float32_t DENOM;
extern float32_t VMat[9];
extern float32_t V_IN[3];
extern float32_t* virt_a;
extern float32_t* virt_b;
extern float32_t* virt_c;
extern float32_t triangleWave;
extern uint16_t Mat[9];
extern uint16_t currVec;
extern uint16_t hasStarted;
extern uint16_t triangle;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define Short_Pin GPIO_PIN_2
#define Short_GPIO_Port GPIOB
#define Short_EXTI_IRQn EXTI2_IRQn
#define LAA1_Pin GPIO_PIN_10
#define LAA1_GPIO_Port GPIOB
#define LAA2_Pin GPIO_PIN_11
#define LAA2_GPIO_Port GPIOB
#define LBB1_Pin GPIO_PIN_12
#define LBB1_GPIO_Port GPIOB
#define LBB2_Pin GPIO_PIN_13
#define LBB2_GPIO_Port GPIOB
#define LCC1_Pin GPIO_PIN_14
#define LCC1_GPIO_Port GPIOB
#define LCC2_Pin GPIO_PIN_15
#define LCC2_GPIO_Port GPIOB
#define Start_Pin GPIO_PIN_8
#define Start_GPIO_Port GPIOA
#define V_C_Pin GPIO_PIN_9
#define V_C_GPIO_Port GPIOA
#define V_C_EXTI_IRQn EXTI9_5_IRQn
#define V_B_Pin GPIO_PIN_10
#define V_B_GPIO_Port GPIOA
#define V_B_EXTI_IRQn EXTI15_10_IRQn
#define V_A_Pin GPIO_PIN_11
#define V_A_GPIO_Port GPIOA
#define V_A_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
