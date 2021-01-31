/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void mouse_reset(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CS_Pin GPIO_PIN_15
#define SPI2_CS_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOH
#define SPI3_CS_Pin GPIO_PIN_1
#define SPI3_CS_GPIO_Port GPIOH
#define FL_LED_ADC_Pin GPIO_PIN_0
#define FL_LED_ADC_GPIO_Port GPIOA
#define SL_LED_ADC_Pin GPIO_PIN_1
#define SL_LED_ADC_GPIO_Port GPIOA
#define SR_LED_ADC_Pin GPIO_PIN_2
#define SR_LED_ADC_GPIO_Port GPIOA
#define FR_LED_ADC_Pin GPIO_PIN_3
#define FR_LED_ADC_GPIO_Port GPIOA
#define Battery_ADC_Pin GPIO_PIN_4
#define Battery_ADC_GPIO_Port GPIOA
#define FL_LED_Pin GPIO_PIN_0
#define FL_LED_GPIO_Port GPIOB
#define SL_LED_Pin GPIO_PIN_1
#define SL_LED_GPIO_Port GPIOB
#define SR_LED_Pin GPIO_PIN_2
#define SR_LED_GPIO_Port GPIOB
#define FR_LED_Pin GPIO_PIN_10
#define FR_LED_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define Push_SW_Pin GPIO_PIN_12
#define Push_SW_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
