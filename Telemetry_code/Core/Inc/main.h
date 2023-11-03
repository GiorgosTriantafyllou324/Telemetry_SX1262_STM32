/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define NRESET_Pin GPIO_PIN_0
#define NRESET_GPIO_Port GPIOB
#define BUSY_Pin GPIO_PIN_1
#define BUSY_GPIO_Port GPIOB
#define DIO1_Pin GPIO_PIN_2
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI2_IRQn
#define DIO2_Pin GPIO_PIN_10
#define DIO2_GPIO_Port GPIOB
#define DIO2_EXTI_IRQn EXTI15_10_IRQn
#define DIO3_Pin GPIO_PIN_15
#define DIO3_GPIO_Port GPIOB
#define RF_SWITCH_Pin GPIO_PIN_6
#define RF_SWITCH_GPIO_Port GPIOC
#define RF_SWITCH_SUPP_Pin GPIO_PIN_7
#define RF_SWITCH_SUPP_GPIO_Port GPIOC
#define CAN_RX_LED_Pin GPIO_PIN_9
#define CAN_RX_LED_GPIO_Port GPIOC
#define CAN_TX_LED_Pin GPIO_PIN_9
#define CAN_TX_LED_GPIO_Port GPIOA
#define RADIO_TX_LED_Pin GPIO_PIN_11
#define RADIO_TX_LED_GPIO_Port GPIOC
#define RADIO_RX_LED_Pin GPIO_PIN_2
#define RADIO_RX_LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
