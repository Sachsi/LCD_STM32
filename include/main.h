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
#include "stm32g0xx_hal.h"

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
void SystemClock_Config(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define LCD_RD_Pin GPIO_PIN_0
#define LCD_RD_GPIO_Port GPIOA
#define LCD_WR_Pin GPIO_PIN_1
#define LCD_WR_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOA
#define LED_GREEN_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_11
#define LCD_RST_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_8
#define LCD_D7_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_9
#define LCD_D0_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_7
#define LCD_D1_GPIO_Port GPIOC
#define LCD_D2_Pin GPIO_PIN_10
#define LCD_D2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_4
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_5
#define LCD_D4_GPIO_Port GPIOB

enum _LCD_PIN{
	LCD_D0 = 9,
	LCD_D1 = 7,
	LCD_D2 = 10,
	LCD_D3 = 3,
	LCD_D4 = 5,
	LCD_D5 = 4,
	LCD_D6 = 14,
	LCD_D7 = 8
} LCD_PIN;
/* USER CODE BEGIN Private defines */
//GPIOC Port
#define LCD_MASK_C (1 << 7)
#define LCD_BIT_MASK_C (1 << 1 )

//GPIOA Port
#define LCD_MASK_A (1 << 9 | 1 << 10 | 1 << 8)
#define LCD_BIT_MASK_A ( 1 << 0 | 1 << 2 | 1 << 7)
//GPIOB Port
#define LCD_MASK_B (1 << 3 | 1 << 4 | 1 << 5 | 1 << 6)
#define LCD_BIT_MASK_B (1 << 5 | 1 << 4 | 1 << 3 | 1 << 6 )
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
