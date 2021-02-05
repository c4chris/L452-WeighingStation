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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define I2Cx_TIMEOUT_MAX 0x3000 /* The value of the maximal timeout for I2C waiting loops */
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
#define I2C3_SCL_Pin GPIO_PIN_0
#define I2C3_SCL_GPIO_Port GPIOC
#define I2C3_SDA_Pin GPIO_PIN_1
#define I2C3_SDA_GPIO_Port GPIOC
#define SPI2_MISO_Pin GPIO_PIN_2
#define SPI2_MISO_GPIO_Port GPIOC
#define SPI2_MOSI_Pin GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port GPIOC
#define DISP_TE_Pin GPIO_PIN_0
#define DISP_TE_GPIO_Port GPIOA
#define DISP_TE_EXTI_IRQn EXTI0_IRQn
#define DISP_NRESET_Pin GPIO_PIN_1
#define DISP_NRESET_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define nENP3B_Pin GPIO_PIN_4
#define nENP3B_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define nENP3A_Pin GPIO_PIN_0
#define nENP3A_GPIO_Port GPIOB
#define nENP1B_Pin GPIO_PIN_2
#define nENP1B_GPIO_Port GPIOB
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define JOY_CENTER_Pin GPIO_PIN_8
#define JOY_CENTER_GPIO_Port GPIOC
#define JOY_LEFT_Pin GPIO_PIN_9
#define JOY_LEFT_GPIO_Port GPIOC
#define nENP1A_Pin GPIO_PIN_8
#define nENP1A_GPIO_Port GPIOA
#define I2C1_SCL_Pin GPIO_PIN_9
#define I2C1_SCL_GPIO_Port GPIOA
#define I2C1_SDA_Pin GPIO_PIN_10
#define I2C1_SDA_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define JOY_DOWN_Pin GPIO_PIN_10
#define JOY_DOWN_GPIO_Port GPIOC
#define JOY_RIGHT_Pin GPIO_PIN_11
#define JOY_RIGHT_GPIO_Port GPIOC
#define JOY_UP_Pin GPIO_PIN_12
#define JOY_UP_GPIO_Port GPIOC
#define SPI1_DCX_Pin GPIO_PIN_3
#define SPI1_DCX_GPIO_Port GPIOB
#define SPI1_NCS_Pin GPIO_PIN_5
#define SPI1_NCS_GPIO_Port GPIOB
#define I2C4_SCL_Pin GPIO_PIN_6
#define I2C4_SCL_GPIO_Port GPIOB
#define I2C4_SDA_Pin GPIO_PIN_7
#define I2C4_SDA_GPIO_Port GPIOB
#define SPI2_NCS_Pin GPIO_PIN_9
#define SPI2_NCS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
