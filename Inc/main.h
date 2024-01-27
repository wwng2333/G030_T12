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

#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "u8g2.h"

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
#define SW_Pin LL_GPIO_PIN_14
#define SW_GPIO_Port GPIOC
#define EC11_D_Pin LL_GPIO_PIN_15
#define EC11_D_GPIO_Port GPIOC
#define SCL_Pin LL_GPIO_PIN_2
#define SCL_GPIO_Port GPIOA
#define SDA_Pin LL_GPIO_PIN_3
#define SDA_GPIO_Port GPIOA
#define SPI1_RST_Pin LL_GPIO_PIN_4
#define SPI1_RST_GPIO_Port GPIOA
#define SPI_DC_Pin LL_GPIO_PIN_6
#define SPI_DC_GPIO_Port GPIOA
#define SPI1_CS_Pin LL_GPIO_PIN_12
#define SPI1_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
void MainScreen(u8g2_t *u8g2);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
