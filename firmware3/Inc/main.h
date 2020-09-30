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
#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user.h"
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
#define GPO_DEBUG_LED_Pin LL_GPIO_PIN_7
#define GPO_DEBUG_LED_GPIO_Port GPIOB
#define GPO_COL_0_Pin LL_GPIO_PIN_0
#define GPO_COL_0_GPIO_Port GPIOA
#define GPO_COL_1_Pin LL_GPIO_PIN_1
#define GPO_COL_1_GPIO_Port GPIOA
#define GPO_COL_2_Pin LL_GPIO_PIN_2
#define GPO_COL_2_GPIO_Port GPIOA
#define GPO_COL_ENABLE_Pin LL_GPIO_PIN_3
#define GPO_COL_ENABLE_GPIO_Port GPIOA
#define GPO_LED_LATCH_Pin LL_GPIO_PIN_4
#define GPO_LED_LATCH_GPIO_Port GPIOA
#define GPI_SET_TIME_Pin LL_GPIO_PIN_6
#define GPI_SET_TIME_GPIO_Port GPIOA
#define ADC1_IN7_LIGHT_SENSE_Pin LL_GPIO_PIN_7
#define ADC1_IN7_LIGHT_SENSE_GPIO_Port GPIOA
#define TIM14_CH1_LED_PWM_Pin LL_GPIO_PIN_1
#define TIM14_CH1_LED_PWM_GPIO_Port GPIOB
#define USART1_TX_RX_Pin LL_GPIO_PIN_6
#define USART1_TX_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
