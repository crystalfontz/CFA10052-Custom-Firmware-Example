/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define TIMER_MAX					UINT32_MAX
#define TIMER_RATE_MS				(HAL_GetTickFreq())
#define IF_TIMER_EXPIRED(var)		if ((HAL_GetTick() - (var)) < (TIMER_MAX/2))
#define TIMER_SET_NEXT(var,time_ms)	(var) = HAL_GetTick() + ((time_ms) * TIMER_RATE_MS);
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
#define H1_13_GPIO4_Pin LL_GPIO_PIN_13
#define H1_13_GPIO4_GPIO_Port GPIOC
#define OSC_IN_Pin LL_GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin LL_GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define LCD_D0_KEY_UP_Pin LL_GPIO_PIN_0
#define LCD_D0_KEY_UP_GPIO_Port GPIOC
#define LCD_D1_KEY_DOWN_Pin LL_GPIO_PIN_1
#define LCD_D1_KEY_DOWN_GPIO_Port GPIOC
#define LCD_D2_KEY_LEFT_Pin LL_GPIO_PIN_2
#define LCD_D2_KEY_LEFT_GPIO_Port GPIOC
#define LCD_D3_KEY_RIGHT_Pin LL_GPIO_PIN_3
#define LCD_D3_KEY_RIGHT_GPIO_Port GPIOC
#define LED0R_Pin LL_GPIO_PIN_0
#define LED0R_GPIO_Port GPIOA
#define LED0G_Pin LL_GPIO_PIN_1
#define LED0G_GPIO_Port GPIOA
#define FBSCAB_TX_Pin LL_GPIO_PIN_2
#define FBSCAB_TX_GPIO_Port GPIOA
#define FBSCAB_RX_Pin LL_GPIO_PIN_3
#define FBSCAB_RX_GPIO_Port GPIOA
#define H1_9_GPIO2_Pin LL_GPIO_PIN_4
#define H1_9_GPIO2_GPIO_Port GPIOA
#define H1_10_GPIO3_Pin LL_GPIO_PIN_5
#define H1_10_GPIO3_GPIO_Port GPIOA
#define LED1R_Pin LL_GPIO_PIN_6
#define LED1R_GPIO_Port GPIOA
#define LED1G_Pin LL_GPIO_PIN_7
#define LED1G_GPIO_Port GPIOA
#define LCD_D4_KEY_OK_Pin LL_GPIO_PIN_4
#define LCD_D4_KEY_OK_GPIO_Port GPIOC
#define LCD_D5_KEY_CANCEL_Pin LL_GPIO_PIN_5
#define LCD_D5_KEY_CANCEL_GPIO_Port GPIOC
#define H1_6_GPIO6_Pin LL_GPIO_PIN_1
#define H1_6_GPIO6_GPIO_Port GPIOB
#define LCD_A0_BOOT1_Pin LL_GPIO_PIN_2
#define LCD_A0_BOOT1_GPIO_Port GPIOB
#define H1_4_GPIO10_Pin LL_GPIO_PIN_10
#define H1_4_GPIO10_GPIO_Port GPIOB
#define LCD_CS_Pin LL_GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_BL_PWM_Pin LL_GPIO_PIN_13
#define LCD_BL_PWM_GPIO_Port GPIOB
#define KEY_BL_PWM_Pin LL_GPIO_PIN_14
#define KEY_BL_PWM_GPIO_Port GPIOB
#define LCD_WR_Pin LL_GPIO_PIN_15
#define LCD_WR_GPIO_Port GPIOB
#define LCD_D6_Pin LL_GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D7_Pin LL_GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOC
#define MICROSD_D0_Pin LL_GPIO_PIN_8
#define MICROSD_D0_GPIO_Port GPIOC
#define MICROSD_D1_Pin LL_GPIO_PIN_9
#define MICROSD_D1_GPIO_Port GPIOC
#define LCD_RD_Pin LL_GPIO_PIN_8
#define LCD_RD_GPIO_Port GPIOA
#define H1_2_TX_Pin LL_GPIO_PIN_9
#define H1_2_TX_GPIO_Port GPIOA
#define H1_1_RX_Pin LL_GPIO_PIN_10
#define H1_1_RX_GPIO_Port GPIOA
#define USB_DM_Pin LL_GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin LL_GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define H1_11_SWDIO_GPIO0_Pin LL_GPIO_PIN_13
#define H1_11_SWDIO_GPIO0_GPIO_Port GPIOA
#define H1_12_SWCLK_GPIO1_Pin LL_GPIO_PIN_14
#define H1_12_SWCLK_GPIO1_GPIO_Port GPIOA
#define LCD_RST_Pin LL_GPIO_PIN_15
#define LCD_RST_GPIO_Port GPIOA
#define MICROSD_D2_Pin LL_GPIO_PIN_10
#define MICROSD_D2_GPIO_Port GPIOC
#define MICROSD_D3_Pin LL_GPIO_PIN_11
#define MICROSD_D3_GPIO_Port GPIOC
#define MICROSD_CK_Pin LL_GPIO_PIN_12
#define MICROSD_CK_GPIO_Port GPIOC
#define MICROSD_CMD_Pin LL_GPIO_PIN_2
#define MICROSD_CMD_GPIO_Port GPIOD
#define H1_3_GPIO9_Pin LL_GPIO_PIN_3
#define H1_3_GPIO9_GPIO_Port GPIOB
#define H1_7_GPIO11_Pin LL_GPIO_PIN_4
#define H1_7_GPIO11_GPIO_Port GPIOB
#define H1_8_GPIO12_Pin LL_GPIO_PIN_5
#define H1_8_GPIO12_GPIO_Port GPIOB
#define LED2R_Pin LL_GPIO_PIN_6
#define LED2R_GPIO_Port GPIOB
#define LED2G_Pin LL_GPIO_PIN_7
#define LED2G_GPIO_Port GPIOB
#define LED3R_Pin LL_GPIO_PIN_8
#define LED3R_GPIO_Port GPIOB
#define LED3G_Pin LL_GPIO_PIN_9
#define LED3G_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
