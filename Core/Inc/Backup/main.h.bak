/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**
  * @brief Статусы приложения для возврата из функций
  */
typedef enum {
  APP_OK       = 0x00U, // Операция завершена успешно
  APP_ERROR    = 0x01U, // Общая ошибка выполнения
  APP_BUSY     = 0x02U, // Устройство или ресурс заняты
  APP_TIMEOUT  = 0x03U  // Превышен таймаут операции
} App_StatusTypeDef;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF1_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF1_USART2

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL            DMA1_Channel7
#define USARTx_RX_DMA_CHANNEL            DMA1_Channel6

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn               DMA1_Channel7_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Channel6_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Channel7_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA1_Channel6_IRQHandler

#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Exported macro -----------------------------------------------------------*/
#define TXSTARTMESSAGESIZE                   (COUNTOF(aTxStartMessage) - 1)
#define TXENDMESSAGESIZE                     (COUNTOF(aTxEndMessage) - 1)

/* Size of Reception buffer */
#define RX_BUFFER_SIZE                       256

// Объявления handle-ов для модулей
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_DE_Pin GPIO_PIN_4
#define RS485_DE_GPIO_Port GPIOA
#define ADC_CH1_VOLTAGE_Pin GPIO_PIN_5
#define ADC_CH1_VOLTAGE_GPIO_Port GPIOA
#define ADC_CH2_CURRENT_Pin GPIO_PIN_6
#define ADC_CH2_CURRENT_GPIO_Port GPIOA
#define RESET_BUTTON_Pin GPIO_PIN_11
#define RESET_BUTTON_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define RESET_BUTTON_PIN      GPIO_PIN_11
#define RESET_BUTTON_PORT     GPIOB
#define RESET_HOLD_TIME_MS    3000    // 3 секунды удержания для сброса настроек

/* Таймеры системы:
 * TIM2 - One-Pulse режим для таймаутов Modbus (разрешение 1us)
 * TIM3 - Триггер АЦП с настраиваемым периодом (5-30ms)
 * TIM4 - ЕДИНСТВЕННЫЙ системный таймер заменяющий SysTick (1ms период)
 * IWDG - Сторожевой таймер 1 секунда
 */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
