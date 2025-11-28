/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    globals.h
  * @brief   Глобальные переменные и определения для системы на аппаратных таймерах
  ******************************************************************************
  * Замена системного времени на основе HAL tick на чисто аппаратные таймеры.
  * TIM4 - системный таймер с периодом 1ms, заменяет HAL_GetTick().
  * ДОБАВЛЕНЫ ВНЕШНИЕ ОБЪЯВЛЕНИЯ ДЛЯ СИСТЕМЫ ДОСТУПА
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __GLOBALS_H
#define __GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Global variables ----------------------------------------------------------*/
/* USER CODE BEGIN GV */

/**
  * @brief Глобальная переменная системного времени (в миллисекундах)
  * @note Обновляется в прерывании TIM4, заменяет HAL_GetTick()
  *       Используется для измерения интервалов времени в системе
  */
extern volatile uint32_t system_ticks;

/**
  * @brief Время последнего принятого символа Modbus (в системных тиках)
  * @note Используется для отслеживания активности на линии RS-485
  */
extern uint32_t modbus_last_rx_time;

/**
  * @brief Текущий уровень доступа к системе
  * @note 0 - только чтение, 1 - специалист эксплуатации, 2 - специалист производителя
  *       Обновляется при вводе пароля через регистр 5
  */
extern volatile uint8_t current_access_level;

/**
  * @brief Флаги изменений конфигурации для автоматической записи в Flash
  * @note Устанавливаются при изменении соответствующих параметров
  */
extern bool config_modified;
extern bool calib_modified;

/* USER CODE END GV */

/* Function prototypes -------------------------------------------------------*/
/* USER CODE BEGIN FP */

/**
  * @brief  Получение текущего системного времени
  * @retval Текущее время в миллисекундах
  * @note   Полная замена HAL_GetTick(), использует аппаратный таймер TIM4
  */
uint32_t GET_TICKS(void);

/**
  * @brief  Проверка истекшего времени
  * @param  timestamp: Временная метка для проверки
  * @param  delay_ms: Задержка в миллисекундах
  * @retval true - время истекло, false - время не истекло
  */
bool TICKS_ELAPSED(uint32_t timestamp, uint32_t delay_ms);

/**
  * @brief  Блокирующая задержка на основе аппаратного таймера
  * @param  delay_ms: Задержка в миллисекундах
  * @retval None
  */
void HARDWARE_DELAY(uint32_t delay_ms);

/**
  * @brief  Слабая реализация HAL_GetTick для совместимости с HAL
  * @retval Текущее время в миллисекундах
  */
uint32_t HAL_GetTick(void);

/* USER CODE END FP */

#ifdef __cplusplus
}
#endif

#endif /* __GLOBALS_H */