/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    utilities.h
  * @brief   Header for utilities module
  ******************************************************************************
  * Модуль вспомогательных функций: CRC расчет, проверочные суммы, точные задержки.
  * УНИФИЦИРОВАННЫЙ расчет CRC16 Modbus - одна реализация для всего проекта.
  * Устранено дублирование кода между utilities.c и modbus.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __UTILITIES_H
#define __UTILITIES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Константы для расчета CRC
#define CRC32_POLYNOMIAL        0x04C11DB7  // Стандартный полином CRC32
#define CRC32_INITIAL_VALUE     0xFFFFFFFF  // Начальное значение CRC32
#define CRC32_XOR_OUT           0xFFFFFFFF  // Финальное XOR значение

// Константы для CRC16 Modbus
#define CRC16_MODBUS_POLYNOMIAL 0xA001      // Полином CRC16 Modbus (0x8005 reversed)
#define CRC16_MODBUS_INIT       0xFFFF      // Начальное значение CRC16 Modbus
/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
// Прототипы вспомогательных функций
uint32_t UTILITIES_Calculate_CRC32(uint8_t *data, uint32_t length);
uint16_t UTILITIES_Calculate_CRC16(uint8_t *data, uint16_t length); /* ЕДИНСТВЕННАЯ РЕАЛИЗАЦИЯ CRC16 */
uint8_t UTILITIES_Check_Sum(uint8_t *data, uint32_t length);
void UTILITIES_Delay_US(uint32_t microseconds);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __UTILITIES_H */