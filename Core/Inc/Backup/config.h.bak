/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    config.h
  * @brief   Header for configuration module
  ******************************************************************************
  * Модуль для работы с конфигурацией устройства, хранящейся во Flash памяти.
  * Обеспечивает загрузку, сохранение и восстановление настроек по умолчанию.
  * РЕАЛИЗОВАНА СИСТЕМА РАЗГРАНИЧЕНИЯ ДОСТУПА С АВТОМАТИЧЕСКОЙ ЗАПИСЬЮ ПРИ ИЗМЕНЕНИИ.
  * CRC32 рассчитывается только для регистров 0-203 как указано в ТЗ.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Константы модуля конфигурации
#define CONFIG_FLASH_ADDRESS    0x0801F800  // Адрес во Flash для хранения конфигурации (Page 127 (1KB pages))
#define CONFIG_FLASH_SIZE       1024        // Размер области конфигурации (1Кб)
#define CONFIG_MAGIC_NUMBER     0x55AA55AA  // Магическое число для проверки валидности конфигурации

// Таймауты для операций Flash (в миллисекундах)
#define FLASH_UNLOCK_TIMEOUT   100U
#define FLASH_ERASE_TIMEOUT    1000U
#define FLASH_PROGRAM_TIMEOUT  500U

// Значения по умолчанию для новых регистров
#define DEFAULT_DEVICE_TYPE    18          // Код типа устройства по умолчанию
#define DEFAULT_SERIAL_NUMBER  0           // Серийный номер по умолчанию

// Объявление внешних переменных Modbus для доступа из config.c
extern uint16_t modbus_registers[MODBUS_REG_COUNT];
extern volatile uint8_t current_access_level;  // Текущий уровень доступа

/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Структура конфигурации с выравниванием по 4 байту для точного размещения во Flash
#pragma pack(push, 4)
typedef struct {
    uint32_t magic_number;                 // Магическое число для проверки валидности
    uint16_t registers[MODBUS_REG_COUNT];  // Все регистры конфигурации
    uint32_t crc32;                        // Контрольная сумма структуры (только регистры 0-203)
} Config_TypeDef;
#pragma pack(pop)
/* USER CODE END ET */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
// Прототипы функций модуля конфигурации
void CONFIG_Load(void);
void CONFIG_Save(void);
void CONFIG_Set_Defaults(void);
uint32_t CONFIG_Calculate_CRC(void);

// Новые функции для работы с системой доступа
void CONFIG_Reset_Communication_Settings(void);
uint8_t CONFIG_Is_First_Start(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */