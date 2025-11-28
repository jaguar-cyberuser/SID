/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    common_defines.h
  * @brief   Common definitions for the entire project
  ******************************************************************************
  * Общие определения и константы для всего проекта.
  * ДОБАВЛЕНЫ КОНСТАНТЫ ДЛЯ СИСТЕМЫ РАЗГРАНИЧЕНИЯ ДОСТУПА
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __COMMON_DEFINES_H
#define __COMMON_DEFINES_H

/* Common definitions for the entire project --------------------------------*/
/* USER CODE BEGIN CommonDefines */

/* Базовые определения состояний */
#define ENABLE  1
#define DISABLE 0

#define SET     1
#define RESET   0

#define ON      1
#define OFF     0

#define HIGH    1
#define LOW     0

#define TRUE    1
#define FALSE   0

#define SUCCESS 1
#define FAILURE 0

/* Коды ошибок для унифицированной обработки */
#define ERROR_NONE              0x00  // Ошибок нет
#define ERROR_TIMEOUT           0x01  // Превышен таймаут операции
#define ERROR_CRC               0x02  // Ошибка контрольной суммы
#define ERROR_INVALID_PARAM     0x03  // Неверный параметр
#define ERROR_HARDWARE          0x04  // Аппаратная ошибка
#define ERROR_COMMUNICATION     0x05  // Ошибка связи
#define ERROR_FLASH             0x06  // Ошибка работы с Flash памятью
#define ERROR_ADC               0x07  // Ошибка АЦП
#define ERROR_MODBUS            0x08  // Ошибка протокола Modbus
#define ERROR_ACCESS_DENIED     0x09  // Ошибка доступа (новый код)

/* Статусы модулей для управления состоянием системы */
#define STATUS_READY            0x00  // Модуль готов к работе
#define STATUS_BUSY             0x01  // Модуль занят выполнением операции
#define STATUS_ERROR            0x02  // Модуль в состоянии ошибки
#define STATUS_INIT             0x03  // Модуль в процессе инициализации
#define STATUS_SLEEP            0x04  // Модуль в режиме энергосбережения

/* Уровни доступа системы */
#define ACCESS_LEVEL_0          0    // Уровень 0: Только чтение (по умолчанию)
#define ACCESS_LEVEL_1          1    // Уровень 1: Специалист эксплуатации
#define ACCESS_LEVEL_2          2    // Уровень 2: Специалист предприятия производителя

/* Пароли для смены уровня доступа */
#define PASSWORD_LEVEL_1        0x55AA  // Пароль для уровня 1
#define PASSWORD_LEVEL_2        0x77FF  // Пароль для уровня 2

/* Маски прав доступа к регистрам */
#define REG_READ_ONLY           0x01    // Регистр только для чтения
#define REG_LEVEL_1_WRITE       0x02    // Запись разрешена на уровне 1
#define REG_LEVEL_2_WRITE       0x04    // Запись разрешена на уровне 2

/* Маски битов для работы с флагами */
#define BIT_0                   (1 << 0)
#define BIT_1                   (1 << 1)
#define BIT_2                   (1 << 2)
#define BIT_3                   (1 << 3)
#define BIT_4                   (1 << 4)
#define BIT_5                   (1 << 5)
#define BIT_6                   (1 << 6)
#define BIT_7                   (1 << 7)

/* Упрощенные макросы для битовых операций */
#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)      ((REG) & (BIT))
#define TOGGLE_BIT(REG, BIT)    ((REG) ^= (BIT))

/* Макросы для проверки диапазонов значений */
#define IN_RANGE(value, min, max) (((value) >= (min)) && ((value) <= (max)))
#define CLAMP(value, min, max)    (((value) < (min)) ? (min) : (((value) > (max)) ? (max) : (value)))

/* Макросы для работы с временем */
#define MS_TO_TICKS(ms)         (ms)  // 1 тик = 1 мс для TIM4
#define TICKS_TO_MS(ticks)      (ticks)
#define SECONDS_TO_MS(s)        ((s) * 1000)
#define MINUTES_TO_MS(m)        ((m) * 60000)

/* Специфичные для проекта константы */
#define ADC_MAX_RAW_VALUE       4095  // Максимальное значение АЦП (12 бит)
#define ADC_REFERENCE_VOLTAGE   3300  // Опорное напряжение АЦП в мВ

/* Modbus специфичные константы */
#define MODBUS_BROADCAST_ADDR   0     // Broadcast адрес Modbus
#define MODBUS_MIN_FRAME_SIZE   8     // Минимальный размер фрейма Modbus RTU
#define MODBUS_MAX_REG_COUNT    125   // Максимальное количество регистров в одном запросе

/* Константы для работы с Flash памятью */
#define CONFIG_FLASH_PAGE_SIZE  1024  // Размер страницы Flash в байтах
#define FLASH_WRITE_ALIGN       4     // Выравнивание для записи в Flash (32 бита)

/* Константы системы доступа */
#define DEFAULT_DEVICE_TYPE     18      // Код типа устройства по умолчанию
#define DEFAULT_SERIAL_NUMBER   0       // Серийный номер по умолчанию

/* USER CODE END CommonDefines */

#endif /* __COMMON_DEFINES_H */
