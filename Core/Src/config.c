/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    config.c
  * @brief   Implementation of configuration module
  ******************************************************************************
  * Модуль для работы с конфигурацией, хранящейся во Flash памяти.
  * РЕАЛИЗОВАНА СИСТЕМА РАЗГРАНИЧЕНИЯ ДОСТУПА С АВТОМАТИЧЕСКОЙ ЗАПИСЬЮ:
  * - Автоматическая запись при первом включении
  * - Автоматическая запись при изменении параметров
  * - Раздельная обработка конфигурационных и калибровочных параметров
  * - Сброс только коммуникационных настроек по кнопке
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "utilities.h"
#include "globals.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Приватная переменная - структура конфигурации в RAM
static Config_TypeDef config;

// Флаг первого запуска устройства
static bool first_start = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
// Приватные прототипы функций для обработки ошибок
static uint8_t CONFIG_Check_Flash_Ready(void);
static uint8_t CONFIG_Verify_Flash_Data(void);
static void CONFIG_Initialize_Previous_Registers(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief Проверка первого запуска устройства
  * @retval 1 - первое включение, 0 - конфигурация уже существует
  * @note Проверяет наличие валидной конфигурации во Flash
  */
uint8_t CONFIG_Is_First_Start(void)
{
    return first_start;
}

/**
  * @brief Инициализация буфера предыдущих значений регистров
  * @retval None
  * @note Копирует текущие значения регистров для последующего обнаружения изменений
  */
static void CONFIG_Initialize_Previous_Registers(void)
{
    // Буфер previous_registers определен в globals.c
    // Просто копируем текущие значения
    // Инициализируем буфер предыдущих значений
    memcpy(previous_registers, modbus_registers, sizeof(modbus_registers));
}

/**
  * @brief Загрузка конфигурации из Flash памяти
  * @retval None
  * @note ПРИ ПЕРВОМ ЗАПУСКЕ АВТОМАТИЧЕСКИ СОЗДАЕТ КОНФИГУРАЦИЮ ПО УМОЛЧАНИЮ
  *       И ЗАПИСЫВАЕТ ЕЕ В FLASH
  */
void CONFIG_Load(void)
{
  /* USER CODE BEGIN CONFIG_Load_1 */
  uint32_t *flash_ptr = (uint32_t*)CONFIG_FLASH_ADDRESS;

  // ПРОВЕРКА 1: Проверяем готовность Flash памяти перед чтением
  if (CONFIG_Check_Flash_Ready() != HAL_OK)
  {
    // Flash не готова - загружаем значения по умолчанию
    CONFIG_Set_Defaults();
    first_start = true;
    return;
  }
  
  // ПРОВЕРКА 2: Предварительная проверка магического числа без полного копирования
  uint32_t magic_test = flash_ptr[0];
  if (magic_test != CONFIG_MAGIC_NUMBER)
  {
    // Магическое число не совпадает - ПЕРВЫЙ ЗАПУСК
    CONFIG_Set_Defaults();
    first_start = true;
    
    // АВТОМАТИЧЕСКАЯ ЗАПИСЬ КОНФИГУРАЦИИ ПО УМОЛЧАНИЮ ПРИ ПЕРВОМ ВКЛЮЧЕНИИ
    CONFIG_Save();
    return;
  }
  
  // БЕЗОПАСНОЕ КОПИРОВАНИЕ: Копируем данные из Flash в RAM структуру
  __disable_irq();
  memcpy(&config, flash_ptr, sizeof(Config_TypeDef));
  __enable_irq();
  
  // ПРОВЕРКА 3: Полная проверка целостности конфигурации
  if (config.magic_number != CONFIG_MAGIC_NUMBER || 
      config.crc32 != CONFIG_Calculate_CRC())
  {
    // Конфигурация невалидна - загружаем значения по умолчанию
    CONFIG_Set_Defaults();
    first_start = true;
    
    // АВТОМАТИЧЕСКАЯ ЗАПИСЬ КОНФИГУРАЦИИ ПО УМОЛЧАНИЮ
    CONFIG_Save();
  }
  else
  {
    // Конфигурация валидна - копируем в рабочие регистры Modbus
    memcpy(modbus_registers, config.registers, sizeof(config.registers));
    
    // Обновляем регистр CRC в Modbus для отображения
    modbus_registers[MB_CONFIG_CRC] = config.crc32 & 0xFFFF;
    
    // Устанавливаем уровень доступа 0 при загрузке
    current_access_level = ACCESS_LEVEL_0;
    first_start = false;
    
    // Инициализируем буфер предыдущих значений
    CONFIG_Initialize_Previous_Registers();
  }

  /* USER CODE END CONFIG_Load_1 */
}

/**
  * @brief Сохранение конфигурации во Flash память
  * @retval None
  * @note АВТОМАТИЧЕСКИ ВЫЗЫВАЕТСЯ ПРИ ИЗМЕНЕНИИ ЛЮБОГО ПАРАМЕТРА
  *       СОХРАНЯЕТ ВСЕ РЕГИСТРЫ, НЕЗАВИСИМО ОТ ТИПА ИЗМЕНЕНИЙ
  */
void CONFIG_Save(void)
{
  /* USER CODE BEGIN CONFIG_Save_1 */
  HAL_StatusTypeDef flash_status;
  uint32_t start_time;
  
  // ПОДГОТОВКА ДАННЫХ: Копируем текущие регистры в структуру конфигурации
  memcpy(config.registers, modbus_registers, sizeof(config.registers));
  config.magic_number = CONFIG_MAGIC_NUMBER;
  
  // ОБНОВЛЕНИЕ CRC: Рассчитываем и сохраняем контрольную сумму
  config.crc32 = CONFIG_Calculate_CRC();
  modbus_registers[MB_CONFIG_CRC] = config.crc32 & 0xFFFF;
  config.registers[MB_CONFIG_CRC] = modbus_registers[MB_CONFIG_CRC];
  
  // ПРОВЕРКА 1: Проверяем готовность Flash памяти
  if (CONFIG_Check_Flash_Ready() != HAL_OK)
  {
    return; // Flash не готова - отменяем операцию
  }
  
  // КРИТИЧЕСКАЯ СЕКЦИЯ: Блокируем прерывания для безопасной работы с Flash
  __disable_irq();
  
  // ОПЕРАЦИЯ 1: Разблокировка Flash памяти с таймаутом
  start_time = GET_TICKS();
  HAL_FLASH_Unlock();

  // Проверяем успешность разблокировки
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) 
  {
    if (TICKS_ELAPSED(start_time, FLASH_UNLOCK_TIMEOUT))
    {
      // Таймаут разблокировки - восстанавливаем и выходим
      HAL_FLASH_Lock();
      __enable_irq();
      return;
    }
  }
  
  // ОПЕРАЦИЯ 2: Стирание страницы Flash
  FLASH_EraseInitTypeDef erase_init;
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = CONFIG_FLASH_ADDRESS;
  erase_init.NbPages = 1;
  
  uint32_t page_error = 0;
  start_time = GET_TICKS();
  flash_status = HAL_FLASHEx_Erase(&erase_init, &page_error);
  
  // Проверяем успешность стирания
  if (flash_status != HAL_OK)
  {
    // Ошибка стирания - восстанавливаем и выходим
    HAL_FLASH_Lock();
    __enable_irq();
    return;
  }
  
  // Ожидаем завершения операции стирания
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) 
  {
    if (TICKS_ELAPSED(start_time, FLASH_ERASE_TIMEOUT))
    {
      // Таймаут стирания - восстанавливаем и выходим
      HAL_FLASH_Lock();
      __enable_irq();
      return;
    }
  }
  
  // ОПЕРАЦИЯ 3: Запись структуры конфигурации во Flash
  uint32_t *src = (uint32_t*)&config;
  uint32_t *dst = (uint32_t*)CONFIG_FLASH_ADDRESS;
  uint32_t words_to_write = sizeof(Config_TypeDef) / 4;
  
  for (uint32_t i = 0; i < words_to_write; i++)
  {
    start_time = GET_TICKS();
    flash_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)dst, *src);
    
    // Проверяем успешность программирования
    if (flash_status != HAL_OK)
    {
      // Ошибка программирования - прерываем операцию
      break;
    }
    
    // Ожидаем завершения программирования
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) 
    {
      if (TICKS_ELAPSED(start_time, FLASH_PROGRAM_TIMEOUT))
      {
        // Таймаут программирования - прерываем операцию
        flash_status = HAL_ERROR;
        break;
      }
    }
    
    if (flash_status != HAL_OK) break;
    
    src++;
    dst++;
  }
  
  // ФИНАЛИЗАЦИЯ: Блокируем Flash и разрешаем прерывания
  HAL_FLASH_Lock();
  __enable_irq();
  
  // ПРОВЕРКА ЦЕЛОСТНОСТИ: После записи проверяем корректность сохраненных данных
  if (flash_status == HAL_OK)
  {
    CONFIG_Verify_Flash_Data();
    
    // Обновляем буфер предыдущих значений после успешной записи
    CONFIG_Initialize_Previous_Registers();
  }

  /* USER CODE END CONFIG_Save_1 */
}

/**
  * @brief Сброс только коммуникационных настроек к значениям по умолчанию
  * @retval None
  * @note Сбрасывает только регистры 0-4, сохраняя калибровку и служебные регистры
  *       ВЫЗЫВАЕТСЯ ПРИ АППАРАТНОМ СБРОСЕ КНОПКОЙ (3 СЕКУНДЫ)
  */
void CONFIG_Reset_Communication_Settings(void)
{
  /* USER CODE BEGIN CONFIG_Reset_Communication_Settings_1 */
  
  // Сохраняем текущие значения калибровочных коэффициентов и служебных регистров
  uint16_t saved_calib_ch1_mult = modbus_registers[MB_CALIB_CH1_MULT];
  uint16_t saved_calib_ch1_offset = modbus_registers[MB_CALIB_CH1_OFFSET];
  uint16_t saved_calib_ch2_mult = modbus_registers[MB_CALIB_CH2_MULT];
  uint16_t saved_calib_ch2_offset = modbus_registers[MB_CALIB_CH2_OFFSET];
  uint16_t saved_device_type = modbus_registers[MB_DEVICE_TYPE];
  uint16_t saved_serial_number = modbus_registers[MB_SERIAL_NUMBER];
  
  // Сбрасываем только коммуникационные регистры 0-4
  modbus_registers[MB_DEVICE_ADDR] = 100;     // Адрес по умолчанию
  modbus_registers[MB_BAUDRATE] = 0;          // 9600 бод
  modbus_registers[MB_PARITY] = 0;            // No parity
  modbus_registers[MB_STOP_BITS] = 0;         // 1 stop bit
  modbus_registers[MB_DATA_BITS] = 0;         // 8 data bits
  
  // Восстанавливаем сохраненные значения
  modbus_registers[MB_CALIB_CH1_MULT] = saved_calib_ch1_mult;
  modbus_registers[MB_CALIB_CH1_OFFSET] = saved_calib_ch1_offset;
  modbus_registers[MB_CALIB_CH2_MULT] = saved_calib_ch2_mult;
  modbus_registers[MB_CALIB_CH2_OFFSET] = saved_calib_ch2_offset;
  modbus_registers[MB_DEVICE_TYPE] = saved_device_type;
  modbus_registers[MB_SERIAL_NUMBER] = saved_serial_number;
  
  // Сбрасываем пароль доступа
  modbus_registers[MB_ACCESS_PASSWORD] = 0;
  current_access_level = ACCESS_LEVEL_0;
  
  // Применяем новые настройки UART
  MODBUS_Apply_Config();
  
  // Сохраняем изменения в Flash
  CONFIG_Save();
  
  /* USER CODE END CONFIG_Reset_Communication_Settings_1 */
}

/**
  * @brief Установка значений конфигурации по умолчанию
  * @retval None
  * @note Инициализирует ВСЕ регистры значениями по умолчанию
  *       Включая новые регистры 6 и 7
  */
void CONFIG_Set_Defaults(void)
{
  /* USER CODE BEGIN CONFIG_Set_Defaults_1 */
  // Очищаем все регистры
  memset(modbus_registers, 0, sizeof(modbus_registers));
  
  // Устанавливаем значения по умолчанию для коммуникационных параметров
  modbus_registers[MB_DEVICE_ADDR] = 100;     // Адрес по умолчанию
  modbus_registers[MB_BAUDRATE] = 0;          // 9600 бод
  modbus_registers[MB_PARITY] = 0;            // No parity
  modbus_registers[MB_STOP_BITS] = 0;         // 1 stop bit
  modbus_registers[MB_DATA_BITS] = 0;         // 8 data bits
  modbus_registers[MB_ACCESS_PASSWORD] = 0;   // Сброс пароля
  
  // Устанавливаем новые служебные регистры
  modbus_registers[MB_DEVICE_TYPE] = DEFAULT_DEVICE_TYPE;  // Код типа устройства
  modbus_registers[MB_SERIAL_NUMBER] = DEFAULT_SERIAL_NUMBER; // Серийный номер
  
  // Параметры ADC
  modbus_registers[MB_ADC_CH_SELECT] = 0;     // Канал напряжения по умолчанию
  modbus_registers[MB_ADC_POLL_PERIOD] = 10;  // Период опроса АЦП по умолчанию
  modbus_registers[MB_ADC_FILTER_DEPTH] = 10; // Глубина фильтра по умолчанию
  
  // Калибровочные коэффициенты по умолчанию (единичные)
  modbus_registers[MB_CALIB_CH1_MULT] = 1000;   // Множитель (для напряжения)
  modbus_registers[MB_CALIB_CH1_OFFSET] = 0;    // Смещение (милливольты)
  modbus_registers[MB_CALIB_CH2_MULT] = 1000;   // Множитель (для тока)
  modbus_registers[MB_CALIB_CH2_OFFSET] = 0;    // Смещение (микроамперы)
  
  // Устанавливаем уровень доступа 0
  current_access_level = ACCESS_LEVEL_0;
  
  // Обновляем структуру конфигурации в RAM
  memcpy(config.registers, modbus_registers, sizeof(config.registers));
  config.magic_number = CONFIG_MAGIC_NUMBER;
  
  // Обновляем CRC
  config.crc32 = CONFIG_Calculate_CRC();
  modbus_registers[MB_CONFIG_CRC] = config.crc32 & 0xFFFF;
  config.registers[MB_CONFIG_CRC] = modbus_registers[MB_CONFIG_CRC];
  
  // Инициализируем буфер предыдущих значений
  CONFIG_Initialize_Previous_Registers();
  
  /* USER CODE END CONFIG_Set_Defaults_1 */
}

/**
  * @brief Расчет CRC32 для структуры конфигурации
  * @retval Рассчитанное значение CRC32
  * @note Рассчитывает CRC только для регистров 0-203 как указано в ТЗ
  */
uint32_t CONFIG_Calculate_CRC(void)
{
  /* USER CODE BEGIN CONFIG_Calculate_CRC_1 */
  // Рассчитываем CRC только для регистров 0-203 (включая) как указано в ТЗ
  // ВАЖНО: config.registers содержит все 501 регистр, но CRC считаем только для первых 204
  return UTILITIES_Calculate_CRC32((uint8_t*)config.registers, 408);
  /* USER CODE END CONFIG_Calculate_CRC_1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief Проверка готовности Flash памяти к операциям
  * @retval HAL_StatusTypeDef: HAL_OK если готова, HAL_ERROR если нет
  */
static uint8_t CONFIG_Check_Flash_Ready(void)
{
  uint32_t timeout = 100; // 100ms таймаут
  uint32_t start_time = GET_TICKS();
  
  // Ожидаем снятия флага занятости Flash
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) 
  {
    if (TICKS_ELAPSED(start_time, timeout))
    {
      return HAL_ERROR; // Таймаут ожидания
    }
  }
  
  // Проверяем флаги ошибок Flash
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR) || 
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR))
  {
    // Очищаем флаги ошибок
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief Проверка целостности данных после записи во Flash
  * @retval uint8_t: 1 если данные корректны, 0 если есть ошибки
  */
static uint8_t CONFIG_Verify_Flash_Data(void)
{
  Config_TypeDef flash_config;
  uint32_t *flash_ptr = (uint32_t*)CONFIG_FLASH_ADDRESS;
  
  // Копируем данные из Flash для проверки
  memcpy(&flash_config, flash_ptr, sizeof(Config_TypeDef));
  
  // Сравниваем с исходными данными
  if (memcmp(&config, &flash_config, sizeof(Config_TypeDef)) == 0)
  {
    return 1; // Данные совпадают - запись успешна
  }
  
  return 0; // Данные не совпадают - ошибка записи
}
/* USER CODE END 1 */
