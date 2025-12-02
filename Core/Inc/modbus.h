/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    modbus.h
  * @brief   Header for Modbus RTU Slave module
  ******************************************************************************
  * Модуль реализует протокол Modbus RTU Slave для обмена данными с устройством.
  * Поддерживает функции чтения/записи регистров, обработку ошибок и управление RS-485.
  * РЕАЛИЗОВАНА СИСТЕМА РАЗГРАНИЧЕНИЯ ДОСТУПА С 3 УРОВНЯМИ:
  * - Уровень 0: только чтение (по умолчанию)
  * - Уровень 1: специалист эксплуатации (пароль 0x55AA)
  * - Уровень 2: специалист производителя (пароль 0x77FF)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MODBUS_H
#define __MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "common_defines.h"  // Добавлено для констант доступа
/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Адреса регистров Modbus
#define MB_DEVICE_ADDR          0    // Адрес устройства Modbus
#define MB_BAUDRATE             1    // Скорость UART (0=9600, 1=19200, 2=38400, 3=57600, 4=115200)
#define MB_PARITY               2    // Четность (0=None, 1=Odd, 2=Even)
#define MB_STOP_BITS            3    // Стоп-биты (0=1, 1=2)
#define MB_DATA_BITS            4    // Биты данных (0=8, 1=7)
#define MB_ACCESS_PASSWORD      5    // Регистр ввода пароля для смены уровня доступа
#define MB_DEVICE_TYPE          6    // Код типа устройства (только для уровня 2)
#define MB_SERIAL_NUMBER        7    // Серийный номер устройства (только для уровня 2)
#define MB_ADC_CH_SELECT        10   // Выбор канала АЦП (0=Канал 1 (Ток 0-20мА), 1=Канал 2 (Напряжение 0-5В))
#define MB_ADC_POLL_PERIOD      11   // Период опроса канала АЦП (5-30 мс)
#define MB_ADC_FILTER_DEPTH     12   // Глубина фильтра АЦП (0-20, 0=фильтрация отключена)
#define MB_VREFINT_VALUE        13   // Значение VREFINT для диагностики
#define MB_ADC_RAW_VALUE        100  // Сырое значение АЦП (0-4095)
#define MB_ADC_SCALED_VALUE     101  // Откалиброванное значение (милливольты/микроамперы)
#define MB_CALIB_CH1_MULT       200  // Множитель калибровки канала 1 (напряжение)
#define MB_CALIB_CH1_OFFSET     201  // Смещение калибровки канала 1 (милливольты)
#define MB_CALIB_CH2_MULT       202  // Множитель калибровки канала 2 (ток)
#define MB_CALIB_CH2_OFFSET     203  // Смещение калибровки канала 2 (микроамперы)
#define MB_CONFIG_CRC           500  // Контрольная сумма конфигурации

#define MODBUS_REG_COUNT        501  // Общее количество регистров

// Коды функций Modbus
#define MB_FUNC_READ_HOLDING     0x03 // Чтение регистров хранения
#define MB_FUNC_WRITE_SINGLE     0x06 // Запись одиночного регистра
#define MB_FUNC_WRITE_MULTIPLE   0x10 // Запись нескольких регистров

// Исключительные коды
#define MB_EX_ILLEGAL_FUNCTION     0x01 // Неподдерживаемая функция
#define MB_EX_ILLEGAL_ADDRESS      0x02 // Неверный адрес регистра
#define MB_EX_ILLEGAL_VALUE        0x03 // Неверное значение
#define MB_EX_SLAVE_FAILURE        0x04 // Ошибка устройства

// Размеры буферов и таймауты
#define MODBUS_RX_BUFFER_SIZE   256  // Размер буфера приема
#define MODBUS_TX_BUFFER_SIZE   256  // Размер буфера передачи

// Макрос для расчета таймаута 3.5 символов в микросекундах
#define MODBUS_CHAR_TIMEOUT_US(baudrate)  (38500000UL / (baudrate))

// Объявления handle-ов
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
// Внешние переменные Modbus
extern uint16_t modbus_registers[MODBUS_REG_COUNT];      // Карта регистров Modbus
extern uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];  // Буфер приема данных

// УПРОЩЕННЫЕ ФЛАГИ СОСТОЯНИЯ - всего 2 флага
extern volatile bool modbus_frame_ready;    // Флаг получения полного фрейма
extern volatile bool modbus_tx_active;      // Флаг активной передачи

extern uint16_t modbus_rx_length;              // Длина принятого фрейма

// Внешние объявления переменных системы доступа (определены в globals.c)
extern volatile uint8_t current_access_level;  // Текущий уровень доступа
extern bool config_modified;                   // Флаг изменения конфигурации
extern bool calib_modified;                    // Флаг изменения калибровки
extern uint16_t previous_registers[MODBUS_REG_COUNT]; // Буфер предыдущих значений
/* USER CODE END EV */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
// Прототипы функций модуля Modbus
void MODBUS_Process(void);
void MODBUS_Apply_Config(void);

// Функции управления RS-485 с защитой
void MODBUS_Set_RS485_TX(void);
void MODBUS_Set_RS485_RX(void);

// Функции работы с таймерами Modbus
void MODBUS_Start_Char_Timeout(uint32_t baudrate);
void MODBUS_Stop_Char_Timeout(void);
void MODBUS_Timeout_Handler(void);

// Новые функции для динамического управления таймаутами
void MODBUS_Set_Timeout_Pulse(uint16_t width_us);
uint16_t MODBUS_Calculate_Timeout_Value(uint32_t baudrate);

// Функции системы разграничения доступа
uint8_t MODBUS_Check_Write_Permission(uint16_t reg_addr);
void MODBUS_Check_Config_Changes(void);
void MODBUS_Process_Access_Password(uint16_t password);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H */
