/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    modbus.c
  * @brief   Implementation of Modbus RTU Slave module
  ******************************************************************************
  * Модуль реализует протокол Modbus RTU Slave. Обрабатывает входящие запросы,
  * выполняет чтение/записи регистров, рассчитывает CRC и управляет линией RS-485.
  * РЕАЛИЗОВАНА СИСТЕМА РАЗГРАНИЧЕНИЯ ДОСТУПА С 3 УРОВНЯМИ:
  * - Уровень 0: только чтение (по умолчанию)
  * - Уровень 1: специалист эксплуатации (пароль 0x55AA)
  * - Уровень 2: специалист производителя (пароль 0x77FF)
  * АВТОМАТИЧЕСКАЯ ЗАПИСЬ В FLASH ПРИ ИЗМЕНЕНИИ ПАРАМЕТРОВ
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "modbus.h"
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "config.h"
#include "utilities.h"
#include "globals.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Приватные переменные модуля Modbus
extern UART_HandleTypeDef huart2;
uint16_t modbus_registers[MODBUS_REG_COUNT];      // Карта регистров Modbus в ОЗУ
uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];  // Буфер приема UART
uint8_t modbus_tx_buffer[MODBUS_TX_BUFFER_SIZE];  // Буфер передачи UART

// УПРОЩЕННЫЕ ФЛАГИ СОСТОЯНИЯ - всего 2 флага
volatile bool modbus_frame_ready = false;      // Флаг получения полного фрейма
volatile bool modbus_tx_active = false;        // Флаг активной передачи

uint16_t modbus_rx_length = 0;                   // Длина принятого фрейма

// Матрица прав доступа к регистрам [адрес_регистра] = маска_прав
static const uint8_t register_access_mask[MODBUS_REG_COUNT] = {
    [MB_DEVICE_ADDR] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,      // 0
    [MB_BAUDRATE] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,         // 1
    [MB_PARITY] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,           // 2
    [MB_STOP_BITS] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,        // 3
    [MB_DATA_BITS] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,        // 4
    [MB_ACCESS_PASSWORD] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,  // 5
    [MB_DEVICE_TYPE] = REG_LEVEL_2_WRITE,                          // 6 - только уровень 2
    [MB_SERIAL_NUMBER] = REG_LEVEL_2_WRITE,                        // 7 - только уровень 2
    [MB_ADC_CH_SELECT] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,    // 10
    [MB_ADC_POLL_PERIOD] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE,  // 11
    [MB_ADC_FILTER_DEPTH] = REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE, // 12
    [MB_VREFINT_VALUE] = REG_READ_ONLY,                            // 13 - только чтение
    [MB_ADC_RAW_VALUE] = REG_READ_ONLY,                            // 100 - только чтение
    [MB_ADC_SCALED_VALUE] = REG_READ_ONLY,                         // 101 - только чтение
    [MB_CALIB_CH1_MULT] = REG_LEVEL_2_WRITE,                       // 200 - только уровень 2
    [MB_CALIB_CH1_OFFSET] = REG_LEVEL_2_WRITE,                     // 201 - только уровень 2
    [MB_CALIB_CH2_MULT] = REG_LEVEL_2_WRITE,                       // 202 - только уровень 2
    [MB_CALIB_CH2_OFFSET] = REG_LEVEL_2_WRITE,                     // 203 - только уровень 2
    [MB_CONFIG_CRC] = REG_READ_ONLY                                // 500 - только чтение
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
// Приватные прототипы функций
static void MODBUS_Process_Request(uint8_t *request, uint16_t length);
static void MODBUS_Process_Read_Holding(uint8_t *request);
static void MODBUS_Process_Write_Single(uint8_t *request);
static void MODBUS_Process_Write_Multiple(uint8_t *request);
static void MODBUS_Send_Exception(uint8_t slave_addr, uint8_t function, uint8_t exception);
static void MODBUS_Check_Register_Changes(uint16_t start_addr, uint16_t reg_count);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief Установка ширины импульса для TIM2 (Modbus таймаут)
  * @param width_us: длительность таймаута в микросекундах
  * @retval None
  */
void MODBUS_Set_Timeout_Pulse(uint16_t width_us)
{
    // Останавливаем таймер перед настройкой
    MODBUS_Stop_Char_Timeout();
    
    // Устанавливаем новую длительность импульса
    __HAL_TIM_SET_AUTORELOAD(&htim2, width_us - 1);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    // Принудительное обновление регистров таймера
    TIM2->EGR = TIM_EGR_UG;
}

/**
  * @brief Расчет значения таймаута для заданной скорости UART
  * @param baudrate: Скорость UART в бодах
  * @retval Значение таймаута в микросекундах
  */
uint16_t MODBUS_Calculate_Timeout_Value(uint32_t baudrate)
{
    // Базовый расчет: 38500000 / baudrate
    uint32_t timeout_us = 38500000UL / baudrate;
    
    // Добавляем запас 10% для надежности
    timeout_us = timeout_us * 110 / 100;
    
    // Ограничение минимального таймаута (не менее 500us)
    if (timeout_us < 500) timeout_us = 500;
    
    // Ограничение максимального таймаута (не более 20000us)
    if (timeout_us > 20000) timeout_us = 20000;
    
    return (uint16_t)timeout_us;
}

/**
  * @brief Запуск таймаута символа для текущей скорости UART
  * @param baudrate: Скорость UART в бодах
  * @retval None
  */
void MODBUS_Start_Char_Timeout(uint32_t baudrate)
{
    uint16_t timeout_value;
    
    /* Рассчитываем таймаут в микросекундах для 3.5 символов */
    timeout_value = MODBUS_Calculate_Timeout_Value(baudrate);
    
    /* Устанавливаем новый период таймаута */
    MODBUS_Set_Timeout_Pulse(timeout_value);
    
    /* Очищаем флаги прерываний */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    
    /* Запускаем таймер в One-Pulse режиме */
    HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_1);
}

/**
  * @brief  Остановка таймаута символа
  * @retval None
  */
void MODBUS_Stop_Char_Timeout(void)
{
    /* Останавливаем таймер */
    HAL_TIM_OnePulse_Stop(&htim2, TIM_CHANNEL_1);
    
    /* Сбрасываем счетчик */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    /* Очищаем флаги прерываний */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

/**
  * @brief  Обработчик таймаута Modbus (вызывается из прерывания TIM2)
  * @retval None
  */
void MODBUS_Timeout_Handler(void)
{
    /* Рассчитываем длину принятых данных через счетчик DMA */
    modbus_rx_length = MODBUS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    
    /* Обрабатываем только неполные фреймы */
    if (modbus_rx_length > 0 && modbus_rx_length < 8)
    {
        /* Сбрасываем буфер при обрыве связи */
        modbus_rx_length = 0;
    }
    /* Полные фреймы обрабатываются сразу в IDLE прерывании */
}

/**
  * @brief Проверка прав доступа на запись в регистр
  * @param reg_addr: адрес регистра для проверки
  * @retval 0 - запись запрещена, 1 - запись разрешена
  * @note Проверяет текущий уровень доступа и права регистра
  */
uint8_t MODBUS_Check_Write_Permission(uint16_t reg_addr)
{
    // Все регистры доступны для чтения на любом уровне
    if (reg_addr >= MODBUS_REG_COUNT) {
        return 0; // Неверный адрес
    }
    
    uint8_t access_mask = register_access_mask[reg_addr];
    
    switch (current_access_level) {
        case ACCESS_LEVEL_0:
            // Уровень 0 - только чтение
            return 0;
            
        case ACCESS_LEVEL_1:
            // Уровень 1 - запись в регистры с маской REG_LEVEL_1_WRITE
            return (access_mask & REG_LEVEL_1_WRITE) ? 1 : 0;
            
        case ACCESS_LEVEL_2:
            // Уровень 2 - запись в регистры с масками REG_LEVEL_1_WRITE и REG_LEVEL_2_WRITE
            return (access_mask & (REG_LEVEL_1_WRITE | REG_LEVEL_2_WRITE)) ? 1 : 0;
            
        default:
            return 0;
    }
}

/**
  * @brief Обработка пароля доступа
  * @param password: введенный пароль
  * @retval None
  * @note Устанавливает соответствующий уровень доступа при правильном пароле
  */
void MODBUS_Process_Access_Password(uint16_t password)
{
    switch (password) {
        case PASSWORD_LEVEL_1:
            current_access_level = ACCESS_LEVEL_1;
            break;
            
        case PASSWORD_LEVEL_2:
            current_access_level = ACCESS_LEVEL_2;
            break;
            
        default:
            // Неверный пароль - сброс до уровня 0
            current_access_level = ACCESS_LEVEL_0;
            break;
    }
}

/**
  * @brief Проверка изменений в регистрах конфигурации и калибровки
  * @param start_addr: начальный адрес изменяемых регистров
  * @param reg_count: количество изменяемых регистров
  * @retval None
  * @note Устанавливает флаги изменений для последующей записи в Flash
  */
static void MODBUS_Check_Register_Changes(uint16_t start_addr, uint16_t reg_count)
{
    for (uint16_t i = start_addr; i < start_addr + reg_count; i++) {
        if (i >= MODBUS_REG_COUNT) break;
        
        // Проверяем, изменилось ли значение регистра
        if (modbus_registers[i] != previous_registers[i]) {
            
            // Определяем тип измененного регистра
            if (i >= MB_CALIB_CH1_MULT && i <= MB_CALIB_CH2_OFFSET) {
                // Калибровочные коэффициенты
                calib_modified = true;
            } else if (i <= MB_DATA_BITS || (i >= MB_ADC_CH_SELECT && i <= MB_ADC_FILTER_DEPTH)) {
                // Конфигурационные параметры (исключая пароль и служебные регистры)
                if (i != MB_ACCESS_PASSWORD && i != MB_DEVICE_TYPE && i != MB_SERIAL_NUMBER) {
                    config_modified = true;
                }
            } else if (i == MB_DEVICE_TYPE || i == MB_SERIAL_NUMBER) {
                // Служебные регистры (только для уровня 2)
                config_modified = true;
            }
            
            // Обновляем предыдущее значение
            previous_registers[i] = modbus_registers[i];
        }
    }
}

/**
  * @brief Проверка и сохранение изменений конфигурации
  * @retval None
  * @note Вызывается после обработки Modbus запросов для записи изменений в Flash
  */
void MODBUS_Check_Config_Changes(void)
{
    if (config_modified) {
        CONFIG_Save();
        config_modified = false;
    }
    
    if (calib_modified) {
        CONFIG_Save();
        calib_modified = false;
    }
}

/**
  * @brief Установка направления RS-485 на передачу
  * @retval None
  */
void MODBUS_Set_RS485_TX(void)
{
    /* Устанавливаем флаги состояния передачи */
    modbus_tx_active = true;
    
    /* Включаем передатчик RS-485 */
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    
    /* Неблокирующая задержка стабилизации */
    UTILITIES_Delay_US(10);
}

/**
  * @brief Установка направления RS-485 на прием
  * @retval None
  */
void MODBUS_Set_RS485_RX(void)
{
    /* Неблокирующая задержка перед переключением */
    UTILITIES_Delay_US(10);
    
    /* Выключаем передатчик RS-485 */
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    
    /* Устанавливаем флаги состояния передачи */
    modbus_tx_active = false;
}

/**
  * @brief Основная функция обработки Modbus
  * @retval None
  */
void MODBUS_Process(void)
{
    static bool processing = false;
    static uint32_t last_frame_time = 0;
    uint16_t current_frame_length;
    
    // Проверяем наличие фрейма и отсутствие активной обработки
    if (modbus_frame_ready && !processing)
    {
        // Защита от слишком частой обработки (минимум 1ms между фреймами)
        if (!TICKS_ELAPSED(last_frame_time, 1)) {
            return;
        }

        // Устанавливаем флаг занятости для защиты от реентерабельности
        processing = true;

        // Сохраняем длину фрейма перед обработкой
        current_frame_length = modbus_rx_length;
        
        // НЕМЕДЛЕННО сбрасываем флаг получения фрейма
        modbus_frame_ready = false;
        
        // Обрабатываем Modbus запрос
        MODBUS_Process_Request(modbus_rx_buffer, current_frame_length);
        
        // Проверяем и сохраняем изменения конфигурации
        MODBUS_Check_Config_Changes();
        
        // Запоминаем время обработки
        last_frame_time = GET_TICKS();
        
        // Сбрасываем флаг занятости
        processing = false;
    }
}

/**
  * @brief Применение конфигурации UART из регистров Modbus
  * @retval None
  */
void MODBUS_Apply_Config(void)
{
    UART_InitTypeDef uart_config = {0};
    uint16_t timeout_value;
    uint32_t baudrate;
    
    // Останавливаем UART перед перенастройкой
    HAL_UART_DeInit(&huart2);
    
    // Базовая конфигурация
    uart_config.Mode = UART_MODE_TX_RX;
    uart_config.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_config.OverSampling = UART_OVERSAMPLING_16;
    
    // Настройка скорости из регистра MB_BAUDRATE
    switch (modbus_registers[MB_BAUDRATE])
    {
        case 1:
            baudrate = 19200;
            break;
        case 2:
            baudrate = 38400;
            break;
        case 3:
            baudrate = 57600;
            break;
        case 4:
            baudrate = 115200;
            break;
        default:
            baudrate = 9600;  // По умолчанию
            break;
    }
    uart_config.BaudRate = baudrate;
    
    // Настройка битов данных из регистра MB_DATA_BITS
    if (modbus_registers[MB_DATA_BITS] == 1)
    {
        uart_config.WordLength = UART_WORDLENGTH_9B;
    }
    else
    {
        uart_config.WordLength = UART_WORDLENGTH_8B;  // По умолчанию
    }
    
    // Настройка четности из регистра MB_PARITY
    switch (modbus_registers[MB_PARITY])
    {
        case 1:
            uart_config.Parity = UART_PARITY_ODD;
            break;
        case 2:
            uart_config.Parity = UART_PARITY_EVEN;
            break;
        default:
            uart_config.Parity = UART_PARITY_NONE;  // По умолчанию
            break;
    }
    
    // Настройка стоп-битов из регистра MB_STOP_BITS
    if (modbus_registers[MB_STOP_BITS] == 1)
    {
        uart_config.StopBits = UART_STOPBITS_2;
    }
    else
    {
        uart_config.StopBits = UART_STOPBITS_1;  // По умолчанию
    }
    
    // Применяем новую конфигурацию UART
    huart2.Init = uart_config;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Настройка таймаута Modbus для новой скорости
    timeout_value = MODBUS_Calculate_Timeout_Value(baudrate);
    MODBUS_Set_Timeout_Pulse(timeout_value);
    
    // ЗАПУСК NORMAL DMA ПРИЕМА после перенастройки UART
    HAL_UART_Receive_DMA(&huart2, modbus_rx_buffer, MODBUS_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/**
  * @brief Обработка Modbus-запроса
  * @param request: указатель на буфер запроса
  * @param length: длина запроса
  * @retval None
  */
static void MODBUS_Process_Request(uint8_t *request, uint16_t length)
{
    uint8_t slave_addr = request[0];
    uint8_t function = request[1];
    uint16_t crc_received, crc_calculated;
    
    // Проверяем адрес устройства (0 - broadcast адрес)
    if (slave_addr != modbus_registers[MB_DEVICE_ADDR] && slave_addr != 0)
    {
        return; // Запрос не для нас
    }
    
    // Проверяем CRC
    crc_received = (request[length-2] << 8) | request[length-1];
    crc_calculated = UTILITIES_Calculate_CRC16(request, length-2);
    
    if (crc_received != crc_calculated)
    {
        return; // CRC не совпадает
    }
    
    // Обрабатываем функцию
    switch (function)
    {
        case MB_FUNC_READ_HOLDING:
            MODBUS_Process_Read_Holding(request);
            break;
            
        case MB_FUNC_WRITE_SINGLE:
            MODBUS_Process_Write_Single(request);
            break;
            
        case MB_FUNC_WRITE_MULTIPLE:
            MODBUS_Process_Write_Multiple(request);
            break;
            
        default:
            MODBUS_Send_Exception(slave_addr, function, MB_EX_ILLEGAL_FUNCTION);
            break;
    }
}

/**
  * @brief Обработка функции 0x03 (Read Holding Registers)
  * @param request: указатель на буфер запроса
  * @retval None
  */
static void MODBUS_Process_Read_Holding(uint8_t *request)
{
    uint8_t slave_addr = request[0];
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t reg_count = (request[4] << 8) | request[5];
    uint16_t response_length;
    
    // Проверяем корректность адреса и количества регистров
    if (start_addr + reg_count > MODBUS_REG_COUNT)
    {
        MODBUS_Send_Exception(slave_addr, MB_FUNC_READ_HOLDING, MB_EX_ILLEGAL_ADDRESS);
        return;
    }
    
    // Обновляем значения регистров ADC перед отправкой если они запрашиваются
    if (start_addr <= MB_ADC_RAW_VALUE && (start_addr + reg_count) > MB_ADC_RAW_VALUE)
    {
        modbus_registers[MB_ADC_RAW_VALUE] = ADC_Get_Filtered_Raw_Value();
    }
    
    if (start_addr <= MB_ADC_SCALED_VALUE && (start_addr + reg_count) > MB_ADC_SCALED_VALUE)
    {
        int32_t scaled_value = ADC_Get_Scaled_Value();
        modbus_registers[MB_ADC_SCALED_VALUE] = scaled_value & 0xFFFF;
    }
    
    if (start_addr <= MB_VREFINT_VALUE && (start_addr + reg_count) > MB_VREFINT_VALUE)
    {
        modbus_registers[MB_VREFINT_VALUE] = ADC_Read_VREFINT();
    }

    // Формируем ответ
    modbus_tx_buffer[0] = slave_addr;
    modbus_tx_buffer[1] = MB_FUNC_READ_HOLDING;
    modbus_tx_buffer[2] = reg_count * 2;
    
    for (int i = 0; i < reg_count; i++)
    {
        modbus_tx_buffer[3 + i*2] = (modbus_registers[start_addr + i] >> 8) & 0xFF;
        modbus_tx_buffer[4 + i*2] = modbus_registers[start_addr + i] & 0xFF;
    }
    
    response_length = 3 + reg_count * 2;
    uint16_t crc = UTILITIES_Calculate_CRC16(modbus_tx_buffer, response_length);
    modbus_tx_buffer[response_length] = (crc >> 8) & 0xFF;
    modbus_tx_buffer[response_length + 1] = crc & 0xFF;
    
    // Отправляем ответ
    MODBUS_Set_RS485_TX();
    HAL_UART_Transmit_DMA(&huart2, modbus_tx_buffer, response_length + 2);
}

/**
  * @brief Обработка функции 0x06 (Write Single Register)
  * @param request: указатель на буфер запроса
  * @retval None
  */
static void MODBUS_Process_Write_Single(uint8_t *request)
{
    uint8_t slave_addr = request[0];
    uint16_t reg_addr = (request[2] << 8) | request[3];
    uint16_t reg_value = (request[4] << 8) | request[5];
    uint16_t response_length;
    
    // Проверяем корректность адреса регистра
    if (reg_addr >= MODBUS_REG_COUNT)
    {
        MODBUS_Send_Exception(slave_addr, MB_FUNC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDRESS);
        return;
    }
    
    // Особые обработки для регистра пароля доступа
    if (reg_addr == MB_ACCESS_PASSWORD)
    {
        // Обработка пароля доступа
        MODBUS_Process_Access_Password(reg_value);
        
        // Сбрасываем значение пароля в регистре для безопасности
        modbus_registers[MB_ACCESS_PASSWORD] = 0;
    }
    else
    {
        // Проверяем права доступа для обычных регистров
        if (!MODBUS_Check_Write_Permission(reg_addr))
        {
            MODBUS_Send_Exception(slave_addr, MB_FUNC_WRITE_SINGLE, MB_EX_ILLEGAL_ADDRESS);
            return;
        }
        
        // Сохраняем предыдущее значение для обнаружения изменений
        uint16_t old_value = modbus_registers[reg_addr];
        
        // Записываем новое значение
        modbus_registers[reg_addr] = reg_value;
        
        // Проверяем изменения для данного регистра
        MODBUS_Check_Register_Changes(reg_addr, 1);
        
        // Особые обработки для служебных регистров
        if (reg_addr == MB_ADC_CH_SELECT)
        {
            // Смена канала АЦП требует перезапуска преобразования
            if (old_value != reg_value) {
                ADC_Init_Start();  // Перезапускаем АЦП с новым каналом
            }
        }
        else if (reg_addr == MB_ADC_POLL_PERIOD)
        {
            // Установка периода опроса АЦП
            if (old_value != reg_value) {
                ADC_Set_Poll_Period(reg_value); // Устанавливаем новый период
            }
        }
        else if (reg_addr == MB_ADC_FILTER_DEPTH)
        {
            // Ограничиваем значение глубины фильтра
            if (reg_value > 20) {
                modbus_registers[reg_addr] = 20;
                reg_value = 20;
            }
        }
    }
    
    // Формируем ответ (эхо запроса)
    modbus_tx_buffer[0] = slave_addr;
    modbus_tx_buffer[1] = MB_FUNC_WRITE_SINGLE;
    modbus_tx_buffer[2] = (reg_addr >> 8) & 0xFF;
    modbus_tx_buffer[3] = reg_addr & 0xFF;
    modbus_tx_buffer[4] = (modbus_registers[reg_addr] >> 8) & 0xFF;
    modbus_tx_buffer[5] = modbus_registers[reg_addr] & 0xFF;
    
    response_length = 6;
    uint16_t crc = UTILITIES_Calculate_CRC16(modbus_tx_buffer, response_length);
    modbus_tx_buffer[response_length] = (crc >> 8) & 0xFF;
    modbus_tx_buffer[response_length + 1] = crc & 0xFF;
    
    // Отправляем ответ
    MODBUS_Set_RS485_TX();
    HAL_UART_Transmit_DMA(&huart2, modbus_tx_buffer, response_length + 2);
}

/**
  * @brief Обработка функции 0x10 (Write Multiple Registers)
  * @param request: указатель на буфер запроса
  * @retval None
  */
static void MODBUS_Process_Write_Multiple(uint8_t *request)
{
    uint8_t slave_addr = request[0];
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t reg_count = (request[4] << 8) | request[5];
    uint8_t byte_count = request[6];
    uint16_t response_length;
    
    // Проверяем корректность адреса и количества регистров
    if (start_addr + reg_count > MODBUS_REG_COUNT)
    {
        MODBUS_Send_Exception(slave_addr, MB_FUNC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDRESS);
        return;
    }
    
    // Проверяем соответствие количества байт
    if (byte_count != reg_count * 2)
    {
        MODBUS_Send_Exception(slave_addr, MB_FUNC_WRITE_MULTIPLE, MB_EX_ILLEGAL_VALUE);
        return;
    }
    
    // Проверяем права доступа для всех регистров перед записью
    for (int i = 0; i < reg_count; i++)
    {
        uint16_t current_addr = start_addr + i;
        
        // Пропускаем проверку для регистра пароля (особая обработка)
        if (current_addr == MB_ACCESS_PASSWORD) {
            continue;
        }
        
        if (!MODBUS_Check_Write_Permission(current_addr))
        {
            MODBUS_Send_Exception(slave_addr, MB_FUNC_WRITE_MULTIPLE, MB_EX_ILLEGAL_ADDRESS);
            return;
        }
    }

    // Записываем данные в регистры
    for (int i = 0; i < reg_count; i++)
    {
        uint16_t value = (request[7 + i*2] << 8) | request[8 + i*2];
        uint16_t current_addr = start_addr + i;
        
        // Особые обработки для регистра пароля доступа
        if (current_addr == MB_ACCESS_PASSWORD)
        {
            MODBUS_Process_Access_Password(value);
            // Сбрасываем значение пароля в регистре для безопасности
            modbus_registers[current_addr] = 0;
        }
        else
        {
            // Обычная запись регистра
            modbus_registers[current_addr] = value;
        }
    }
    
    // Проверяем изменения во всех записанных регистрах
    MODBUS_Check_Register_Changes(start_addr, reg_count);
    
    // Особые обработки после записи всех регистров
    for (int i = 0; i < reg_count; i++)
    {
        uint16_t current_addr = start_addr + i;
        
        if (current_addr == MB_ADC_CH_SELECT)
        {
            // Перезапуск АЦП при изменении канала
            ADC_Init_Start();
        }
        else if (current_addr == MB_ADC_POLL_PERIOD)
        {
            // Установка нового периода опроса АЦП
            ADC_Set_Poll_Period(modbus_registers[MB_ADC_POLL_PERIOD]);
        }
        else if (current_addr == MB_ADC_FILTER_DEPTH)
        {
            // Ограничение глубины фильтра
            if (modbus_registers[current_addr] > 20) {
                modbus_registers[current_addr] = 20;
            }
        }
    }
    
    // Формируем ответ (подтверждение записи)
    modbus_tx_buffer[0] = slave_addr;
    modbus_tx_buffer[1] = MB_FUNC_WRITE_MULTIPLE;
    modbus_tx_buffer[2] = (start_addr >> 8) & 0xFF;
    modbus_tx_buffer[3] = start_addr & 0xFF;
    modbus_tx_buffer[4] = (reg_count >> 8) & 0xFF;
    modbus_tx_buffer[5] = reg_count & 0xFF;
    
    response_length = 6;
    uint16_t crc = UTILITIES_Calculate_CRC16(modbus_tx_buffer, response_length);
    modbus_tx_buffer[response_length] = (crc >> 8) & 0xFF;
    modbus_tx_buffer[response_length + 1] = crc & 0xFF;
    
    // Отправляем ответ
    MODBUS_Set_RS485_TX();
    HAL_UART_Transmit_DMA(&huart2, modbus_tx_buffer, response_length + 2);
}

/**
  * @brief Отправка исключительного ответа Modbus
  * @param slave_addr: адрес устройства
  * @param function: код функции
  * @param exception: код исключения
  * @retval None
  */
static void MODBUS_Send_Exception(uint8_t slave_addr, uint8_t function, uint8_t exception)
{
    uint16_t crc;
    
    // Формируем исключительный ответ
    modbus_tx_buffer[0] = slave_addr;
    modbus_tx_buffer[1] = function | 0x80; // Устанавливаем старший бит для исключения
    modbus_tx_buffer[2] = exception;
    
    // Расчет CRC
    crc = UTILITIES_Calculate_CRC16(modbus_tx_buffer, 3);
    modbus_tx_buffer[3] = (crc >> 8) & 0xFF;
    modbus_tx_buffer[4] = crc & 0xFF;
    
    // Отправляем ответ
    MODBUS_Set_RS485_TX();
    HAL_UART_Transmit_DMA(&huart2, modbus_tx_buffer, 5);
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Callback завершения передачи UART
  * @param  huart: указатель на handle UART
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Transmission complete - switch back to receive mode for RS-485 */
    MODBUS_Set_RS485_RX();

    /* DMA в NORMAL режиме автоматически останавливается после передачи */
  }
}

/**
  * @brief  Callback ошибки UART
  * @param  huart: указатель на handle UART
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* При ЛЮБОЙ ошибке гарантированно переключаемся на прием */
    MODBUS_Set_RS485_RX();

    /* Перезапуск DMA приема в NORMAL режиме */
    HAL_UART_Receive_DMA(huart, modbus_rx_buffer, MODBUS_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  }
}

/* USER CODE END 1 */
