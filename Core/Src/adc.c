/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   Implementation of ADC module
  ******************************************************************************
  * Модуль для работы с АЦП. Реализует:
  * - Инициализацию АЦП с DMA в циклическом режиме
  * - Фильтрацию данных методом скользящего среднего
  * - Калибровку сырых значений АЦП в физические величины
  * - Автоматическое переключение каналов по конфигурации Modbus
  * - Динамическое изменение периода опроса через TIM3
  * - Для токового канала (канал 2) значение отображается в микроамперах
  * - Смещение для токового канала также в микроамперах
  * Циклический DMA обеспечивает непрерывное обновление данных без перезапуска.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"
/* USER CODE BEGIN Includes */
#include "globals.h"  // Для GET_TICKS() и TICKS_ELAPSED()
#include "config.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

// Буфер DMA для хранения выборок АЦП (циклический режим)
uint16_t adc_dma_buffer[ADC_MAX_FILTER_DEPTH];

// Автономный фильтр скользящего среднего - ДОБАВЛЕНО
static uint32_t filter_running_sum = 0;
static uint16_t filter_sample_buffer[ADC_MAX_FILTER_DEPTH];
static uint8_t filter_write_index = 0;
static uint8_t filter_samples_collected = 0;
static uint8_t filter_current_depth = 10; // Значение по умолчанию
static uint16_t current_filtered_value = 0;
static bool filter_initialized = false;

// Текущий активный канал АЦП
static uint8_t current_adc_channel = ADC_CHANNEL_VOLTAGE;

// Флаг инициализации АЦП
static bool adc_initialized = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Чтение внутреннего источника опорного напряжения (VREFINT)
  * @retval Сырое значение VREFINT
  * @note   VREFINT имеет номинальное значение 1.2V
  *         Для STM32F1 канал VREFINT = ADC_CHANNEL_17
  */
uint16_t ADC_Read_VREFINT(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t vrefint_value = 0;

    // Временно останавливаем DMA для ручного чтения VREFINT
    HAL_ADC_Stop_DMA(&hadc1);

    // Настраиваем канал VREFINT (ADC_CHANNEL_VREFINT = 17 для STM32F1)
    sConfig.Channel = ADC_CHANNEL_17;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // Увеличиваем время выборки для точности

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        goto restore_adc;
    }

    // Запускаем преобразование
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        vrefint_value = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

restore_adc:
    // Восстанавливаем оригинальный канал
    if (current_adc_channel == ADC_CHANNEL_VOLTAGE)
    {
        sConfig.Channel = ADC_CHANNEL_5;
    }
    else
    {
        sConfig.Channel = ADC_CHANNEL_6;
    }
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Перезапускаем DMA с оригинальным каналом
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_MAX_FILTER_DEPTH);

    return vrefint_value;
}

/**
  * @brief  Получение фактического значения опорного напряжения
  * @retval Фактическое опорное напряжение в милливольтах
  * @note   Использует известное значение VREFINT (1.2V) для расчета
  */
float ADC_Get_Actual_VREF(void)
{
    static float actual_vref = 3300.0; // Значение по умолчанию
    static uint32_t last_vref_read = 0;

    // Читаем VREFINT не чаще чем раз в 30 секунд (чтобы не мешать основным измерениям)
    if (TICKS_ELAPSED(last_vref_read, 30000))
    {
        uint16_t vrefint_raw = ADC_Read_VREFINT();

        if (vrefint_raw > 100 && vrefint_raw < 4095) // Валидный диапазон
        {
            // Расчет фактического VREF: VREF_actual = (VREFINT_nominal * 4095) / VREFINT_raw
            // VREFINT_nominal = 1.2V = 1200mV
            actual_vref = (1200.0f * 4095.0f) / (float)vrefint_raw;
            last_vref_read = GET_TICKS();

            // Для отладки можно обновить регистр Modbus
            modbus_registers[MB_VREFINT_VALUE] = vrefint_raw;
        }
    }

    return actual_vref;
}

/**
  * @brief Инициализация и запуск АЦП с DMA в циклическом режиме
  * @retval None
  */
void ADC_Init_Start(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Останавливаем АЦП и DMA перед перенастройкой канала
    if (adc_initialized)
    {
        HAL_ADC_Stop_DMA(&hadc1);
    }
    
    // СБРОС состояния фильтра при смене канала
    filter_running_sum = 0;
    filter_write_index = 0;
    filter_samples_collected = 0;
    filter_initialized = false;

    // Определяем текущий канал из регистра Modbus
    current_adc_channel = (modbus_registers[MB_ADC_CH_SELECT] == 0) ? 
                          ADC_CHANNEL_VOLTAGE : ADC_CHANNEL_CURRENT;
    
    // Настраиваем канал АЦП
    sConfig.Channel = (current_adc_channel == ADC_CHANNEL_VOLTAGE) ? 5 : 6;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    // ИНИЦИАЛИЗАЦИЯ DMA буфера нулями
    for (int i = 0; i < ADC_MAX_FILTER_DEPTH; i++)
    {
        adc_dma_buffer[i] = 0;
    }

    // Запускаем АЦП с DMA в ЦИКЛИЧЕСКОМ РЕЖИМЕ
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_MAX_FILTER_DEPTH) != HAL_OK)
    {
        Error_Handler();
    }
    
    adc_initialized = true;
}

/**
  * @brief Установка периода опроса канала АЦП
  * @param period_ms: период опроса в миллисекундах (5-30 мс)
  * @retval None
  * @note Динамически изменяет период TIM3, который запускает преобразования АЦП
  *       через TRGO. TIM3 работает в режиме Master для аппаратного триггера АЦП.
  */
void ADC_Set_Poll_Period(uint16_t period_ms)
{
    /* USER CODE BEGIN ADC_Set_Poll_Period_1 */
    
    // Ограничение периода в диапазоне 5-30 мс
    if (period_ms < ADC_MIN_POLL_PERIOD) 
    {
        period_ms = ADC_MIN_POLL_PERIOD;
    }
    else if (period_ms > ADC_MAX_POLL_PERIOD) 
    {
        period_ms = ADC_MAX_POLL_PERIOD;
    }
    
    // Обновляем значение в регистре Modbus
    modbus_registers[MB_ADC_POLL_PERIOD] = period_ms;
    
    // Останавливаем таймер перед перенастройкой
    HAL_TIM_Base_Stop_IT(&htim3);
    
    // Устанавливаем новый период для TIM3
    // Period = period_ms - 1 (счет от 0 до Period включительно)
    __HAL_TIM_SET_AUTORELOAD(&htim3, period_ms - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    
    // Принудительное обновление регистров таймера
    TIM3->EGR = TIM_EGR_UG;
    
    // Перезапускаем таймер
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* USER CODE END ADC_Set_Poll_Period_1 */
}

/**
  * @brief  Автономное обновление фильтра скользящего среднего для CIRCULAR DMA
  * @retval None
  * @note   Адаптировано для работы с CIRCULAR DMA буфером
  * 		Вызывается при получении новых данных АЦП, работает независимо от Modbus
  *         Использует кольцевой буфер ADC, где данные постоянно обновляются
  */
void ADC_Update_Filter(void)
{
    uint16_t new_sample;
    uint32_t primask;
    
    // Получаем текущую глубину фильтра из регистра Modbus
    uint8_t target_depth = modbus_registers[MB_ADC_FILTER_DEPTH];
    if (target_depth > ADC_MAX_FILTER_DEPTH) {
        target_depth = ADC_MAX_FILTER_DEPTH;
    }

    // Обнаружение изменения глубины фильтра - сброс состояния
    if (target_depth != filter_current_depth && filter_initialized)
    {
        filter_running_sum = 0;
        filter_write_index = 0;
        filter_samples_collected = 0;
        filter_current_depth = target_depth;

        // Инициализация буфера нулями
        for (int i = 0; i < ADC_MAX_FILTER_DEPTH; i++)
        {
            filter_sample_buffer[i] = 0;
        }
    }

    // КРИТИЧЕСКАЯ СЕКЦИЯ: Запрещаем прерывания на время чтения DMA буфера
    primask = __get_PRIMASK();
    __disable_irq();

    // В CIRCULAR DMA последнее значение всегда в adc_dma_buffer[0]
    // DMA автоматически перезаписывает буфер по кругу
    new_sample = adc_dma_buffer[0];

    // Восстанавливаем состояние прерываний
    __set_PRIMASK(primask);

    // Валидация значения ADC
    if (new_sample > 4095) {
        return; // Игнорируем некорректные значения
    }

    // Если фильтрация отключена (глубина = 0)
    if (filter_current_depth == 0) {
        current_filtered_value = new_sample;
        filter_initialized = true;
        return;
    }
    
    // АЛГОРИТМ СКОЛЬЗЯЩЕГО СРЕДНЕГО ДЛЯ CIRCULAR DMA

    if (filter_samples_collected < filter_current_depth)
    {
        // Фаза начального накопления - заполняем буфер
        filter_sample_buffer[filter_write_index] = new_sample;
        filter_running_sum += new_sample;
        filter_samples_collected++;
        filter_write_index++;

        // Если заполнили буфер, переходим в циклический режим
        if (filter_samples_collected == filter_current_depth)
        {
            filter_write_index = 0; // Следующая запись будет в начало
        }

        // Временное значение - среднее по накопленным samples
        current_filtered_value = filter_running_sum / filter_samples_collected;
    }
    else
    {
        // Фаза скользящего среднего - полный буфер

        // Сохраняем самое старое значение для вычитания
        uint16_t oldest_sample = filter_sample_buffer[filter_write_index];

        // Заменяем самое старое значение новым
        filter_sample_buffer[filter_write_index] = new_sample;

        // Обновляем running sum: вычитаем старое, добавляем новое
        filter_running_sum = filter_running_sum - oldest_sample + new_sample;

        // Перемещаем индекс записи по кругу
        filter_write_index = (filter_write_index + 1) % filter_current_depth;

        // Вычисляем новое отфильтрованное значение
        current_filtered_value = filter_running_sum / filter_current_depth;
    }
    
    // Обновляем регистры Modbus с отфильтрованными значениями
    modbus_registers[MB_ADC_RAW_VALUE] = current_filtered_value;

    filter_initialized = true;
}

/**
  * @brief  Получение последнего сырого значения из CIRCULAR DMA буфера
  * @retval Последнее значение ADC (0-4095)
  * @note   Безопасное чтение из DMA буфера с защитой от прерываний
  */
uint16_t ADC_Get_Latest_Raw_Value(void)
{
    uint16_t raw_value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    raw_value = adc_dma_buffer[0]; // Всегда последнее значение в CIRCULAR DMA
    __set_PRIMASK(primask);

    return raw_value;
}

/**
  * @brief  Получение текущего отфильтрованного значения АЦП
  * @retval Отфильтрованное значение АЦП (0-4095)
  * @note   Возвращает заранее рассчитанное значение из автономного фильтра
  */
uint16_t ADC_Get_Filtered_Raw_Value(void)
{
    // Если фильтр еще не инициализирован, возвращаем текущее значение из DMA
    if (!filter_initialized) {
        return ADC_Get_Latest_Raw_Value();
    }

    return current_filtered_value;
}

/**
  * @brief Получение откалиброванного значения в физических единицах
  * @retval Откалиброванное значение в милливольтах (канал 1) или микроамперах (канал 2)
  * @note Использует VREFINT для точного преобразования
  */
int32_t ADC_Get_Scaled_Value(void)
{
    /* USER CODE BEGIN ADC_Get_Scaled_Value_1 */
    uint16_t raw_value = ADC_Get_Filtered_Raw_Value();
    int32_t scaled_value;
    int32_t physical_value;

    // Получаем фактическое опорное напряжение через VREFINT
    float actual_vref = ADC_Get_Actual_VREF();

    // Преобразование сырого значения АЦП в милливольты
    // V_measured = (raw_value * VREF_actual) / 4095
    uint32_t voltage_mv = ((uint32_t)raw_value * (uint32_t)actual_vref) / 4095UL;
    
    // Применяем калибровочные коэффициенты в зависимости от активного канала
    if (current_adc_channel == ADC_CHANNEL_VOLTAGE)
    {
        // Канал 1: Напряжение 0-5V через делитель
        // Предполагаем делитель 5V -> 3.3V: V_input = V_measured * (R1+R2)/R2
        // Для делителя 5V->3.3V: коэффициент = 5000/3300 ≈ 1.515
        physical_value = (voltage_mv * 5000UL) / 3300UL;

        // Применяем калибровку: scaled = (physical * mult / 1000) + offset
        scaled_value = ((int32_t)physical_value * modbus_registers[MB_CALIB_CH1_MULT]) / 1000;
        scaled_value += (int32_t)modbus_registers[MB_CALIB_CH1_OFFSET];
    }  
    else
    {
        // Канал 2: Ток 0-20mA через шунт
        // Предполагаем: 20mA -> 3.3V, I_uA = V_measured * 20000 / 3300
        // 20mA = 20000 микроампер
        physical_value = (voltage_mv * 20000UL) / 3300UL;

        // Применяем калибровку: scaled = (physical * mult / 1000) + offset
        scaled_value = ((int32_t)physical_value * modbus_registers[MB_CALIB_CH2_MULT]) / 1000;
        scaled_value += (int32_t)modbus_registers[MB_CALIB_CH2_OFFSET];
    }
    
    return scaled_value;
    /* USER CODE END ADC_Get_Scaled_Value_1 */
}

/**
  * @brief Получение текущего активного канала АЦП
  * @retval Текущий активный канал АЦП
  * @note Возвращает ADC_CHANNEL_VOLTAGE или ADC_CHANNEL_CURRENT
  *       Используется для диагностики и отладки
  */
uint8_t ADC_Get_Current_Channel(void)
{
    /* USER CODE BEGIN ADC_Get_Current_Channel_1 */
    return current_adc_channel;
    /* USER CODE END ADC_Get_Current_Channel_1 */
}


/* USER CODE BEGIN 1 */

/**
  * @brief  Callback ошибки ADC
  * @param  hadc: указатель на handle ADC
  * @retval None
  * @note   Автоматический перезапуск ADC в CIRCULAR режиме при ошибках
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* Handle ADC error - restart conversion в ЦИКЛИЧЕСКОМ РЕЖИМЕ */
    HAL_ADC_Stop_DMA(hadc);
    HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_dma_buffer, ADC_MAX_FILTER_DEPTH);
  }
}

/* USER CODE END 1 */
