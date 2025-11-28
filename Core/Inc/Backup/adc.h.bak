/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   Header for ADC module
  ******************************************************************************
  * Модуль для работы с АЦП, включая фильтрацию данных и калибровку.
  * Использует DMA для непрерывного сбора данных и реализует фильтр скользящего среднего.
  * Период опроса АЦП настраивается через регистр Modbus MB_ADC_POLL_PERIOD (5-30 мс).
  * Циклический DMA режим обеспечивает непрерывное обновление данных без перезапуска.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#include "modbus.h"
/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Константы модуля ADC
#define ADC_FILTER_DEPTH         (modbus_registers[MB_ADC_FILTER_DEPTH]) 	// Глубина фильтра скользящего среднего из регистра Modbus
#define ADC_CHANNEL_VOLTAGE      5              							// PA5 Канал 1: Напряжение 0-5В (PA5) - номер канала ADC
#define ADC_CHANNEL_CURRENT      6              							// PA6 Канал 2: Ток 0-20мА (PA6) - номер канала ADC

// Допустимые значения периода опроса АЦП и глубины фильтра
#define ADC_MIN_POLL_PERIOD      5   // Минимальный период опроса, мс
#define ADC_MAX_POLL_PERIOD      30  // Максимальный период опроса, мс
#define ADC_DEFAULT_POLL_PERIOD  10  // Период опроса по умолчанию, мс
#define ADC_MAX_FILTER_DEPTH     20  // Максимальная глубина фильтра скользящего среднего

// Объявления handle-ов
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
// Прототипы функций модуля ADC
void ADC_Init_Start(void);
uint16_t ADC_Get_Filtered_Raw_Value(void);
int32_t ADC_Get_Scaled_Value(void);
uint8_t ADC_Get_Current_Channel(void);
void ADC_Set_Poll_Period(uint16_t period_ms);
uint16_t ADC_Read_VREFINT(void);
float ADC_Get_Actual_VREF(void);
void ADC_Update_Filter(void);

// Внешние переменные
extern uint16_t adc_dma_buffer[ADC_MAX_FILTER_DEPTH];
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
