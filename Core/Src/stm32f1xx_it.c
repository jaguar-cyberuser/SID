/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * Обработчики прерываний для системы на аппаратных таймерах.
  * Устранены конфликты с HAL - удалена ручная обработка флагов.
  * TIM4 используется как единственный системный таймер.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus.h"
#include "adc.h"
#include "globals.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
// Внешние переменные из globals.c
extern volatile uint32_t system_ticks;
extern uint32_t modbus_last_rx_time;

// Внешние переменные из modbus.c
extern uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];
extern volatile bool modbus_frame_ready;
extern volatile bool modbus_tx_active;
extern uint16_t modbus_rx_length;

// Внешние переменные для ADC
extern uint16_t adc_dma_buffer[ADC_MAX_FILTER_DEPTH];

// Внешние обработчики из модулей
extern void MODBUS_Timeout_Handler(void);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* Критическая ошибка - система зависает в бесконечном цикле */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
    // Оставайтесь в цикле для отладки
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* Аппаратная ошибка - перезагрузка системы */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    NVIC_SystemReset();
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* Ошибка управления памятью - перезагрузка */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    // Ошибка управления памятью - перезагрузка
    NVIC_SystemReset();
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* Ошибка шины - перезагрузка */
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
	// Ошибка шины - перезагрузка
    NVIC_SystemReset();
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* Ошибка использования процессора - перезагрузка */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    NVIC_SystemReset();
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* Не используется в данном проекте (без ОС) */
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* Обработчик отладки - пропускаем прерывание */
  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* Не используется в данном проекте (без ОС) */
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* SysTick отключен в пользу TIM4, но оставляем для совместимости */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  /* ВСЯ РУЧНАЯ ОБРАБОТКА УДАЛЕНА - доверяем HAL */
  /* CIRCULAR DMA автоматически поддерживает непрерывные преобразования */
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
  /* ВСЯ РУЧНАЯ ОБРАБОТКА УДАЛЕНА - доверяем HAL */
  /* В NORMAL режиме HAL автоматически останавливает DMA при завершении */
  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */
  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
  /* ВСЯ РУЧНАЯ ОБРАБОТКА УДАЛЕНА - доверяем HAL */
  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  /* ВСЯ РУЧНАЯ ОБРАБОТКА ФЛАГОВ УДАЛЕНА - доверяем HAL */
  /* HAL автоматически обрабатывает EOC, AWD и другие прерывания ADC */
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  /* ВСЯ РУЧНАЯ ОБРАБОТКА ФЛАГОВ УДАЛЕНА - доверяем HAL */
  /* Обработка таймаута Modbus перенесена в HAL_TIM_PeriodElapsedCallback */
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* Проверяем флаг update interrupt */
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
  {
    /* Очищаем флаг прерывания */
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    
    // Автономное обновление фильтра АЦП
    ADC_Update_Filter();
  }
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  /* Проверяем флаг update interrupt */
  if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
  {
    /* Очищаем флаг прерывания */
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    
    /* ИНКРЕМЕНТ ЕДИНСТВЕННОГО СИСТЕМНОГО ТАЙМЕРА */
    system_ticks++;

    /* Вызов HAL инкремента тиков для совместимости */
    HAL_IncTick();
  }
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  // Обновляем время последнего принятого символа при любом прерывании UART
  modbus_last_rx_time = GET_TICKS();

  /* Handle IDLE line interrupt - detection of frame end */
  /* ЭТО ЕДИНСТВЕННАЯ РУЧНАЯ ОБРАБОТКА - уникальная логика проекта */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    /* Расчет длины принятого фрейма */
    /* DMA в NORMAL режиме останавливается после заполнения буфера */
    modbus_rx_length = MODBUS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

    /* Проверяем наличие полного Modbus фрейма (минимум 8 байт) */
    if (modbus_rx_length >= 8)
    {
      /* Устанавливаем флаг готовности фрейма для обработки в main */
      modbus_frame_ready = true;

      /* Останавливаем таймаут символа - фрейм полностью принят */
      MODBUS_Stop_Char_Timeout();
    }
    else if (modbus_rx_length > 0)
    {
      /* Неполный фрейм - запускаем таймаут для детектирования обрыва */
      MODBUS_Start_Char_Timeout(huart2.Init.BaudRate);
    }

    /* ПЕРЕЗАПУСК DMA ДЛЯ ПРИЕМА СЛЕДУЮЩЕГО ФРЕЙМА (NORMAL режим) */
    HAL_UART_DMAStop(&huart2);
    HAL_UART_Receive_DMA(&huart2, modbus_rx_buffer, MODBUS_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  }
  
  /* ВСЯ РУЧНАЯ ОБРАБОТКА ОШИБОК (ORE, NE, FE) УДАЛЕНА - доверяем HAL */
  /* HAL_UART_IRQHandler автоматически обрабатывает все стандартные ошибки UART */
  
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
