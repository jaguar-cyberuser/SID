/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    syscalls.c
  * @brief   Реализация системных вызовов для системы без SysTick (STM32F1 HAL v1.8.6)
  ******************************************************************************
  * Адаптированные системные вызовы для работы с аппаратными таймерами TIM2/TIM3/TIM4
  * вместо стандартного SysTick. Использует наши функции времени из globals.c
  * Устранена зависимость от HAL системного времени.
  ******************************************************************************
  */
/* USER CODE END Header */

#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include "stm32f1xx.h"

/* Наши собственные функции времени вместо HAL */
#include "globals.h"

/* Внешние объявления для возможного использования UART */
extern UART_HandleTypeDef huart2;

/**
  * @brief  Запись данных в файл (БЕЗ ИСПОЛЬЗОВАНИЯ HAL)
  * @param  file: дескриптор файла (1 = stdout, 2 = stderr)
  * @param  ptr:  указатель на данные для записи
  * @param  len:  количество байт для записи
  * @retval Количество записанных байт
  * @note   Простая передача без сложных циклов ожидания. Не использует нашу собственную реализацию таймаута на основе TIM4
  *         Использует короткий таймаут 10ms для передачи UART
  */
int _write(int file, char *ptr, int len)
{
    if (file == 1 || file == 2) {
        // Прямой доступ к UART без HAL
        for (int i = 0; i < len; i++) {
            // Ожидание готовности передатчика
            uint32_t timeout = GET_TICKS();
            while (!(USART2->SR & USART_SR_TXE)) {
                if (TICKS_ELAPSED(timeout, 10)) { // Таймаут 10ms
                    return i; // Возвращаем количество записанных байт
                }
            }
            USART2->DR = (uint8_t)ptr[i];
        }
        return len;
    }
    return 0;
}

/**
  * @brief  Чтение данных из файла (БЕЗ ИСПОЛЬЗОВАНИЯ HAL_MAX_DELAY)
  * @param  file: дескриптор файла
  * @param  ptr:  указатель на буфер для данных
  * @param  len:  количество байт для чтения
  * @retval Количество прочитанных байт
  * @note   Базовая реализация - всегда возвращает 0
  *         Для embedded-систем обычно не требуется ввод через stdin
  */
int _read(int file, char *ptr, int len) 
{
  /* Базовая реализация - всегда возвращает 0 */
  /* Для embedded-систем обычно не требуется ввод через stdin */
  return 0;
}

/* Остальные функции НЕ ИСПОЛЬЗУЮТ HAL tick и остаются без изменений: */

/**
  * @brief  Закрытие файла
  * @param  file: дескриптор файла
  * @retval Всегда -1 (не поддерживается)
  */
int _close(int file) 
{
  return -1;
}

/**
  * @brief  Получение статуса файла
  * @param  file: дескриптор файла
  * @param  st:   указатель на структуру stat
  * @retval Всегда 0 (успех)
  */
int _fstat(int file, struct stat *st) 
{
  st->st_mode = S_IFCHR;
  return 0;
}

/**
  * @brief  Проверка, является ли файл терминалом
  * @param  file: дескриптор файла
  * @retval Всегда 1 (да)
  */
int _isatty(int file) 
{
  return 1;
}

/**
  * @brief  Перемещение позиции в файле
  * @param  file: дескриптор файла
  * @param  ptr:  смещение
  * @param  dir:  направление
  * @retval Всегда 0 (не поддерживается)
  */
int _lseek(int file, int ptr, int dir) 
{
  return 0;
}

/**
  * @brief  Завершение программы
  * @param  status: статус завершения
  * @retval None
  */
void _exit(int status) 
{
  while (1) {
    __asm("NOP");
  }
}

/**
  * @brief  Завершение процесса
  * @param  pid: ID процесса
  * @param  sig: сигнал
  * @retval None
  */
void _kill(int pid, int sig) 
{
  return;
}

/**
  * @brief  Получение ID процесса
  * @retval Всегда 1
  */
int _getpid(void) 
{
  return 1;
}

/**
  * @brief  Создание дочернего процесса
  * @retval Всегда -1 (не поддерживается)
  */
int _fork(void) 
{
  return -1;
}

/**
  * @brief  Ожидание завершения процесса
  * @param  status: указатель на статус
  * @retval Всегда -1 (не поддерживается)
  */
int _wait(int *status) 
{
  return -1;
}

/**
  * @brief  Получение времени выполнения
  * @param  buf: указатель на структуру tms
  * @retval Всегда -1 (не поддерживается)
  */
clock_t _times(struct tms *buf) 
{
  return -1;
}

/**
  * @brief  Создание жесткой ссылки
  * @param  name: имя файла
  * @param  path: путь ссылки
  * @retval Всегда -1 (не поддерживается)
  */
int _link(const char *name, const char *path) 
{
  return -1;
}

/**
  * @brief  Удаление ссылки на файл
  * @param  name: имя файла
  * @retval Всегда -1 (не поддерживается)
  */
int _unlink(const char *name) 
{
  return -1;
}

/**
  * @brief  Получение текущего времени
  * @param  tv:  указатель на timeval
  * @param  tz:  указатель на timezone
  * @retval Всегда -1 (не поддерживается)
  */
int _gettimeofday(struct timeval *tv, void *tz) 
{
  return -1;
}

/* Управление heap - использует наши аппаратные таймеры */
extern char _end;
static char *heap_end = &_end;

/**
  * @brief  Изменение размера heap
  * @param  incr: приращение размера
  * @retval Указатель на предыдущую границу heap
  */
caddr_t _sbrk(int incr) 
{
  char *prev_heap_end;
  extern char _estack;
  extern char _Min_Stack_Size;

  prev_heap_end = heap_end;
  
  /* Проверка переполнения heap с использованием нашего системного времени */
  if (heap_end + incr > &_estack - (size_t)&_Min_Stack_Size) {
    errno = ENOMEM;
    return (caddr_t)-1;
  }
  
  heap_end += incr;
  return (caddr_t)prev_heap_end;
}
