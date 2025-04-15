#include "stm32rcos/hal.hpp"

#include "macro.hpp"

#ifdef HAL_UART_MODULE_ENABLED

static void **uart_context(UART_HandleTypeDef *huart) {
#ifdef USART1
  DECLARE_CONTEXT(huart, USART1);
#endif
#ifdef USART2
  DECLARE_CONTEXT(huart, USART2);
#endif
#ifdef USART3
  DECLARE_CONTEXT(huart, USART3);
#endif
#ifdef UART4
  DECLARE_CONTEXT(huart, UART4);
#endif
#ifdef UART5
  DECLARE_CONTEXT(huart, UART5);
#endif
#ifdef USART6
  DECLARE_CONTEXT(huart, USART6);
#endif
#ifdef UART7
  DECLARE_CONTEXT(huart, UART7);
#endif
#ifdef UART8
  DECLARE_CONTEXT(huart, UART8);
#endif
  __builtin_unreachable();
}

void *stm32rcos::hal::get_uart_context(UART_HandleTypeDef *huart) {
  return *uart_context(huart);
}

void stm32rcos::hal::set_uart_context(UART_HandleTypeDef *huart,
                                      void *context) {
  *uart_context(huart) = context;
}

#endif
