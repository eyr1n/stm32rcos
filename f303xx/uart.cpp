#include "stm32rcos/hal.hpp"

#ifdef HAL_UART_MODULE_ENABLED

static void **uart_context(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    static void *context;
    return &context;
  }
  if (huart->Instance == USART2) {
    static void *context;
    return &context;
  }
  if (huart->Instance == USART3) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

#endif
