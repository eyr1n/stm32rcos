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
  if (huart->Instance == UART4) {
    static void *context;
    return &context;
  }
  if (huart->Instance == UART5) {
    static void *context;
    return &context;
  }
  if (huart->Instance == USART6) {
    static void *context;
    return &context;
  }
  if (huart->Instance == UART7) {
    static void *context;
    return &context;
  }
  if (huart->Instance == UART8) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

void *stm32rcos::get_uart_context(UART_HandleTypeDef *huart) {
  return *uart_context(huart);
}

void stm32rcos::set_uart_context(UART_HandleTypeDef *huart, void *context) {
  *uart_context(huart) = context;
}

#endif
