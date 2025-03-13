#include "stm32_hal.h"

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
  __builtin_unreachable();
}

void *stm32rcos::get_uart_context(UART_HandleTypeDef *huart) {
  return *uart_context(huart);
}

void stm32rcos::set_uart_context(UART_HandleTypeDef *huart, void *context) {
  *uart_context(huart) = context;
}

#include "stm32rcos/peripheral/uart.hpp"

using stm32rcos::peripheral::UART;

UART &UART::get_instance(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    static UART uart1{huart};
    return uart1;
  }
  if (huart->Instance == USART2) {
    static UART uart2{huart};
    return uart2;
  }
  if (huart->Instance == USART3) {
    static UART uart3{huart};
    return uart3;
  }
  if (huart->Instance == UART4) {
    static UART uart4{huart};
    return uart4;
  }
  if (huart->Instance == UART5) {
    static UART uart5{huart};
    return uart5;
  }
  if (huart->Instance == USART6) {
    static UART uart6{huart};
    return uart6;
  }
  __builtin_unreachable();
}

#endif
