#include "stm32_hal.h"

#ifdef HAL_UART_MODULE_ENABLED

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
  if (huart->Instance == UART7) {
    static UART uart7{huart};
    return uart7;
  }
  if (huart->Instance == UART8) {
    static UART uart8{huart};
    return uart8;
  }
  __builtin_unreachable();
}

#endif
