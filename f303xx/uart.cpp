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
  __builtin_unreachable();
}

#endif
