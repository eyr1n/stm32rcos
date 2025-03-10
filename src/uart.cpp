#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32rcos/peripheral/uart.hpp"

using stm32rcos::peripheral::UART;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  UART &uart = UART::get_instance(huart);
  if (uart.tx_callback_) {
    uart.tx_callback_(uart.tx_args_);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  UART &uart = UART::get_instance(huart);
  if (uart.rx_callback_) {
    uart.rx_callback_(uart.rx_args_);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  HAL_UART_Abort_IT(huart);
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
  UART &uart = UART::get_instance(huart);
  if (uart.abort_callback_) {
    uart.abort_callback_(uart.abort_args_);
  }
}

int _write(int, char *ptr, int len) {
  UART *uart = UART::get_stdout_uart();
  if (uart) {
    if (uart->transmit(reinterpret_cast<uint8_t *>(ptr), len, HAL_MAX_DELAY)) {
      return len;
    }
  }
  return -1;
}

#endif
