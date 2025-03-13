#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32rcos/peripheral/uart.hpp"

using stm32rcos::peripheral::UART;

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
