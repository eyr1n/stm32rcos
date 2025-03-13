#include "stm32rcos/hal.hpp"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32rcos/peripheral/uart.hpp"

int _write(int, char *ptr, int len) {
  auto uart = *stm32rcos::peripheral::UART::uart_stdout();
  if (uart) {
    if (uart->transmit(reinterpret_cast<uint8_t *>(ptr), len, HAL_MAX_DELAY)) {
      return len;
    }
  }
  return -1;
}

#endif
