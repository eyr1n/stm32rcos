#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"
#include "stm32rcos/peripheral.hpp"

#ifdef HAL_UART_MODULE_ENABLED

static stm32rcos::peripheral::UartBase **uart_stdout() {
  static stm32rcos::peripheral::UartBase *uart;
  return &uart;
}

bool stm32rcos::peripheral::enable_stdout(
    stm32rcos::peripheral::UartBase &uart) {
  if (*uart_stdout()) {
    return false;
  }
  *uart_stdout() = &uart;
  return true;
}

bool stm32rcos::peripheral::disable_stdout() {
  if (!*uart_stdout()) {
    return false;
  }
  *uart_stdout() = nullptr;
  return true;
}

extern "C" int _write(int, char *ptr, int len) {
  auto uart = *uart_stdout();
  if (uart) {
    if (uart->transmit(reinterpret_cast<uint8_t *>(ptr), len, osWaitForever)) {
      return len;
    }
  }
  return -1;
}

#endif