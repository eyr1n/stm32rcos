#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32rcos/peripheral/uart.hpp"

static stm32rcos::peripheral::UARTBase **uart_stdout() {
  static stm32rcos::peripheral::UARTBase *uart;
  return &uart;
}

void stm32rcos::peripheral::enable_stdout(
    stm32rcos::peripheral::UARTBase &uart) {
  *uart_stdout() = &uart;
}

void stm32rcos::peripheral::disable_stdout() { *uart_stdout() = nullptr; }

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