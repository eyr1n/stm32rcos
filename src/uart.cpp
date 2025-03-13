#include "stm32rcos/hal.hpp"

#ifdef HAL_UART_MODULE_ENABLED

#include "stm32rcos/peripheral/uart.hpp"

int _write(int, char *ptr, int len) {
  auto uart = *stm32rcos::peripheral::UART::uart_stdout();
  if (uart) {
    if (__get_IPSR()) { // for debug
      if (HAL_UART_Transmit(uart->huart_, reinterpret_cast<uint8_t *>(ptr), len,
                            HAL_MAX_DELAY) == HAL_OK) {
        return len;
      }
    } else {
      if (uart->transmit(reinterpret_cast<uint8_t *>(ptr), len,
                         osWaitForever)) {
        return len;
      }
    }
  }
  return -1;
}

#endif
