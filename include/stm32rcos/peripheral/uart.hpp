#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

extern "C" int _write(int file, char *ptr, int len);

namespace stm32rcos {
namespace peripheral {

class UART {
public:
  UART(UART_HandleTypeDef *huart) : huart_{huart} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Transmit(huart_, data, size, timeout) == HAL_OK;
  }

  bool enable_stdout() {
    if (get_stdout_uart()) {
      return false;
    }
    get_stdout_uart() = this;
    return true;
  }

  bool disable_stdout() {
    if (!get_stdout_uart()) {
      return false;
    }
    get_stdout_uart() = nullptr;
    return true;
  }

private:
  UART_HandleTypeDef *huart_;

  static inline UART *&get_stdout_uart() {
    static UART *uart;
    return uart;
  }

  friend int ::_write(int file, char *ptr, int len);
};

} // namespace peripheral
} // namespace stm32rcos
