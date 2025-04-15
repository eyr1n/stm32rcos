#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32rcos/hal.hpp"

#include "uart.hpp"

namespace stm32rcos {
namespace peripheral {

template <> class UartTx<UartType::Polling> {
public:
  UartTx(UART_HandleTypeDef *huart) : huart_{huart} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Transmit(huart_, data, size, timeout) == HAL_OK;
  }

private:
  UART_HandleTypeDef *huart_;
};

template <> class UartRx<UartType::Polling> {
public:
  UartRx(UART_HandleTypeDef *huart, size_t rx_buf_size) : huart_{huart} {}

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Receive(huart_, data, size, timeout);
  }

  void flush() {}

  size_t available() { return 0; }

private:
  UART_HandleTypeDef *huart_;
};

} // namespace peripheral
} // namespace stm32rcos
