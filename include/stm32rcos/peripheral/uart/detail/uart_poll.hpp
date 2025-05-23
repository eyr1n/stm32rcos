#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32rcos/hal.hpp"

#include "../uart_type.hpp"

namespace stm32rcos {
namespace peripheral {
namespace detail {

template <UartType TxType> class UartTx;
template <UartType RxType> class UartRx;

template <> class UartTx<UartType::POLL> {
public:
  UartTx(UART_HandleTypeDef *huart) : huart_{huart} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Transmit(huart_, data, size, timeout) == HAL_OK;
  }

private:
  UART_HandleTypeDef *huart_;

  UartTx(const UartTx &) = delete;
  UartTx &operator=(const UartTx &) = delete;
};

template <> class UartRx<UartType::POLL> {
public:
  UartRx(UART_HandleTypeDef *huart, size_t) : huart_{huart} {}

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Receive(huart_, data, size, timeout);
  }

  void flush() {}

  size_t available() { return 0; }

private:
  UART_HandleTypeDef *huart_;

  UartRx(const UartRx &) = delete;
  UartRx &operator=(const UartRx &) = delete;
};

} // namespace detail
} // namespace peripheral
} // namespace stm32rcos
