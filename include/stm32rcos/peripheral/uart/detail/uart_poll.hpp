#pragma once

#include <cstddef>
#include <cstdint>

#include <stm32cubemx_helper/device.hpp>

#include "../uart_type.hpp"

namespace stm32rcos {
namespace peripheral {
namespace detail {

template <auto *Handle, UartType TxType> class UartTx;
template <auto *Handle, UartType RxType> class UartRx;

template <auto *Handle> class UartTx<Handle, UartType::POLL> {
public:
  UartTx() = default;

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Transmit(Handle, data, size, timeout) == HAL_OK;
  }

private:
  UartTx(const UartTx &) = delete;
  UartTx &operator=(const UartTx &) = delete;
};

template <auto *Handle> class UartRx<Handle, UartType::POLL> {
public:
  UartRx(size_t) {}

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Receive(Handle, data, size, timeout);
  }

  void flush() {}

  size_t available() { return 0; }

private:
  UartRx(const UartRx &) = delete;
  UartRx &operator=(const UartRx &) = delete;
};

} // namespace detail
} // namespace peripheral
} // namespace stm32rcos
