#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <stm32cubemx_helper/device.hpp>

#include "stm32rcos/core.hpp"

#include "../uart_type.hpp"

namespace stm32rcos {
namespace peripheral {
namespace detail {

template <auto *Handle, UartType TxType> class UartTx;
template <auto *Handle, UartType RxType> class UartRx;

template <auto *Handle> class UartTx<Handle, UartType::DMA> {
public:
  UartTx() = default;

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    if (HAL_UART_Transmit_DMA(Handle, data, size) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(Handle);
      return false;
    }
    core::TimeoutHelper timeout_helper;
    while (Handle->gState != HAL_UART_STATE_READY) {
      if (timeout_helper.is_timeout(timeout)) {
        HAL_UART_AbortTransmit_IT(Handle);
        return false;
      }
      osDelay(1);
    }
    return true;
  }

private:
  UartTx(const UartTx &) = delete;
  UartTx &operator=(const UartTx &) = delete;
};

template <auto *Handle> class UartRx<Handle, UartType::DMA> {
public:
  UartRx(size_t buf_size) : buf_(buf_size) {
    HAL_UART_Receive_DMA(Handle, buf_.data(), buf_.size());
  }

  ~UartRx() { HAL_UART_Abort_IT(Handle); }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    core::TimeoutHelper timeout_helper;
    while (available() < size) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      data[i] = buf_[read_idx_];
      advance(1);
    }
    return true;
  }

  void flush() { advance(available()); }

  size_t available() {
    size_t write_idx = buf_.size() - __HAL_DMA_GET_COUNTER(Handle->hdmarx);
    return (buf_.size() + write_idx - read_idx_) % buf_.size();
  }

private:
  std::vector<uint8_t> buf_;
  size_t read_idx_ = 0;

  UartRx(const UartRx &) = delete;
  UartRx &operator=(const UartRx &) = delete;

  void advance(size_t len) { read_idx_ = (read_idx_ + len) % buf_.size(); }
};

} // namespace detail
} // namespace peripheral
} // namespace stm32rcos
