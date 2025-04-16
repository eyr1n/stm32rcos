#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

#include "../uart_type.hpp"

namespace stm32rcos {
namespace peripheral {
namespace detail {

template <UartType TxType> class UartTx;
template <UartType RxType> class UartRx;

template <> class UartTx<UartType::DMA> {
public:
  UartTx(UART_HandleTypeDef *huart) : huart_{huart} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    if (HAL_UART_Transmit_DMA(huart_, data, size) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    core::TimeoutHelper timeout_helper;
    while (huart_->gState != HAL_UART_STATE_READY) {
      if (timeout_helper.is_timeout(timeout)) {
        HAL_UART_AbortTransmit_IT(huart_);
        return false;
      }
      osDelay(1);
    }
    return true;
  }

private:
  UART_HandleTypeDef *huart_;

  UartTx(const UartTx &) = delete;
  UartTx &operator=(const UartTx &) = delete;
};

template <> class UartRx<UartType::DMA> {
public:
  UartRx(UART_HandleTypeDef *huart, size_t buf_size)
      : huart_{huart}, buf_(buf_size) {
    HAL_UART_Receive_DMA(huart, buf_.data(), buf_.size());
  }

  ~UartRx() { HAL_UART_Abort_IT(huart_); }

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
    size_t write_idx = buf_.size() - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    return (buf_.size() + write_idx - read_idx_) % buf_.size();
  }

private:
  UART_HandleTypeDef *huart_;
  std::vector<uint8_t> buf_;
  size_t read_idx_ = 0;

  UartRx(const UartRx &) = delete;
  UartRx &operator=(const UartRx &) = delete;

  void advance(size_t len) { read_idx_ = (read_idx_ + len) % buf_.size(); }
};

} // namespace detail
} // namespace peripheral
} // namespace stm32rcos
