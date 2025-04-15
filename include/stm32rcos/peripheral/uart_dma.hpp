#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

#include "uart_base.hpp"

namespace stm32rcos {
namespace peripheral {

class UART_DMA : public UARTBase {
public:
  UART_DMA(UART_HandleTypeDef *huart, size_t rx_buf_size = 64)
      : huart_{huart}, rx_buf_(rx_buf_size) {
    HAL_UART_Receive_DMA(huart, rx_buf_.data(), rx_buf_.size());
  }

  ~UART_DMA() override { HAL_UART_Abort_IT(huart_); }

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) override {
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
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

  bool receive(uint8_t *data, size_t size, uint32_t timeout) override {
    core::TimeoutHelper timeout_helper;
    while (available() < size) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      data[i] = rx_buf_[rx_read_idx_];
      advance(1);
    }
    return true;
  }

  void flush() override { advance(available()); }

  size_t available() override {
    size_t write_idx = rx_buf_.size() - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    return (rx_buf_.size() + write_idx - rx_read_idx_) % rx_buf_.size();
  }

private:
  UART_HandleTypeDef *huart_;
  std::vector<uint8_t> rx_buf_;
  size_t rx_read_idx_ = 0;

  UART_DMA(const UART_DMA &) = delete;
  UART_DMA &operator=(const UART_DMA &) = delete;

  void advance(size_t len) {
    rx_read_idx_ = (rx_read_idx_ + len) % rx_buf_.size();
  }
};

} // namespace peripheral
} // namespace stm32rcos
