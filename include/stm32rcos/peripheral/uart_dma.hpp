#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

extern "C" int _write(int file, char *ptr, int len);

namespace stm32rcos {
namespace peripheral {

class UART_DMA {
public:
  UART_DMA(UART_HandleTypeDef *huart, size_t rx_buf_size = 64)
      : huart_{huart}, rx_buf_(rx_buf_size) {
    set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_TX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UART_DMA *>(get_uart_context(huart));
          uart->tx_sem_.release();
        });
    HAL_UART_Receive_DMA(huart, rx_buf_.data(), rx_buf_.size());
  }

  ~UART_DMA() {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
    set_uart_context(huart_, nullptr);
  }

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    tx_sem_.try_acquire(0);
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    if (!tx_sem_.try_acquire(timeout)) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    return true;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    core::TimeoutHelper timeout_helper;
    while (rx_available() < size) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      data[i] = rx_buf_[rx_read_idx_];
      rx_advance(1);
    }
    return true;
  }

  void flush() { rx_advance(rx_available()); }

private:
  UART_HandleTypeDef *huart_;
  core::Semaphore tx_sem_{1, 1};
  std::vector<uint8_t> rx_buf_;
  size_t rx_read_idx_ = 0;

  size_t rx_available() {
    size_t write_idx = rx_buf_.size() - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    return (rx_buf_.size() + write_idx - rx_read_idx_) % rx_buf_.size();
  }

  void rx_advance(size_t len) {
    rx_read_idx_ = (rx_read_idx_ + len) % rx_buf_.size();
  }
};

} // namespace peripheral
} // namespace stm32rcos
