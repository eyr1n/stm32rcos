#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

extern "C" int _write(int file, char *ptr, int len);

namespace stm32rcos {
namespace peripheral {

class UART {
public:
  UART(UART_HandleTypeDef *huart, size_t rx_buf_size = 64)
      : huart_{huart}, rx_buf_{rx_buf_size} {
    set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_TX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UART *>(get_uart_context(huart));
          uart->tx_sem_.release();
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UART *>(get_uart_context(huart));
          uart->rx_buf_.push(uart->rx_tmp_, 0);
          HAL_UART_Receive_IT(huart, &uart->rx_tmp_, 1);
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ERROR_CB_ID,
        [](UART_HandleTypeDef *huart) { HAL_UART_AbortReceive_IT(huart); });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID,
        [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UART *>(get_uart_context(huart));
          HAL_UART_Receive_IT(huart, &uart->rx_tmp_, 1);
        });
    HAL_UART_Receive_IT(huart, &rx_tmp_, 1);
  }

  ~UART() {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_RX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ERROR_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID);
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
    while (rx_buf_.size() < size) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      if (!rx_buf_.pop(data[i], 0)) {
        return false;
      }
    }
    return true;
  }

  void flush() { rx_buf_.clear(); }

  bool enable_stdout() {
    if (*uart_stdout()) {
      return false;
    }
    *uart_stdout() = this;
    return true;
  }

  bool disable_stdout() {
    if (!*uart_stdout()) {
      return false;
    }
    *uart_stdout() = nullptr;
    return true;
  }

private:
  UART_HandleTypeDef *huart_;
  core::Semaphore tx_sem_{1, 1};
  core::Queue<uint8_t> rx_buf_;
  uint8_t rx_tmp_;

  static inline UART **uart_stdout() {
    static UART *uart;
    return &uart;
  }

  friend int ::_write(int file, char *ptr, int len);
};

} // namespace peripheral
} // namespace stm32rcos
