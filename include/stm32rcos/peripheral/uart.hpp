#pragma once

#include "main.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>

#include "stm32rcos/core/mutex.hpp"
#include "stm32rcos/core/queue.hpp"

extern "C" int _write(int file, char *ptr, int len);

namespace stm32rcos {
namespace peripheral {

class UART {
public:
  static UART &get_instance(UART_HandleTypeDef *huart);

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Transmit(huart_, data, size, timeout) == HAL_OK;
  }

  bool transmit_it(const uint8_t *data, size_t size) {
    return HAL_UART_Transmit_IT(huart_, data, size) == HAL_OK;
  }

  bool transmit_dma(const uint8_t *data, size_t size) {
    return HAL_UART_Transmit_DMA(huart_, data, size) == HAL_OK;
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    return HAL_UART_Receive(huart_, data, size, timeout) == HAL_OK;
  }

  bool receive_it(uint8_t *data, size_t size) {
    return HAL_UART_Receive_IT(huart_, data, size) == HAL_OK;
  }

  bool receive_dma(uint8_t *data, size_t size) {
    return HAL_UART_Receive_DMA(huart_, data, size) == HAL_OK;
  }

  bool abort() { return HAL_UART_Abort_IT(huart_) == HAL_OK; }

  bool attach_tx_callback(std::function<void(UART &)> &&callback) {
    if (tx_callback_) {
      return false;
    }
    tx_callback_ = std::move(callback);
    return true;
  }

  bool attach_rx_callback(std::function<void(UART &)> &&callback) {
    if (rx_callback_) {
      return false;
    }
    rx_callback_ = std::move(callback);
    return true;
  }

  bool attach_abort_callback(std::function<void(UART &)> &&callback) {
    if (abort_callback_) {
      return false;
    }
    abort_callback_ = std::move(callback);
    return true;
  }

  bool detach_tx_callback() {
    if (!tx_callback_) {
      return false;
    }
    tx_callback_ = std::function<void(UART &)>{};
    return true;
  }

  bool detach_rx_callback() {
    if (!rx_callback_) {
      return false;
    }
    rx_callback_ = std::function<void(UART &)>{};
    return true;
  }

  bool detach_abort_callback() {
    if (!abort_callback_) {
      return false;
    }
    abort_callback_ = std::function<void(UART &)>{};
    return true;
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
  std::function<void(UART &)> tx_callback_;
  std::function<void(UART &)> rx_callback_;
  std::function<void(UART &)> abort_callback_;

  UART(UART_HandleTypeDef *huart) : huart_{huart} {}

  static inline UART *&get_stdout_uart() {
    static UART *uart;
    return uart;
  }

  friend void ::HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
  friend void ::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
  friend void ::HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
  friend int ::_write(int file, char *ptr, int len);
};

} // namespace peripheral
} // namespace stm32rcos
