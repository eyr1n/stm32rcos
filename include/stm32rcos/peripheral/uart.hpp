#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "stm32_hal.h"

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

  bool attach_tx_callback(void (*callback)(void *), void *args) {
    if (tx_callback_ || tx_args_) {
      return false;
    }
    tx_callback_ = callback;
    tx_args_ = args;
    return true;
  }

  bool attach_rx_callback(void (*callback)(void *), void *args) {
    if (rx_callback_ || rx_args_) {
      return false;
    }
    rx_callback_ = callback;
    rx_args_ = args;
    return true;
  }

  bool attach_abort_callback(void (*callback)(void *), void *args) {
    if (abort_callback_ || abort_args_) {
      return false;
    }
    abort_callback_ = callback;
    abort_args_ = args;
    return true;
  }

  bool detach_tx_callback() {
    if (!tx_callback_ || !tx_args_) {
      return false;
    }
    tx_callback_ = nullptr;
    tx_args_ = nullptr;
    return true;
  }

  bool detach_rx_callback() {
    if (!rx_callback_ || !rx_args_) {
      return false;
    }
    rx_callback_ = nullptr;
    rx_args_ = nullptr;
    return true;
  }

  bool detach_abort_callback() {
    if (!abort_callback_ || !abort_args_) {
      return false;
    }
    abort_callback_ = nullptr;
    abort_args_ = nullptr;
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
  void (*tx_callback_)(void *);
  void *tx_args_;
  void (*rx_callback_)(void *);
  void *rx_args_;
  void (*abort_callback_)(void *);
  void *abort_args_;

  UART(UART_HandleTypeDef *huart) : huart_{huart} {}
  UART(const UART &) = delete;
  UART &operator=(const UART &) = delete;
  UART(UART &&) = delete;
  UART &operator=(UART &&) = delete;

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
