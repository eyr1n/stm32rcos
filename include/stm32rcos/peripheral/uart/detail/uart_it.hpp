#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

#include "../uart_type.hpp"

namespace stm32rcos {
namespace peripheral {
namespace detail {

template <UartType TxType> class UartTx;
template <UartType RxType> class UartRx;

template <> class UartTx<UartType::IT> {
public:
  UartTx(UART_HandleTypeDef *huart) : huart_{huart} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
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

private:
  UART_HandleTypeDef *huart_;

  UartTx(const UartTx &) = delete;
  UartTx &operator=(const UartTx &) = delete;
};

template <> class UartRx<UartType::IT> {
public:
  UartRx(UART_HandleTypeDef *huart, size_t buf_size)
      : huart_{huart}, queue_{buf_size} {
    hal::set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UartRx *>(hal::get_uart_context(huart));
          uart->queue_.push(uart->buf_, 0);
          HAL_UART_Receive_IT(huart, &uart->buf_, 1);
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ERROR_CB_ID,
        [](UART_HandleTypeDef *huart) { HAL_UART_Abort_IT(huart); });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ABORT_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UartRx *>(hal::get_uart_context(huart));
          HAL_UART_Receive_IT(huart, &uart->buf_, 1);
        });
    HAL_UART_Receive_IT(huart, &buf_, 1);
  }

  ~UartRx() {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_RX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ERROR_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ABORT_COMPLETE_CB_ID);
    hal::set_uart_context(huart_, nullptr);
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    core::TimeoutHelper timeout_helper;
    while (queue_.size() < size) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      queue_.pop(data[i], 0);
    }
    return true;
  }

  void flush() { queue_.clear(); }

  size_t available() { return queue_.size(); }

private:
  UART_HandleTypeDef *huart_;
  core::Queue<uint8_t> queue_;
  uint8_t buf_;

  UartRx(const UartRx &) = delete;
  UartRx &operator=(const UartRx &) = delete;
};

} // namespace detail
} // namespace peripheral
} // namespace stm32rcos
