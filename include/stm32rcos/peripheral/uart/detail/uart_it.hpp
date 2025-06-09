#pragma once

#include <cstddef>
#include <cstdint>

#include <stm32cubemx_helper/context.hpp>
#include <stm32cubemx_helper/device.hpp>

#include "stm32rcos/core.hpp"

#include "../uart_type.hpp"

namespace stm32rcos {
namespace peripheral {
namespace detail {

template <auto *Handle, UartType TxType> class UartTx;
template <auto *Handle, UartType RxType> class UartRx;

template <auto *Handle> class UartTx<Handle, UartType::IT> {
public:
  UartTx() = default;

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    if (HAL_UART_Transmit_IT(Handle, data, size) != HAL_OK) {
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

template <auto *Handle> class UartRx<Handle, UartType::IT> {
public:
  UartRx(size_t buf_size) : queue_{buf_size} {
    stm32cubemx_helper::set_context<Handle, UartRx>(this);
    HAL_UART_RegisterCallback(
        Handle, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = stm32cubemx_helper::get_context<Handle, UartRx>();
          uart->queue_.push(uart->buf_, 0);
          HAL_UART_Receive_IT(huart, &uart->buf_, 1);
        });
    HAL_UART_RegisterCallback(
        Handle, HAL_UART_ERROR_CB_ID,
        [](UART_HandleTypeDef *huart) { HAL_UART_Abort_IT(huart); });
    HAL_UART_RegisterCallback(
        Handle, HAL_UART_ABORT_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = stm32cubemx_helper::get_context<Handle, UartRx>();
          HAL_UART_Receive_IT(huart, &uart->buf_, 1);
        });
    HAL_UART_Receive_IT(Handle, &buf_, 1);
  }

  ~UartRx() {
    HAL_UART_Abort_IT(Handle);
    HAL_UART_UnRegisterCallback(Handle, HAL_UART_RX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(Handle, HAL_UART_ERROR_CB_ID);
    HAL_UART_UnRegisterCallback(Handle, HAL_UART_ABORT_COMPLETE_CB_ID);
    stm32cubemx_helper::set_context<Handle, UartRx>(nullptr);
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
  core::Queue<uint8_t> queue_;
  uint8_t buf_;

  UartRx(const UartRx &) = delete;
  UartRx &operator=(const UartRx &) = delete;
};

} // namespace detail
} // namespace peripheral
} // namespace stm32rcos
