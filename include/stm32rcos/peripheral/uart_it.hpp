#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

#include "uart_base.hpp"

namespace stm32rcos {
namespace peripheral {

class UART_IT : public UARTBase {
public:
  UART_IT(UART_HandleTypeDef *huart, size_t rx_queue_size = 64)
      : huart_{huart}, rx_queue_{rx_queue_size} {
    hal::set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UART_IT *>(hal::get_uart_context(huart));
          uart->rx_queue_.push(uart->rx_buf_, 0);
          HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1);
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ERROR_CB_ID,
        [](UART_HandleTypeDef *huart) { HAL_UART_Abort_IT(huart); });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ABORT_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto uart = reinterpret_cast<UART_IT *>(hal::get_uart_context(huart));
          HAL_UART_Receive_IT(huart, &uart->rx_buf_, 1);
        });
    HAL_UART_Receive_IT(huart, &rx_buf_, 1);
  }

  ~UART_IT() override {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_RX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ERROR_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ABORT_COMPLETE_CB_ID);
    hal::set_uart_context(huart_, nullptr);
  }

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
    while (rx_queue_.size() < size) {
      if (timeout_helper.is_timeout(timeout)) {
        return false;
      }
      osDelay(1);
    }
    for (size_t i = 0; i < size; ++i) {
      rx_queue_.pop(data[i], 0);
    }
    return true;
  }

  void flush() override { rx_queue_.clear(); }

  size_t available() override { return rx_queue_.size(); }

private:
  UART_HandleTypeDef *huart_;
  core::Queue<uint8_t> rx_queue_;
  uint8_t rx_buf_;

  UART_IT(const UART_IT &) = delete;
  UART_IT &operator=(const UART_IT &) = delete;
};

} // namespace peripheral
} // namespace stm32rcos
