#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <Eigen/Geometry>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

namespace stm32rcos {
namespace module {

class BNO055 {
public:
  BNO055(UART_HandleTypeDef *huart) : huart_{huart} {
    set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_TX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto bno055 = reinterpret_cast<BNO055 *>(get_uart_context(huart));
          bno055->tx_sem_.release();
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto bno055 = reinterpret_cast<BNO055 *>(get_uart_context(huart));
          bno055->rx_queue_.push(bno055->rx_buf_, 0);
          HAL_UART_Receive_IT(huart, &bno055->rx_buf_, 1);
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ERROR_CB_ID,
        [](UART_HandleTypeDef *huart) { HAL_UART_Abort_IT(huart); });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_ABORT_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto bno055 = reinterpret_cast<BNO055 *>(get_uart_context(huart));
          HAL_UART_Receive_IT(huart, &bno055->rx_buf_, 1);
        });
    HAL_UART_Receive_IT(huart, &rx_buf_, 1);
  }

  ~BNO055() {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_RX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ERROR_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_ABORT_COMPLETE_CB_ID);
    set_uart_context(huart_, nullptr);
  }

  bool start(uint32_t timeout) {
    TimeOut_t timeout_state;
    vTaskSetTimeOutState(&timeout_state);
    while (xTaskCheckForTimeOut(&timeout_state, &timeout) == pdFALSE) {
      uint8_t data = 0x00;
      if (!write_reg(0x3D, &data, 1)) {
        continue;
      }
      data = 0x08;
      if (!write_reg(0x3D, &data, 1)) {
        continue;
      }
      return true;
    }
    return false;
  }

  std::optional<Eigen::Quaternionf> get_quaternion() {
    std::array<int16_t, 4> data;
    if (!read_reg(0x20, reinterpret_cast<uint8_t *>(data.data()), 8)) {
      return std::nullopt;
    }
    return Eigen::Quaternionf{data[0] / 16384.0f, data[1] / 16384.0f,
                              data[2] / 16384.0f, data[3] / 16384.0f};
  }

private:
  UART_HandleTypeDef *huart_;
  core::Semaphore tx_sem_{1, 1};
  core::Queue<uint8_t> rx_queue_{64};
  uint8_t rx_buf_;

  bool write_reg(uint8_t addr, const uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x00, addr, size};
    rx_queue_.clear();
    tx_sem_.try_acquire(0);
    if (HAL_UART_Transmit_IT(huart_, buf.data(), buf.size()) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    if (HAL_UART_Transmit_IT(huart_, data, size) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    for (size_t i = 0; i < 2; ++i) {
      if (!rx_queue_.pop(buf[i], 5)) {
        return false;
      }
    }
    return buf[0] == 0xEE && buf[1] == 0x01;
  }

  bool read_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x01, addr, size};
    rx_queue_.clear();
    tx_sem_.try_acquire(0);
    if (HAL_UART_Transmit_IT(huart_, buf.data(), buf.size()) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    for (size_t i = 0; i < 2; ++i) {
      if (!rx_queue_.pop(buf[i], 5)) {
        return false;
      }
    }
    if (buf[0] != 0xBB || buf[1] != size) {
      return false;
    }
    for (size_t i = 0; i < size; ++i) {
      if (!rx_queue_.pop(data[i], 5)) {
        return false;
      }
    }
    return true;
  }
};

} // namespace module
} // namespace stm32rcos
