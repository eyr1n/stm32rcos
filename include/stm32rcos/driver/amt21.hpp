#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

namespace stm32rcos {
namespace driver {

enum class AMT21Resolution : uint8_t {
  _12 = 12,
  _14 = 14,
};

class AMT21 {
public:
  AMT21(UART_HandleTypeDef *huart, AMT21Resolution resolution, uint8_t address)
      : huart_{huart}, resolution_{resolution}, address_{address} {
    set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_TX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto amt21 = reinterpret_cast<AMT21 *>(get_uart_context(huart));
          amt21->tx_sem_.release();
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto amt21 = reinterpret_cast<AMT21 *>(get_uart_context(huart));
          amt21->rx_sem_.release();
        });
  }

  ~AMT21() {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_RX_COMPLETE_CB_ID);
    set_uart_context(huart_, nullptr);
  }

  std::optional<uint16_t> read_position() {
    if (auto position = send_command(0x00)) {
      return *position & ((1 << core::to_underlying(resolution_)) - 1);
    }
    return std::nullopt;
  }

  std::optional<int16_t> read_turns() { return send_command(0x01); }

  bool set_zero_point() { return send_extended_command(0x5E); }

  bool reset() { return send_extended_command(0x75); }

private:
  UART_HandleTypeDef *huart_;
  core::Semaphore tx_sem_{1, 1};
  core::Semaphore rx_sem_{1, 1};

  AMT21Resolution resolution_;
  uint8_t address_;

  std::optional<int16_t> send_command(uint8_t command) {
    uint8_t tx_data = address_ | command;
    std::array<uint8_t, 2> rx_data;
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (HAL_UART_Receive_DMA(huart_, rx_data.data(), rx_data.size()) !=
        HAL_OK) {
      HAL_UART_AbortReceive_IT(huart_);
      return std::nullopt;
    }
    if (HAL_UART_Transmit_IT(huart_, &tx_data, sizeof(tx_data)) != HAL_OK) {
      HAL_UART_Abort_IT(huart_);
      return std::nullopt;
    }
    if (!tx_sem_.try_acquire(1)) {
      HAL_UART_Abort_IT(huart_);
      return std::nullopt;
    }
    if (!rx_sem_.try_acquire(1)) {
      HAL_UART_AbortReceive_IT(huart_);
      return std::nullopt;
    }
    if (!test_checksum(rx_data[0], rx_data[1])) {
      return std::nullopt;
    }
    return (rx_data[1] << 8) | rx_data[0];
  }

  bool send_extended_command(uint8_t command) {
    std::array<uint8_t, 2> data{static_cast<uint8_t>(address_ | 0x02), command};
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (HAL_UART_Transmit_IT(huart_, data.data(), data.size()) != HAL_OK) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    if (!tx_sem_.try_acquire(1)) {
      HAL_UART_AbortTransmit_IT(huart_);
      return false;
    }
    return true;
  }

  bool test_checksum(uint8_t l, uint8_t h) {
    bool k1 = !(bit(h, 5) ^ bit(h, 3) ^ bit(h, 1) ^ bit(l, 7) ^ bit(l, 5) ^
                bit(l, 3) ^ bit(l, 1));
    bool k0 = !(bit(h, 4) ^ bit(h, 2) ^ bit(h, 0) ^ bit(l, 6) ^ bit(l, 4) ^
                bit(l, 2) ^ bit(l, 0));
    return (k1 == bit(h, 7)) && (k0 == bit(h, 6));
  }

  bool bit(uint8_t x, uint8_t i) { return ((x >> i) & 1) == 1; }
};

} // namespace driver
} // namespace stm32rcos