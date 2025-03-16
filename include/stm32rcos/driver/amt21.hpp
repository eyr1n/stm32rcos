#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

namespace stm32rcos {
namespace driver {

enum class AMT21Resolution : uint8_t {
  BIT_12 = 12,
  BIT_14 = 14,
};

class AMT21;

class AMT21Manager {
public:
  AMT21Manager(UART_HandleTypeDef *huart) : huart_{huart} {
    set_uart_context(huart_, this);
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_TX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto manager =
              reinterpret_cast<AMT21Manager *>(get_uart_context(huart));
          manager->tx_sem_.release();
        });
    HAL_UART_RegisterCallback(
        huart_, HAL_UART_RX_COMPLETE_CB_ID, [](UART_HandleTypeDef *huart) {
          auto manager =
              reinterpret_cast<AMT21Manager *>(get_uart_context(huart));
          manager->rx_sem_.release();
        });
  }

  ~AMT21Manager() {
    HAL_UART_Abort_IT(huart_);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_TX_COMPLETE_CB_ID);
    HAL_UART_UnRegisterCallback(huart_, HAL_UART_RX_COMPLETE_CB_ID);
    set_uart_context(huart_, nullptr);
  }

private:
  UART_HandleTypeDef *huart_;
  core::Semaphore tx_sem_{1, 1};
  core::Semaphore rx_sem_{1, 1};

  friend AMT21;

  bool send_command(uint8_t address, uint8_t command, uint8_t *data) {
    uint8_t buf = address | command;
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (HAL_UART_Receive_DMA(huart_, data, 2) != HAL_OK) {
      HAL_UART_AbortReceive_IT(huart_);
      return false;
    }
    if (HAL_UART_Transmit_IT(huart_, &buf, sizeof(buf)) != HAL_OK) {
      HAL_UART_Abort_IT(huart_);
      return false;
    }
    if (!tx_sem_.try_acquire(1)) {
      HAL_UART_Abort_IT(huart_);
      return false;
    }
    if (!rx_sem_.try_acquire(1)) {
      HAL_UART_AbortReceive_IT(huart_);
      return false;
    }
    if (!test_checksum(data[0], data[1])) {
      return false;
    }
    return data;
  }

  bool send_extended_command(uint8_t address, uint8_t command) {
    std::array<uint8_t, 2> buf{static_cast<uint8_t>(address | 0x02), command};
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (HAL_UART_Transmit_IT(huart_, buf.data(), buf.size()) != HAL_OK) {
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

class AMT21 {
public:
  AMT21(AMT21Manager &manager, AMT21Resolution resolution, uint8_t address)
      : manager_{manager}, resolution_{resolution}, address_{address} {}

  std::optional<uint16_t> read_position() {
    std::array<uint8_t, 2> data;
    if (!manager_.send_command(address_, 0x00, data.data())) {
      return std::nullopt;
    }
    return (data[1] << 8 | data[0]) &
           ((1 << core::to_underlying(resolution_)) - 1);
  }

  std::optional<int16_t> read_turns() {
    std::array<uint8_t, 2> data;
    if (!manager_.send_command(address_, 0x01, data.data())) {
      return std::nullopt;
    }
    int16_t turns = (data[1] << 8 | data[0]) & 0x3FFF;
    if (turns & 0x2000) {
      turns |= 0xC000;
    }
    return turns;
  }

  bool set_zero_point() {
    return manager_.send_extended_command(address_, 0x5E);
  }

  bool reset() { return manager_.send_extended_command(address_, 0x75); }

private:
  AMT21Manager &manager_;
  AMT21Resolution resolution_;
  uint8_t address_;
};

} // namespace driver
} // namespace stm32rcos