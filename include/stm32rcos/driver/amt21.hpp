#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

#include "encoder_base.hpp"

namespace stm32rcos {
namespace driver {

enum class AMT21Resolution : uint8_t {
  _12 = 12,
  _14 = 14,
};

enum class AMT21Mode {
  SINGLE_TURN,
  MULTI_TURN,
};

class AMT21 : public EncoderBase {
public:
  AMT21(UART_HandleTypeDef *huart, AMT21Resolution resolution, AMT21Mode mode,
        uint8_t address)
      : EncoderBase{1 << core::to_underlying(resolution)}, huart_{huart},
        resolution_{resolution}, mode_{mode}, address_{address} {
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

  bool update() {
    uint16_t cpr = 1 << core::to_underlying(resolution_);
    uint16_t response;
    if (!send_command(0x00, reinterpret_cast<uint8_t *>(&response))) {
      return false;
    }

    int16_t count = response & (cpr - 1);
    switch (mode_) {
    case AMT21Mode::SINGLE_TURN: {
      set_count(count);
      break;
    }
    case AMT21Mode::MULTI_TURN: {
      int16_t delta = count - prev_count_;
      if (delta > (cpr / 2)) {
        delta -= cpr;
      } else if (delta < -(cpr / 2)) {
        delta += cpr;
      }
      set_count(get_count() + delta);
      prev_count_ = count;
      break;
    }
    }
    return true;
  }

  bool set_zero_point() {
    if (!send_extended_command(0x5E)) {
      return false;
    }
    prev_count_ = 0;
    set_count(0);
    return true;
  }

private:
  UART_HandleTypeDef *huart_;
  core::Semaphore tx_sem_{1, 1};
  core::Semaphore rx_sem_{1, 1};

  AMT21Resolution resolution_;
  AMT21Mode mode_;
  uint8_t address_;
  int16_t prev_count_ = 0;

  bool send_command(uint8_t command, uint8_t *response) {
    uint8_t data = address_ | command;
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (HAL_UART_Receive_DMA(huart_, reinterpret_cast<uint8_t *>(response),
                             2) != HAL_OK) {
      HAL_UART_AbortReceive_IT(huart_);
      return false;
    }
    if (HAL_UART_Transmit_IT(huart_, &data, 1) != HAL_OK) {
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
    return test_checksum(response[0], response[1]);
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