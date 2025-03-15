#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "stm32rcos/core.hpp"
#include "stm32rcos/hal.hpp"

namespace stm32rcos {
namespace driver {

enum class AMT22Resolution : uint8_t {
  BIT_12 = 12,
  BIT_14 = 14,
};

class AMT22 {
public:
  AMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
        AMT22Resolution resolution)
      : hspi_{hspi}, cs_port_{cs_port}, cs_pin_{cs_pin},
        resolution_{resolution} {
    set_spi_context(hspi_, this);
    HAL_SPI_RegisterCallback(
        hspi_, HAL_SPI_TX_RX_COMPLETE_CB_ID, [](SPI_HandleTypeDef *hspi) {
          auto amt22 = reinterpret_cast<AMT22 *>(get_spi_context(hspi));
          amt22->tx_rx_sem_.release();
        });
  }

  ~AMT22() {
    HAL_SPI_Abort_IT(hspi_);
    HAL_SPI_UnRegisterCallback(hspi_, HAL_SPI_TX_RX_COMPLETE_CB_ID);
    set_spi_context(hspi_, nullptr);
  }

  std::optional<uint16_t> read_position() {
    std::array<uint8_t, 2> command{0x00, 0x00};
    std::array<uint8_t, 2> buf;
    if (!send_command(command.data(), buf.data(), command.size())) {
      return std::nullopt;
    }
    return (buf[0] << 8 | buf[1]) &
           ((1 << core::to_underlying(resolution_)) - 1);
  }

  std::optional<int16_t> read_turns() {
    std::array<uint8_t, 4> command{0x00, 0xA0, 0x00, 0x00};
    std::array<uint8_t, 4> buf;
    if (!send_command(command.data(), buf.data(), command.size())) {
      return std::nullopt;
    }
    int16_t turns = (buf[2] << 8 | buf[3]) & 0x3FFF;
    if (turns & 0x2000) {
      turns |= 0xC000;
    }
    return turns;
  }

  bool set_zero_point() {
    std::array<uint8_t, 2> command{0x00, 0x70};
    std::array<uint8_t, 2> buf;
    return send_command(command.data(), buf.data(), command.size());
  }

  bool reset() {
    std::array<uint8_t, 2> command{0x00, 0x60};
    std::array<uint8_t, 2> buf;
    return send_command(command.data(), buf.data(), command.size());
  }

private:
  SPI_HandleTypeDef *hspi_;
  GPIO_TypeDef *cs_port_;
  uint16_t cs_pin_;
  core::Semaphore tx_rx_sem_{1, 1};
  AMT22Resolution resolution_;

  bool send_command(const uint8_t *command, uint8_t *response, size_t size) {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    for (size_t i = 0; i < size; ++i) {
      tx_rx_sem_.try_acquire(0);
      if (HAL_SPI_TransmitReceive_IT(hspi_, command + i, response + i,
                                     sizeof(uint8_t))) {
        HAL_SPI_Abort_IT(hspi_);
        return false;
      }
      if (!tx_rx_sem_.try_acquire(1)) {
        HAL_SPI_Abort_IT(hspi_);
        return false;
      }
    }
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    for (size_t i = 0; i < size; i += 2) {
      if (!test_checksum(response[i], response[i + 1])) {
        return false;
      }
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