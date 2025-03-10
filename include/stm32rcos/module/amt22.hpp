#pragma once

#include "main.h"

#include <array>
#include <cstdint>
#include <type_traits>

#include "encoder_base.hpp"

namespace stm32rcos {
namespace module {

enum class AMT22Resolution : uint8_t {
  _12 = 12,
  _14 = 14,
};

enum class AMT22Mode {
  SINGLE_TURN,
  MULTI_TURN,
};

class AMT22 : public EncoderBase {
public:
  AMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
        AMT22Resolution resolution, AMT22Mode mode)
      : EncoderBase{1 << static_cast<std::underlying_type_t<AMT22Resolution>>(
                        resolution)},
        hspi_{hspi}, cs_port_{cs_port}, cs_pin_{cs_pin},
        resolution_{resolution}, mode_{mode} {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
  }

  bool update() {
    uint16_t cpr =
        1 << static_cast<std::underlying_type_t<AMT22Resolution>>(resolution_);
    std::array<uint8_t, 2> command{0x00, 0x00};
    uint16_t response;
    if (!send_command(command.data(), reinterpret_cast<uint8_t *>(&response),
                      command.size())) {
      return false;
    }

    int16_t count = response & (cpr - 1);
    switch (mode_) {
    case AMT22Mode::SINGLE_TURN: {
      set_count(count);
      break;
    }
    case AMT22Mode::MULTI_TURN: {
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
    std::array<uint8_t, 2> command{0x00, 0x70};
    std::array<uint8_t, 2> response{};
    if (!send_command(command.data(), response.data(), command.size())) {
      return false;
    }
    prev_count_ = 0;
    set_count(0);
    return true;
  }

private:
  SPI_HandleTypeDef *hspi_;
  GPIO_TypeDef *cs_port_;
  uint16_t cs_pin_;
  AMT22Resolution resolution_;
  AMT22Mode mode_;
  int16_t prev_count_ = 0;

  bool send_command(const uint8_t *command, uint8_t *response, size_t size) {
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
    for (size_t i = 0; i < size; ++i) {
      if (!HAL_SPI_TransmitReceive(hspi_, &command[i], &response[size - i - 1],
                                   1, 1)) {
        return false;
      }
    }
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    for (size_t i = 0; i < size; i += 2) {
      if (!checksum(response[i], response[i + 1])) {
        return false;
      }
    }
    return true;
  }

  bool checksum(uint8_t l, uint8_t h) {
    bool k1 = !(bit(h, 5) ^ bit(h, 3) ^ bit(h, 1) ^ bit(l, 7) ^ bit(l, 5) ^
                bit(l, 3) ^ bit(l, 1));
    bool k0 = !(bit(h, 4) ^ bit(h, 2) ^ bit(h, 0) ^ bit(l, 6) ^ bit(l, 4) ^
                bit(l, 2) ^ bit(l, 0));
    return (k1 == bit(h, 7)) && (k0 == bit(h, 6));
  }

  bool bit(uint8_t x, uint8_t i) { return ((x >> i) & 1) == 1; }
};

} // namespace module
} // namespace stm32rcos