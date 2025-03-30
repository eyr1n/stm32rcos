#pragma once

#include <cstdint>
#include <optional>

#include "amt21_manager.hpp"

namespace stm32rcos {
namespace driver {

enum class AMT21Resolution : uint8_t {
  BIT_12 = 12,
  BIT_14 = 14,
};

class AMT21 {
public:
  AMT21(AMT21Manager &manager, AMT21Resolution resolution, uint8_t address)
      : manager_{manager}, resolution_{resolution}, address_{address} {}

  std::optional<uint16_t> read_position() {
    auto res = manager_.send_command(address_, 0x00);
    if (!res) {
      return std::nullopt;
    }
    return ((*res)[1] << 8 | (*res)[0]) &
           ((1 << core::to_underlying(resolution_)) - 1);
  }

  std::optional<int16_t> read_turns() {
    auto res = manager_.send_command(address_, 0x01);
    if (!res) {
      return std::nullopt;
    }
    int16_t turns = ((*res)[1] << 8 | (*res)[0]) & 0x3FFF;
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