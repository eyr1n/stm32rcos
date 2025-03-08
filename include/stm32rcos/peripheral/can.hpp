#pragma once

#include <array>
#include <cstdint>

namespace stm32rcos {
namespace peripheral {

struct CANFilter {
  uint32_t id;
  uint32_t mask;
  bool ide;
};

struct CANMessage {
  uint32_t id;
  bool ide;
  uint8_t dlc;
  std::array<uint8_t, 8> data;
};

} // namespace peripheral
} // namespace stm32rcos