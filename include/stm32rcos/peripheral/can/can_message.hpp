#pragma once

#include <array>
#include <cstdint>

namespace stm32rcos {
namespace peripheral {

struct CanMessage {
  uint32_t id;
  bool ide;
  uint8_t dlc;
  std::array<uint8_t, 8> data;
};

} // namespace peripheral
} // namespace stm32rcos