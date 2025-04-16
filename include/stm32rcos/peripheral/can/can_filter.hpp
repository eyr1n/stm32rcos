#pragma once

#include <cstdint>

namespace stm32rcos {
namespace peripheral {

struct CanFilter {
  uint32_t id;
  uint32_t mask;
  bool ide;
};

} // namespace peripheral
} // namespace stm32rcos