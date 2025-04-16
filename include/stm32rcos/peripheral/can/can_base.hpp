#pragma once

#include <cstdint>

#include "can_filter.hpp"
#include "can_message.hpp"

namespace stm32rcos {
namespace peripheral {

class CanBase {
public:
  virtual ~CanBase() {}
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool transmit(const CanMessage &msg, uint32_t timeout) = 0;
  virtual bool attach_rx_queue(const CanFilter &filter,
                               core::Queue<CanMessage> &queue) = 0;
  virtual bool detach_rx_queue(const core::Queue<CanMessage> &queue) = 0;
};

} // namespace peripheral
} // namespace stm32rcos