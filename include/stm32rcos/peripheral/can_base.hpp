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

class CANBase {
public:
  virtual ~CANBase() {}
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool transmit(const CANMessage &msg, uint32_t timeout) = 0;
  virtual bool attach_rx_queue(const CANFilter &filter,
                               core::Queue<CANMessage> &queue) = 0;
  virtual bool detach_rx_queue(const core::Queue<CANMessage> &queue) = 0;
};

} // namespace peripheral
} // namespace stm32rcos