#pragma once

#include <array>
#include <cstdint>

namespace stm32rcos {
namespace peripheral {

struct CanFilter {
  uint32_t id;
  uint32_t mask;
  bool ide;
};

struct CanMessage {
  uint32_t id;
  bool ide;
  uint8_t dlc;
  std::array<uint8_t, 8> data;
};

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

template <class Handle> class Can;

template <class Handle> Can(Handle *) -> Can<Handle>;

} // namespace peripheral
} // namespace stm32rcos