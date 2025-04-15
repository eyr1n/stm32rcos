#pragma once

#include <cstddef>
#include <cstdint>

namespace stm32rcos {
namespace peripheral {

enum class UartType {
  POLL,
  IT,
  DMA,
};

class UartBase {
public:
  virtual ~UartBase() {}
  virtual bool transmit(const uint8_t *data, size_t size, uint32_t timeout) = 0;
  virtual bool receive(uint8_t *data, size_t size, uint32_t timeout) = 0;
  virtual void flush() = 0;
  virtual size_t available() = 0;
};

bool enable_stdout(UartBase &uart);

bool disable_stdout();

} // namespace peripheral
} // namespace stm32rcos
