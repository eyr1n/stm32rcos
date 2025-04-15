#pragma once

#include <cstddef>
#include <cstdint>

namespace stm32rcos {
namespace peripheral {

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

enum class UartType {
  Polling,
  Interrupt,
  Dma,
};

namespace internal {

template <UartType Tx> class UartTx;
template <UartType Rx> class UartRx;

} // namespace internal

template <UartType Tx = UartType::Interrupt, UartType Rx = UartType::Interrupt>
class Uart : public UartBase {
public:
  Uart(UART_HandleTypeDef *huart, size_t rx_buf_size = 64)
      : tx_{huart}, rx_{huart, rx_buf_size} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return tx_.transmit(data, size, timeout);
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    return rx_.receive(data, size, timeout);
  }

  void flush() { rx_.flush(); }

  size_t available() { return rx_.available(); }

private:
  internal::UartTx<Tx> tx_;
  internal::UartRx<Rx> rx_;
};

} // namespace peripheral
} // namespace stm32rcos
