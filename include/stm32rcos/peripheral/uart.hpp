#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32rcos/hal.hpp"

#include "uart/stdout.hpp"
#include "uart/uart_base.hpp"
#include "uart/uart_type.hpp"

#ifdef HAL_UART_MODULE_ENABLED
#include "uart/internal/uart_dma.hpp"
#include "uart/internal/uart_it.hpp"
#include "uart/internal/uart_poll.hpp"
#endif

namespace stm32rcos {
namespace peripheral {

namespace internal {

template <UartType TxType> class UartTx;
template <UartType RxType> class UartRx;

} // namespace internal

template <UartType TxType = UartType::IT, UartType RxType = UartType::IT>
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
  internal::UartTx<TxType> tx_;
  internal::UartRx<RxType> rx_;
};

} // namespace peripheral
} // namespace stm32rcos
