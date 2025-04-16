#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32rcos/hal.hpp"

#include "uart/stdout.hpp"
#include "uart/uart_base.hpp"
#include "uart/uart_type.hpp"

#ifdef HAL_UART_MODULE_ENABLED
#include "uart/detail/uart_dma.hpp"
#include "uart/detail/uart_it.hpp"
#include "uart/detail/uart_poll.hpp"
#endif

namespace stm32rcos {
namespace peripheral {

namespace detail {

template <UartType TxType> class UartTx;
template <UartType RxType> class UartRx;

} // namespace detail

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
  detail::UartTx<TxType> tx_;
  detail::UartRx<RxType> rx_;
};

} // namespace peripheral
} // namespace stm32rcos
