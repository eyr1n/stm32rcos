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

template <UART_HandleTypeDef *Handle, UartType TxType> class UartTx;
template <UART_HandleTypeDef *Handle, UartType RxType> class UartRx;

} // namespace detail

/**
 * デフォルトでTx, Rxともに割り込みを使用します。
 *
 * @code{.cpp}
 * #include <cstdio>
 * #include <stm32rcos/core.hpp>
 * #include <stm32rcos/hal.hpp>
 * #include <stm32rcos/peripheral.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace stm32rcos::core;
 *   using namespace stm32rcos::peripheral;
 *
 *   Uart<&huart2> uart2;
 *   enable_stdout(uart2);
 *
 *   while (true) {
 *     // 7バイト送信
 *     uint8_t data[] = {'h', 'e', 'l', 'l', 'o', '\r', '\n'};
 *     uart2.transmit(data, sizeof(data), osWaitForever);
 *
 *     // 1バイト受信
 *     char c;
 *     if (uart2.receive((uint8_t *)&c, 1, osWaitForever)) {
 *       printf("入力した文字: %c\r\n", c);
 *     }
 *
 *     osDelay(10);
 *   }
 * }
 * @endcode
 */
template <UART_HandleTypeDef *Handle, UartType TxType = UartType::IT,
          UartType RxType = UartType::IT>
class Uart : public UartBase {
public:
  Uart(size_t rx_buf_size = 64) : rx_{rx_buf_size} {}

  bool transmit(const uint8_t *data, size_t size, uint32_t timeout) {
    return tx_.transmit(data, size, timeout);
  }

  bool receive(uint8_t *data, size_t size, uint32_t timeout) {
    return rx_.receive(data, size, timeout);
  }

  void flush() { rx_.flush(); }

  size_t available() { return rx_.available(); }

private:
  detail::UartTx<Handle, TxType> tx_;
  detail::UartRx<Handle, RxType> rx_;
};

} // namespace peripheral
} // namespace stm32rcos
