#pragma once

#include "stm32rcos/hal.hpp"

#include "can/can_base.hpp"
#include "can/can_filter.hpp"
#include "can/can_message.hpp"

#ifdef HAL_CAN_MODULE_ENABLED
#include "can/detail/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "can/detail/fdcan.hpp"
#endif

namespace stm32rcos {
namespace peripheral {

/**
 * @code{.cpp}
 * #include <cstdio>
 * #include <stm32rcos/core.hpp>
 * #include <stm32rcos/hal.hpp>
 * #include <stm32rcos/peripheral.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 * extern FDCAN_HandleTypeDef hfdcan1;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace stm32rcos::core;
 *   using namespace stm32rcos::peripheral;
 *
 *   Uart<&huart2> uart2;
 *   enable_stdout(uart2);
 *
 *   Can<&hfdcan1> can1;
 *
 *   // 受信フィルター、受信キューの設定
 *   CanFilter rx_filter = {
 *       .id = 0x0,
 *       .mask = 0x0,
 *       .ide = false,
 *   };
 *   Queue<CanMessage> rx_queue(10);
 *   can1.attach_rx_queue(rx_filter, rx_queue);
 *
 *   // CAN通信開始
 *   can1.start();
 *
 *   // 送信するメッセージの作成
 *   CanMessage tx_message = {
 *       .id = 0x3,
 *       .ide = false,
 *       .dlc = 1,
 *       .data = {0x0},
 *   };
 *
 *   for (int i = 0; i < 10; ++i) {
 *     // 送信
 *     can1.transmit(tx_message, osWaitForever);
 *     tx_message.data[0]++;
 *   }
 *
 *   while (true) {
 *     // 受信
 *     CanMessage rx_message;
 *     if (rx_queue.pop(rx_message, osWaitForever)) {
 *       printf("id: %d, data: %d\r\n", (int)rx_message.id,
 *              (int)rx_message.data[0]);
 *     }
 *     osDelay(10);
 *   }
 * }
 * @endcode
 */
template <auto *Handle, class HandleType = decltype(Handle)>
class Can : public CanBase {
public:
  bool start() override;
  bool stop() override;
  bool transmit(const CanMessage &msg, uint32_t timeout) override;
  bool attach_rx_queue(const CanFilter &filter,
                       core::Queue<CanMessage> &queue) override;
  bool detach_rx_queue(const core::Queue<CanMessage> &queue) override;
};

} // namespace peripheral
} // namespace stm32rcos