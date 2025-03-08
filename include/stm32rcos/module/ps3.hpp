#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "stm32rcos/core/queue.hpp"
#include "stm32rcos/peripheral/uart.hpp"

namespace stm32rcos {
namespace module {

/**
 * @code{.cpp}
 * #include <cstdio>
 * #include <tutrcos.hpp>
 * #include <tutrcos/module/ps3.hpp>
 *
 * extern UART_HandleTypeDef huart1;
 * extern UART_HandleTypeDef huart2;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *   using namespace tutrcos::module;
 *
 *   UART uart2(&huart2); // デバッグ出力用
 *   uart2.enable_stdout();
 *
 *   UART uart1(&huart1);
 *   PS3 ps3(uart1);
 *
 *   while (true) {
 *     ps3.update();
 *
 *     // DualShock 左スティックのx, y座標を出力
 *     printf("%f %f\r\n", ps3.get_axis(PS3::Axis::LEFT_X),
 *            ps3.get_axis(PS3::Axis::LEFT_Y));
 *
 *     if (ps3.get_key_down(PS3::Key::CIRCLE)) {
 *       printf("O ボタンが押されたよ\r\n");
 *     }
 *
 *     if (ps3.get_key_up(PS3::Key::CIRCLE)) {
 *       printf("O ボタンが離されたよ\r\n");
 *     }
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class PS3 {
public:
  enum class Axis {
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y,
  };

  enum class Key {
    UP,
    DOWN,
    RIGHT,
    LEFT,
    TRIANGLE,
    CROSS,
    CIRCLE,
    SQUARE = 8,
    L1,
    L2,
    R1,
    R2,
    START,
    SELECT,
  };

  PS3(peripheral::UART &uart) : uart_{uart} {
    uart_.attach_rx_callback([this](peripheral::UART &uart) {
      rx_queue_.push(rx_buf_, 0);
      uart.receive_it(&rx_buf_, 1);
    });
    uart_.attach_abort_callback(
        [this](peripheral::UART &uart) { uart.receive_it(&rx_buf_, 1); });
    uart_.receive_it(&rx_buf_, 1);
  }

  ~PS3() {
    uart_.abort();
    uart_.detach_rx_callback();
    uart_.detach_abort_callback();
  }

  void update() {
    keys_prev_ = keys_;

    while (receive_message()) {
      if (test_checksum(msg_)) {
        keys_ = (msg_[1] << 8) | msg_[2];
        if ((keys_ & 0x03) == 0x03) {
          keys_ &= ~0x03;
          keys_ |= 1 << 13;
        }
        if ((keys_ & 0x0C) == 0x0C) {
          keys_ &= ~0x0C;
          keys_ |= 1 << 14;
        }
        for (size_t i = 0; i < 4; ++i) {
          axes_[i] = (static_cast<float>(msg_[i + 3]) - 64) / 64;
        }
      }

      msg_.fill(0);
    }
  }

  float get_axis(Axis axis) {
    return axes_[static_cast<std::underlying_type_t<Axis>>(axis)];
  }

  bool get_key(Key key) {
    return (keys_ & (1 << static_cast<std::underlying_type_t<Key>>(key))) != 0;
  }

  bool get_key_down(Key key) {
    return ((keys_ ^ keys_prev_) & keys_ &
            (1 << static_cast<std::underlying_type_t<Key>>(key))) != 0;
  }

  bool get_key_up(Key key) {
    return ((keys_ ^ keys_prev_) & keys_prev_ &
            (1 << static_cast<std::underlying_type_t<Key>>(key))) != 0;
  }

private:
  peripheral::UART &uart_;
  uint8_t rx_buf_;
  core::Queue<uint8_t> rx_queue_{64};
  std::array<uint8_t, 8> msg_{};
  std::array<float, 4> axes_{};
  uint16_t keys_ = 0;
  uint16_t keys_prev_ = 0;

  bool receive_message() {
    // ヘッダ探す
    for (size_t i = 0; i < 8; ++i) {
      if (msg_[0] == 0x80) {
        break;
      }
      if (!rx_queue_.pop(msg_[0], 0)) {
        return false;
      }
    }
    if (msg_[0] != 0x80) {
      return false;
    }

    // メッセージの残りの部分を受信
    for (size_t i = 0; i < 7; ++i) {
      if (!rx_queue_.pop(msg_[i + 1], 0)) {
        return false;
      }
    }
    return true;
  }

  static inline bool test_checksum(const std::array<uint8_t, 8> &msg) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < 7; ++i) {
      checksum += msg[i];
    }
    return (checksum & 0x7F) == msg[7];
  }
};

} // namespace module
} // namespace stm32rcos
