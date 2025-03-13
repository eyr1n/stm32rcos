#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "stm32rcos/core.hpp"

#include "stm32rcos/peripheral/uart.hpp"

namespace stm32rcos {
namespace module {

enum class PS3Axis {
  LEFT_X,
  LEFT_Y,
  RIGHT_X,
  RIGHT_Y,
};

enum class PS3Key {
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

class PS3 {
public:
  PS3(peripheral::UART &uart) : uart_{uart} {
    uart_.attach_rx_callback(
        [](void *args) {
          auto ps3 = reinterpret_cast<PS3 *>(args);
          ps3->rx_queue_.push(ps3->rx_buf_, 0);
          ps3->uart_.receive_it(&ps3->rx_buf_, 1);
        },
        this);
    uart_.attach_abort_callback(
        [](void *args) {
          auto ps3 = reinterpret_cast<PS3 *>(args);
          ps3->uart_.receive_it(&ps3->rx_buf_, 1);
        },
        this);
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

  float get_axis(PS3Axis axis) { return axes_[utility::to_underlying(axis)]; }

  bool get_key(PS3Key key) {
    return (keys_ & (1 << utility::to_underlying(key))) != 0;
  }

  bool get_key_down(PS3Key key) {
    return ((keys_ ^ keys_prev_) & keys_ &
            (1 << utility::to_underlying(key))) != 0;
  }

  bool get_key_up(PS3Key key) {
    return ((keys_ ^ keys_prev_) & keys_prev_ &
            (1 << utility::to_underlying(key))) != 0;
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
