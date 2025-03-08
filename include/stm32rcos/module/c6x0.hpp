#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "stm32rcos/peripheral/can.hpp"

#include "encoder_base.hpp"

namespace stm32rcos {
namespace module {

enum class C6x0Type {
  C610,
  C620,
};

enum class C6x0ID {
  _1,
  _2,
  _3,
  _4,
  _5,
  _6,
  _7,
  _8,
};

template <class CAN_> class C6x0;

template <class CAN_> class C6x0Manager {
public:
  C6x0Manager(CAN_ &can) : can_{can} {
    can_.attach_rx_queue({0x200, 0x200, false}, rx_queue_);
  }

  ~C6x0Manager() { can_.detach_rx_queue(rx_queue_); }

  bool update() {
    peripheral::CANMessage msg;
    while (rx_queue_.pop(msg, 0)) {
      if (msg.id >= 0x201 && msg.id < 0x201 + 8) {
        C6x0<CAN_> *motor = motors_[msg.id - 0x201];
        if (motor) {
          int16_t count = static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
          int16_t delta = count - motor->prev_count_;
          if (delta > 4096) {
            delta -= 8192;
          } else if (delta < -4096) {
            delta += 8192;
          }
          motor->set_count(motor->get_count() + delta);
          motor->prev_count_ = count;

          motor->rpm_ = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
          motor->current_ =
              static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
        }
      }
    }

    msg.data.fill(0);
    msg.ide = false;
    msg.id = 0x200;
    msg.dlc = 8;
    for (size_t i = 0; i < 4; ++i) {
      if (motors_[i]) {
        msg.data[i * 2] = motors_[i]->target_current_ >> 8;
        msg.data[i * 2 + 1] = motors_[i]->target_current_;
      }
    }
    if (!can_.transmit(msg)) {
      return false;
    }

    msg.data.fill(0);
    msg.id = 0x1FF;
    for (size_t i = 0; i < 4; ++i) {
      if (motors_[i + 4]) {
        msg.data[i * 2] = motors_[i + 4]->target_current_ >> 8;
        msg.data[i * 2 + 1] = motors_[i + 4]->target_current_;
      }
    }
    return can_.transmit(msg);
  }

private:
  CAN_ &can_;
  core::Queue<peripheral::CANMessage> rx_queue_{64};
  std::array<C6x0<CAN_> *, 8> motors_{};

  friend C6x0<CAN_>;
};

template <class CAN_> class C6x0 : public EncoderBase {
public:
  C6x0(C6x0Manager<CAN_> &manager, C6x0Type type, C6x0ID id)
      : EncoderBase{8192}, manager_{manager}, type_{type}, id_{id} {
    manager_.motors_[static_cast<std::underlying_type_t<C6x0ID>>(id_)] = this;
  }

  ~C6x0() {
    manager_.motors_[static_cast<std::underlying_type_t<C6x0ID>>(id_)] =
        nullptr;
  }

  float get_rps() override {
    return get_rpm() / 60;
  }

  float get_rpm() override {
    return rpm_;
  }

  int16_t get_current() {
    switch (type_) {
    case C6x0Type::C610:
      return current_;
    case C6x0Type::C620:
      return current_ * 20000 / 16384;
    }
  }

  void set_current(int16_t current) {
    switch (type_) {
    case C6x0Type::C610:
      target_current_ = current;
      break;
    case C6x0Type::C620:
      target_current_ = static_cast<int64_t>(current) * 16384 / 20000;
      break;
    }
  }

private:
  C6x0Manager<CAN_> &manager_;
  C6x0Type type_;
  C6x0ID id_;

  int16_t prev_count_ = 0;
  int16_t rpm_ = 0;
  int16_t current_ = 0;
  int16_t target_current_ = 0;
};

} // namespace module
} // namespace stm32rcos