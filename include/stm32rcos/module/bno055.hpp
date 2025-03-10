#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <Eigen/Geometry>

#include "stm32rcos/core/queue.hpp"
#include "stm32rcos/core/semaphore.hpp"
#include "stm32rcos/peripheral/uart.hpp"

namespace stm32rcos {
namespace module {

class BNO055 {
public:
  BNO055(peripheral::UART &uart) : uart_{uart} {
    uart_.attach_tx_callback(
        [](void *args) {
          auto bno055 = reinterpret_cast<BNO055 *>(args);
          bno055->tx_sem_.release();
        },
        this);
    uart_.attach_rx_callback(
        [](void *args) {
          auto bno055 = reinterpret_cast<BNO055 *>(args);
          bno055->rx_queue_.push(bno055->rx_buf_, 0);
          bno055->uart_.receive_it(&bno055->rx_buf_, 1);
        },
        this);
    uart_.attach_abort_callback(
        [](void *args) {
          auto bno055 = reinterpret_cast<BNO055 *>(args);
          bno055->tx_sem_.release();
          bno055->uart_.receive_it(&bno055->rx_buf_, 1);
        },
        this);
    uart_.receive_it(&rx_buf_, 1);
  }

  ~BNO055() {
    uart_.abort();
    uart_.detach_tx_callback();
    uart_.detach_rx_callback();
    uart_.detach_abort_callback();
  }

  bool init(uint32_t timeout) {
    uint32_t start = osKernelGetTickCount();
    while (osKernelGetTickCount() - start < timeout) {
      uint8_t data = 0x00;
      if (!write_reg(0x3D, &data, 1)) {
        continue;
      }
      data = 0x08;
      if (!write_reg(0x3D, &data, 1)) {
        continue;
      }
      return true;
    }
    return false;
  }

  bool update() {
    std::array<int16_t, 4> data;
    if (!read_reg(0x20, reinterpret_cast<uint8_t *>(data.data()), 8)) {
      return false;
    }
    quat_orig_ = Eigen::Quaternionf{data[0] / 16384.0f, data[1] / 16384.0f,
                                    data[2] / 16384.0f, data[3] / 16384.0f};
    quat_ = quat_orig_ * quat_offset_;
    return true;
  }

  void reset() { quat_offset_ = quat_orig_; }

  const Eigen::Quaternionf &get_quaternion() { return quat_; }

private:
  peripheral::UART &uart_;
  core::Semaphore tx_sem_{1, 0};
  core::Queue<uint8_t> rx_queue_{64};
  uint8_t rx_buf_;

  Eigen::Quaternionf quat_orig_{Eigen::Quaternionf::Identity()};
  Eigen::Quaternionf quat_offset_{Eigen::Quaternionf::Identity()};
  Eigen::Quaternionf quat_{Eigen::Quaternionf::Identity()};

  bool write_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x00, addr, size};
    rx_queue_.clear();
    if (!uart_.transmit_it(buf.data(), 4)) {
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      return false;
    }
    if (!uart_.transmit_it(data, size)) {
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      return false;
    }
    for (size_t i = 0; i < 2; ++i) {
      if (!rx_queue_.pop(buf[i], 5)) {
        return false;
      }
    }
    return buf[0] == 0xEE && buf[1] == 0x01;
  }

  bool read_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x01, addr, size};
    rx_queue_.clear();
    if (!uart_.transmit_it(buf.data(), 4)) {
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      return false;
    }
    for (size_t i = 0; i < 2; ++i) {
      if (!rx_queue_.pop(buf[i], 5)) {
        return false;
      }
    }
    if (buf[0] != 0xBB || buf[1] != size) {
      return false;
    }
    for (size_t i = 0; i < size; ++i) {
      if (!rx_queue_.pop(data[i], 5)) {
        return false;
      }
    }
    return true;
  }
};

} // namespace module
} // namespace stm32rcos