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
          bno055->rx_sem_.release();
        },
        this);
  }

  ~BNO055() {
    uart_.abort();
    uart_.detach_tx_callback();
    uart_.detach_rx_callback();
  }

  bool start(uint32_t timeout) {
    uint32_t start = osKernelGetTickCount();
    while (osKernelGetTickCount() - start < timeout) {
      uint8_t data = 0x00;
      if (!write_reg<1>(0x3D, &data)) {
        continue;
      }
      data = 0x08;
      if (!write_reg<1>(0x3D, &data)) {
        continue;
      }
      return true;
    }
    return false;
  }

  bool update() {
    std::array<int16_t, 4> data;
    if (!read_reg<8>(0x20, reinterpret_cast<uint8_t *>(data.data()))) {
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
  core::Semaphore tx_sem_{1, 1};
  core::Semaphore rx_sem_{1, 1};

  Eigen::Quaternionf quat_orig_{Eigen::Quaternionf::Identity()};
  Eigen::Quaternionf quat_offset_{Eigen::Quaternionf::Identity()};
  Eigen::Quaternionf quat_{Eigen::Quaternionf::Identity()};

  template <size_t N> bool write_reg(uint8_t addr, const uint8_t *data) {
    std::array<uint8_t, 4 + N> tx_buf{0xAA, 0x00, addr, N};
    std::array<uint8_t, 2> rx_buf;
    std::copy(data, data + N, tx_buf.begin() + 4);
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (!uart_.receive_it(rx_buf.data(), rx_buf.size())) {
      uart_.abort();
      return false;
    }
    if (!uart_.transmit_it(tx_buf.data(), tx_buf.size())) {
      uart_.abort();
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      return false;
    }
    if (!rx_sem_.try_acquire(5)) {
      return false;
    }
    return rx_buf[0] == 0xEE && rx_buf[1] == 0x01;
  }

  template <size_t N> bool read_reg(uint8_t addr, uint8_t *data) {
    std::array<uint8_t, 4> tx_buf{0xAA, 0x01, addr, N};
    std::array<uint8_t, 2 + N> rx_buf;
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (!uart_.receive_it(rx_buf.data(), rx_buf.size())) {
      uart_.abort();
      return false;
    }
    if (!uart_.transmit_it(tx_buf.data(), tx_buf.size())) {
      uart_.abort();
      return false;
    }
    if (!tx_sem_.try_acquire(5)) {
      return false;
    }
    if (!rx_sem_.try_acquire(5)) {
      return false;
    }
    if (rx_buf[0] != 0xBB || rx_buf[1] != N) {
      return false;
    }
    std::copy(rx_buf.begin() + 2, rx_buf.end(), data);
    return true;
  }
};

} // namespace module
} // namespace stm32rcos