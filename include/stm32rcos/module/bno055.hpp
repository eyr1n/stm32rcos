#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

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
      data = 0x04;
      if (!write_reg(0x3B, &data, 1)) {
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
    if (!read_reg(0x1A, reinterpret_cast<uint8_t *>(data.data()), 6)) {
      return false;
    }
    euler_x_orig_ = data[0] / 900.0f;
    euler_y_orig_ = data[1] / 900.0f;
    euler_z_orig_ = data[2] / 900.0f;

    euler_x_ = normalize_angle(euler_x_orig_ - euler_x_offset_);
    euler_y_ = normalize_angle(euler_y_orig_ - euler_y_offset_);
    euler_z_ = normalize_angle(euler_z_orig_ - euler_z_offset_);

    if (!read_reg(0x20, reinterpret_cast<uint8_t *>(data.data()), 8)) {
      return false;
    }
    quat_w_orig_ = data[0] / 16384.0f;
    quat_x_orig_ = data[1] / 16384.0f;
    quat_y_orig_ = data[2] / 16384.0f;
    quat_z_orig_ = data[3] / 16384.0f;

    quat_w_ = quat_w_orig_ * quat_w_offset_ - quat_x_orig_ * quat_x_offset_ -
              quat_y_orig_ * quat_y_offset_ - quat_z_orig_ * quat_z_offset_;
    quat_x_ = quat_w_orig_ * quat_x_offset_ + quat_x_orig_ * quat_w_offset_ +
              quat_y_orig_ * quat_z_offset_ - quat_z_orig_ * quat_y_offset_;
    quat_y_ = quat_w_orig_ * quat_y_offset_ - quat_x_orig_ * quat_z_offset_ +
              quat_y_orig_ * quat_w_offset_ + quat_z_orig_ * quat_x_offset_;
    quat_z_ = quat_w_orig_ * quat_z_offset_ + quat_x_orig_ * quat_y_offset_ -
              quat_y_orig_ * quat_x_offset_ + quat_z_orig_ * quat_w_offset_;
    return true;
  }

  void reset_euler() {
    euler_x_offset_ = euler_x_orig_;
    euler_y_offset_ = euler_y_orig_;
    euler_z_offset_ = euler_z_orig_;
  }

  void reset_quat() {
    quat_w_offset_ = -quat_w_orig_;
    quat_x_offset_ = quat_x_orig_;
    quat_y_offset_ = quat_y_orig_;
    quat_z_offset_ = quat_z_orig_;
  }

  float get_euler_x() { return euler_x_; }

  float get_euler_y() { return euler_y_; }

  float get_euler_z() { return euler_z_; }

  float get_quat_w() { return quat_w_; }

  float get_quat_x() { return quat_x_; }

  float get_quat_y() { return quat_y_; }

  float get_quat_z() { return quat_z_; }

private:
  peripheral::UART &uart_;
  core::Semaphore tx_sem_{1, 1};
  core::Queue<uint8_t> rx_queue_{64};
  uint8_t rx_buf_;

  float euler_x_orig_ = 0;
  float euler_y_orig_ = 0;
  float euler_z_orig_ = 0;

  float euler_x_offset_ = 0;
  float euler_y_offset_ = 0;
  float euler_z_offset_ = 0;

  float euler_x_ = 0;
  float euler_y_ = 0;
  float euler_z_ = 0;

  float quat_w_orig_ = 1;
  float quat_x_orig_ = 0;
  float quat_y_orig_ = 0;
  float quat_z_orig_ = 0;

  float quat_w_offset_ = -1;
  float quat_x_offset_ = 0;
  float quat_y_offset_ = 0;
  float quat_z_offset_ = 0;

  float quat_w_ = 1;
  float quat_x_ = 0;
  float quat_y_ = 0;
  float quat_z_ = 0;

  bool write_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x00, addr, size};
    rx_queue_.clear();
    tx_sem_.try_acquire(5);
    if (!uart_.transmit_it(buf.data(), 4)) {
      return false;
    }
    tx_sem_.try_acquire(5);
    if (!uart_.transmit_it(data, size)) {
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
    tx_sem_.try_acquire(5);
    if (!uart_.transmit_it(buf.data(), 4)) {
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

  float normalize_angle(float angle) {
    float res = fmod(angle, 2.0f * M_PI);
    if (res < 0) {
      return res + 2.0f * M_PI;
    }
    return res;
  }
};

} // namespace module
} // namespace stm32rcos
