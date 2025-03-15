#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <Eigen/Geometry>

#include "stm32rcos/core.hpp"

#include "stm32rcos/peripheral/uart.hpp"

namespace stm32rcos {
namespace module {

class BNO055 {
public:
  BNO055(peripheral::UART &uart) : uart_{uart} {}

  bool start(uint32_t timeout) {
    core::TimeoutHelper timeout_helper;
    while (!timeout_helper.is_timeout(timeout)) {
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

  std::optional<Eigen::Quaternionf> get_quaternion() {
    std::array<int16_t, 4> data;
    if (!read_reg(0x20, reinterpret_cast<uint8_t *>(data.data()), 8)) {
      return std::nullopt;
    }
    return Eigen::Quaternionf{data[0] / 16384.0f, data[1] / 16384.0f,
                              data[2] / 16384.0f, data[3] / 16384.0f};
  }

private:
  peripheral::UART &uart_;

  bool write_reg(uint8_t addr, const uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x00, addr, size};
    uart_.flush();
    if (!uart_.transmit(buf.data(), buf.size(), 5)) {
      return false;
    }
    if (!uart_.transmit(data, size, 5)) {
      return false;
    }
    if (!uart_.receive(buf.data(), 2, 5)) {
      return false;
    }
    return buf[0] == 0xEE && buf[1] == 0x01;
  }

  bool read_reg(uint8_t addr, uint8_t *data, uint8_t size) {
    std::array<uint8_t, 4> buf{0xAA, 0x01, addr, size};
    uart_.flush();
    if (!uart_.transmit(buf.data(), 4, 5)) {
      return false;
    }
    if (!uart_.receive(buf.data(), 2, 5)) {
      return false;
    }
    if (buf[0] != 0xBB || buf[1] != size) {
      return false;
    }
    return uart_.receive(data, size, 5);
  }
};

} // namespace module
} // namespace stm32rcos
