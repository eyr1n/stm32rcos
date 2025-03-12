#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "stm32rcos/core/queue.hpp"
#include "stm32rcos/core/semaphore.hpp"
#include "stm32rcos/peripheral/uart.hpp"

#include "encoder_base.hpp"

namespace stm32rcos {
namespace module {

enum class AMT21Resolution : uint8_t {
  _12 = 12,
  _14 = 14,
};

enum class AMT21Mode {
  SINGLE_TURN,
  MULTI_TURN,
};

class AMT21 : public EncoderBase {
public:
  AMT21(peripheral::UART &uart, AMT21Resolution resolution, AMT21Mode mode,
        uint8_t address)
      : EncoderBase{1 << static_cast<std::underlying_type_t<AMT21Resolution>>(
                        resolution)},
        uart_{uart}, resolution_{resolution}, mode_{mode}, address_{address} {
    uart_.attach_tx_callback(
        [](void *args) {
          auto amt21 = reinterpret_cast<AMT21 *>(args);
          amt21->tx_sem_.release();
        },
        this);
    uart_.attach_rx_callback(
        [](void *args) {
          auto amt21 = reinterpret_cast<AMT21 *>(args);
          amt21->rx_sem_.release();
        },
        this);
  }

  ~AMT21() {
    uart_.abort();
    uart_.detach_tx_callback();
    uart_.detach_rx_callback();
  }

  bool update() {
    uint16_t cpr =
        1 << static_cast<std::underlying_type_t<AMT21Resolution>>(resolution_);
    uint16_t response;
    if (!send_command(0x00, reinterpret_cast<uint8_t *>(&response))) {
      return false;
    }

    int16_t count = response & (cpr - 1);
    switch (mode_) {
    case AMT21Mode::SINGLE_TURN: {
      set_count(count);
      break;
    }
    case AMT21Mode::MULTI_TURN: {
      int16_t delta = count - prev_count_;
      if (delta > (cpr / 2)) {
        delta -= cpr;
      } else if (delta < -(cpr / 2)) {
        delta += cpr;
      }
      set_count(get_count() + delta);
      prev_count_ = count;
      break;
    }
    }
    return true;
  }

  bool set_zero_point() {
    if (!send_extended_command(0x5E)) {
      return false;
    }
    prev_count_ = 0;
    set_count(0);
    return true;
  }

private:
  peripheral::UART &uart_;
  core::Semaphore tx_sem_{1, 1};
  core::Semaphore rx_sem_{1, 1};

  AMT21Resolution resolution_;
  AMT21Mode mode_;
  uint8_t address_;
  int16_t prev_count_ = 0;

  bool send_command(uint8_t command, uint8_t *response) {
    uint8_t data = address_ | command;
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (!uart_.receive_dma(reinterpret_cast<uint8_t *>(response), 2)) {
      uart_.abort();
      return false;
    }
    if (!uart_.transmit_dma(&data, 1)) {
      uart_.abort();
      return false;
    }
    if (!tx_sem_.try_acquire(1)) {
      return false;
    }
    if (!rx_sem_.try_acquire(1)) {
      return false;
    }
    return checksum(response[0], response[1]);
  }

  bool send_extended_command(uint8_t command) {
    std::array<uint8_t, 2> data{static_cast<uint8_t>(address_ | 0x02), command};
    tx_sem_.try_acquire(0);
    rx_sem_.try_acquire(0);
    if (!uart_.transmit_dma(data.data(), data.size())) {
      uart_.abort();
      return false;
    }
    if (!tx_sem_.try_acquire(1)) {
      return false;
    }
    return true;
  }

  bool checksum(uint8_t l, uint8_t h) {
    bool k1 = !(bit(h, 5) ^ bit(h, 3) ^ bit(h, 1) ^ bit(l, 7) ^ bit(l, 5) ^
                bit(l, 3) ^ bit(l, 1));
    bool k0 = !(bit(h, 4) ^ bit(h, 2) ^ bit(h, 0) ^ bit(l, 6) ^ bit(l, 4) ^
                bit(l, 2) ^ bit(l, 0));
    return (k1 == bit(h, 7)) && (k0 == bit(h, 6));
  }

  bool bit(uint8_t x, uint8_t i) { return ((x >> i) & 1) == 1; }
};

} // namespace module
} // namespace stm32rcos