#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "stm32rcos/peripheral/uart.hpp"

#include "encoder_base.hpp"

namespace stm32rcos {
namespace module {

enum class STS3215WorkMode {
  RAD = 0,
  PWM = 2,
};

enum class STS3215Mode {
  SINGLE_TURN,
  MULTI_TURN,
};

/**
 *
 * modeはSTS3215のworkmodeの設定に応じて選択してください。
 * シリアル通信のbaudrateが1Mbpsだと、オーバーランエラーになる可能性があります（処理能力依存）
 * 500kbps以下を推奨します。
 *
 * @code{.cpp}
 * #include <cstdio>
 * #include <tutrcos.hpp>
 * #include <tutrcos/module/sts3215.hpp>
 *
 * extern UART_HandleTypeDef huart2;
 * extern UART_HandleTypeDef huart4;
 *
 * extern "C" void main_thread(void *) {
 *   using namespace tutrcos::core;
 *   using namespace tutrcos::peripheral;
 *   using namespace tutrcos::module;
 *
 *   UART uart2(&huart2); // デバッグ出力用
 *   uart2.enable_stdout();
 *
 *   UART uart3(&huart4);
 *   STS3215 sts(uart3, STS3215::WorkMode::PWM, 10, STS3215::Mode::MULTI_TURN);
 *
 *   while (true) {
 *     if(!sts.update()){ // データ送受信
 *       printf("error\r\n");
 *     }else{
 *       float Kp = 5;
 *       float x_target = 0.5f;
 *       // 現在の角度をrotationで取得
 *       float x_actual = sts.get_rotation();
 *       float error = x_target - x_actual;
 *
 *       // 出力(-1~1)を指定
 *       sts.set_input(Kp * error);
 *       sts.transmit();
 *
 *       // STS3215の回転速度と絶対位置を出力
 *       printf("%f %f\r\n", sts.get_rps(), sts.get_rotation());
 *     }
 *
 *     Thread::delay(10);
 *   }
 * }
 * @endcode
 */
class STS3215 : public EncoderBase {
public:
  STS3215(peripheral::UART &uart, STS3215WorkMode workmode, uint8_t id, STS3215Mode mode)
      : EncoderBase{ppr_}, uart_{uart}, workmode_{workmode}, id_{id},
        mode_{mode} {
    uart_.attach_tx_callback(
        [](void *args) {
          auto sts3215 = reinterpret_cast<STS3215 *>(args);
          sts3215->tx_sem_.release();
        },
        this);
    uart_.attach_rx_callback(
        [](void *args) {
          auto sts3215 = reinterpret_cast<STS3215 *>(args);
          sts3215->rx_queue_.push(sts3215->rx_buf_, 0);
          sts3215->uart_.receive_it(&sts3215->rx_buf_, 1);
        },
        this);
    uart_.attach_abort_callback(
        [](void *args) {
          auto sts3215 = reinterpret_cast<STS3215 *>(args);
          sts3215->tx_sem_.release();
          sts3215->uart_.receive_it(&sts3215->rx_buf_, 1);
        },
        this);
    uart_.receive_it(&rx_buf_, 1);
  }

  ~STS3215() {
    uart_.abort();
    uart_.detach_tx_callback();
    uart_.detach_rx_callback();
    uart_.detach_abort_callback();
  }

  bool update() {
    uint8_t rx_data[8] = {0};
    rx_queue_.clear();
    if (send({0x02, 0x38, 0x02})) {
      // rx_data : 0xff 0xff id size cmd data data checksum
      for (size_t i = 0; i < 8; ++i) {
        if (!rx_queue_.pop(rx_data[i], 1)) {
          return false;
        }
      }
      uint8_t checksum = 0;
      for (uint8_t i = 2; i < 8; i++) {
        checksum += rx_data[i];
      }
      if ((rx_data[0] == 0xff) && (rx_data[1] == 0xff) && (checksum == 0xff) &&
          (rx_data[2] == id_)) {

        int16_t count = static_cast<int16_t>(rx_data[6] << 8) | rx_data[5];
        int16_t delta = count - prev_count_;

        switch (mode_) {
        case STS3215Mode::SINGLE_TURN: {
          set_count(count);
          break;
        }
        case STS3215Mode::MULTI_TURN: {
          if (delta > (get_cpr() / 2)) {
            delta -= get_cpr();
          } else if (delta < -(get_cpr() / 2)) {
            delta += get_cpr();
          }
          set_count(get_count() + delta);
          break;
        }
        }

        prev_count_ = count;
      }
    }
    return true;
  }

  void set_input(float value) { input_ = value; }
  STS3215Mode get_mode() { return mode_; };

  bool transmit() {
    int16_t target = 0;
    uint8_t upper, lower;
    bool res = true;
    switch (workmode_) {
    case STS3215WorkMode::RAD:
      input_ = std::clamp<float>(input_, 0, 2 * M_PI);
      target = input_ / (2 * M_PI) * (ppr_ - 1);
      upper = static_cast<uint8_t>(target >> 8);
      lower = static_cast<uint8_t>(target);
      res = send({0x03, 0x2A, lower, upper});
      break;
    case STS3215WorkMode::PWM:
      input_ = std::clamp<float>(input_, -1, 1);
      target = static_cast<uint16_t>(abs(input_ * 1023));
      upper = static_cast<uint8_t>(target >> 8) | ((input_ > 0) ? 0x04 : 0);
      lower = static_cast<uint8_t>(target);
      res = send({0x03, 0x2C, lower, upper});
      break;
    }
    return res;
  }

private:
  peripheral::UART &uart_;
  core::Semaphore tx_sem_{1, 1};
  core::Queue<uint8_t> rx_queue_{64};
  uint8_t rx_buf_;

  inline static constexpr uint16_t ppr_ = 4096;
  const STS3215WorkMode workmode_;
  const uint8_t id_;
  const STS3215Mode mode_;
  int16_t prev_count_ = 0;
  float input_ = 0;

  bool send(std::vector<uint8_t> tx) {
    uint8_t size = tx.size() + 1;
    tx.insert(tx.begin(), {0xff, 0xff, id_, size});
    uint8_t checksum = 0;
    for (uint8_t i = 2, size = tx.size(); i < size; i++) {
      checksum += tx[i];
    }
    tx.emplace_back(~checksum);
    tx_sem_.try_acquire(1);
    return uart_.transmit_it(tx.data(), size + 5);
  }
};

} // namespace module
} // namespace stm32rcos