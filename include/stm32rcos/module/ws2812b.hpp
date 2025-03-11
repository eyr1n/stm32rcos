#pragma once

#include "main.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm32rcos {
namespace module {

struct RGB {
  float r;
  float g;
  float b;
};

struct HSV {
  float h;
  float s;
  float v;
};

inline RGB to_rgb(const HSV &hsv) {
  float C = hsv.v * hsv.s; // 彩度に対する最大の色成分の差
  float X = C * (1 - fabsf(fmodf(hsv.h / 60.0, 2) - 1));
  float m = hsv.v - C;

  float r_prime, g_prime, b_prime;

  if (hsv.h >= 0 && hsv.h < 60) {
    r_prime = C;
    g_prime = X;
    b_prime = 0;
  } else if (hsv.h >= 60 && hsv.h < 120) {
    r_prime = X;
    g_prime = C;
    b_prime = 0;
  } else if (hsv.h >= 120 && hsv.h < 180) {
    r_prime = 0;
    g_prime = C;
    b_prime = X;
  } else if (hsv.h >= 180 && hsv.h < 240) {
    r_prime = 0;
    g_prime = X;
    b_prime = C;
  } else if (hsv.h >= 240 && hsv.h < 300) {
    r_prime = X;
    g_prime = 0;
    b_prime = C;
  } else {
    r_prime = C;
    g_prime = 0;
    b_prime = X;
  }

  // RGB値に変換
  return {(r_prime + m), (g_prime + m), (b_prime + m)};
}

// 1.25us周期のPWMを生成する = 800kHz
// 25カウント周期
// 0.4us = 8カウント
// 0.85us = 16カウント
class WS2812B {
public:
  WS2812B(TIM_HandleTypeDef *htim, uint32_t channel, size_t size)
      : htim_{htim}, channel_{channel}, size_{size},
        buf_(RESET_CODE + size * 24, 0) {
    HAL_TIM_PWM_Start_DMA(htim_, channel_,
                          reinterpret_cast<uint32_t *>(buf_.data()),
                          buf_.size());
  }

  void set_rgb(size_t index, const RGB &rgb) {
    uint8_t r = rgb.r * 255.0f;
    uint8_t g = rgb.g * 255.0f;
    uint8_t b = rgb.b * 255.0f;

    // G
    size_t offset = RESET_CODE + index * 24;
    for (size_t i = 0; i < 8; ++i) {
      if (g & (1 << (7 - i))) {
        buf_[offset + i] = ONE_CODE;
      } else {
        buf_[offset + i] = ZERO_CODE;
      }
    }

    // R
    for (size_t i = 0; i < 8; ++i) {
      if (r & (1 << (7 - i))) {
        buf_[offset + 8 + i] = ONE_CODE;
      } else {
        buf_[offset + 8 + i] = ZERO_CODE;
      }
    }

    // B
    for (size_t i = 0; i < 8; ++i) {
      if (b & (1 << (7 - i))) {
        buf_[offset + 16 + i] = ONE_CODE;
      } else {
        buf_[offset + 16 + i] = ZERO_CODE;
      }
    }
  }

  void set_rgb_all(const RGB &rgb) {
    for (size_t i = 0; i < size_; ++i) {
      set_rgb(i, rgb);
    }
  }

  size_t size() { return size_; }

private:
  static constexpr uint8_t RESET_CODE = 50;
  static constexpr uint8_t ZERO_CODE = 8;
  static constexpr uint8_t ONE_CODE = 16;

  TIM_HandleTypeDef *htim_;
  uint32_t channel_;
  size_t size_;
  std::vector<uint32_t> buf_;
};

} // namespace module
} // namespace stm32rcos