#pragma once

#include "main.h"

#include <algorithm>
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
  double H = std::clamp(hsv.h, 0.0f, 360.0f); // 色相 0〜360
  double S = std::clamp(hsv.s, 0.0f, 1.0f);   // 彩度 0〜1
  double V = std::clamp(hsv.v, 0.0f, 1.0f);   // 明度 0〜1
  float C = V * S;                            // 彩度に対する最大の色成分の差
  float X = C * (1 - fabsf(fmodf(H / 60.0, 2) - 1));
  float m = V - C;

  float r_prime, g_prime, b_prime;

  if (H >= 0 && H < 60) {
    r_prime = C;
    g_prime = X;
    b_prime = 0;
  } else if (H >= 60 && H < 120) {
    r_prime = X;
    g_prime = C;
    b_prime = 0;
  } else if (H >= 120 && H < 180) {
    r_prime = 0;
    g_prime = C;
    b_prime = X;
  } else if (H >= 180 && H < 240) {
    r_prime = 0;
    g_prime = X;
    b_prime = C;
  } else if (H >= 240 && H < 300) {
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
  WS2812B(TIM_HandleTypeDef *handle, uint32_t ch, size_t led_cnt)
      : handle_(handle), ch_(ch), led_cnt_(led_cnt) {
    buf_.resize(RESET_SIZE + led_cnt_ * 24);

    // set to 0
    for (size_t i = 0; i < buf_.size(); i++) {
      buf_[i] = 0;
    }
  }

  void write_rgb(size_t index, const RGB &rgb) {
    uint8_t r_byte, g_byte, b_byte;

    float R = std::clamp(rgb.r, 0.0f, 1.0f);
    float G = std::clamp(rgb.g, 0.0f, 1.0f);
    float B = std::clamp(rgb.b, 0.0f, 1.0f);
    r_byte = R * 255.0f;
    g_byte = G * 255.0f;
    b_byte = B * 255.0f;

    // G
    size_t offset = RESET_SIZE + index * 24;
    for (size_t i = 0; i < 8; i++) {
      if (g_byte & (1 << (7 - i))) {
        buf_[offset + i] = BIT_ONE;
      } else {
        buf_[offset + i] = BIT_ZERO;
      }
    }
    // B
    for (size_t i = 0; i < 8; i++) {
      if (r_byte & (1 << (7 - i))) {
        buf_[offset + 8 + i] = BIT_ONE;
      } else {
        buf_[offset + 8 + i] = BIT_ZERO;
      }
    }
    // B
    for (size_t i = 0; i < 8; i++) {
      if (b_byte & (1 << (7 - i))) {
        buf_[offset + 16 + i] = BIT_ONE;
      } else {
        buf_[offset + 16 + i] = BIT_ZERO;
      }
    }
  }

  void write_rgb_all(const RGB &rgb) {
    for (size_t i = 0; i < led_cnt_; i++) {
      write_rgb(i, rgb);
    }
  }

  void update() {
    HAL_TIM_PWM_Start_DMA(handle_, ch_, (uint32_t *)buf_.data(),
                          RESET_SIZE + led_cnt_ * 24);
  }

  size_t led_count() const { return led_cnt_; }

private:
  TIM_HandleTypeDef *handle_;
  uint32_t ch_;
  std::vector<uint32_t> buf_;
  static constexpr size_t RESET_SIZE = 50;
  static constexpr uint8_t BIT_ONE = 16;
  static constexpr uint8_t BIT_ZERO = 8;
  const size_t led_cnt_;
};

} // namespace module
} // namespace stm32rcos