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
  WS2812B(TIM_HandleTypeDef *htim, uint32_t channel, size_t size)
      : htim_{htim}, channel_{channel}, rgbs_(size, RGB{}),
        buf_(RESET_SIZE + size * 24, 0) {
    HAL_TIM_PWM_Start_DMA(htim_, channel_,
                          reinterpret_cast<uint32_t *>(buf_.data()),
                          buf_.size());
  }

  ~WS2812B() { HAL_TIM_PWM_Stop_DMA(htim_, channel_); }

  void set_rgb(size_t index, const RGB &rgb) {
    float R = std::clamp(rgb.r, 0.0f, 1.0f);
    float G = std::clamp(rgb.g, 0.0f, 1.0f);
    float B = std::clamp(rgb.b, 0.0f, 1.0f);
    uint8_t r_byte = R * 255.0f;
    uint8_t g_byte = G * 255.0f;
    uint8_t b_byte = B * 255.0f;

    __disable_irq();
    // G
    size_t offset = RESET_SIZE + index * 24;
    for (size_t i = 0; i < 8; i++) {
      if (g_byte & (1 << (7 - i))) {
        buf_[offset + i] = BIT_ONE;
      } else {
        buf_[offset + i] = BIT_ZERO;
      }
    }
    // R
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
    __enable_irq();
  }

  void set_rgb_all(const RGB &rgb) {
    for (size_t i = 0; i < rgbs_.size(); i++) {
      set_rgb(i, rgb);
    }
  }

  size_t size() { return rgbs_.size(); }

private:
  static constexpr size_t RESET_SIZE = 50;
  static constexpr uint8_t BIT_ONE = 16;
  static constexpr uint8_t BIT_ZERO = 8;

  TIM_HandleTypeDef *htim_;
  uint32_t channel_;
  std::vector<RGB> rgbs_;
  std::vector<uint32_t> buf_;
};

} // namespace module
} // namespace stm32rcos