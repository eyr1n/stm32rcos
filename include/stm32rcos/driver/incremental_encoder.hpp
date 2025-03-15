#pragma once

#include <cstdint>

#include "stm32rcos/hal.hpp"

#include "encoder_base.hpp"

namespace stm32rcos {
namespace driver {

class IncrementalEncoder  {
public:
  IncrementalEncoder(TIM_HandleTypeDef *htim)
      : htim_{htim} {
    HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
  }

  ~IncrementalEncoder() { HAL_TIM_Encoder_Stop(htim_, TIM_CHANNEL_ALL); }

  void update() {
    int16_t delta = __HAL_TIM_GET_COUNTER(htim_);
    __HAL_TIM_SET_COUNTER(htim_, 0);
    count_ += delta;
  }

  int64_t get_count() { return count_; }

private:
  TIM_HandleTypeDef *htim_;
  int64_t count_ = 0;
};

} // namespace driver
} // namespace stm32rcos
