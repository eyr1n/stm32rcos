#include "stm32rcos/hal.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

static void **tim_context(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM2) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM3) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM6) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM7) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM15) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM16) {
    static void *context;
    return &context;
  }
  if (htim->Instance == TIM17) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

void *stm32rcos::hal::get_tim_context(TIM_HandleTypeDef *htim) {
  return *tim_context(htim);
}

void stm32rcos::hal::set_tim_context(TIM_HandleTypeDef *htim, void *context) {
  *tim_context(htim) = context;
}

#endif
