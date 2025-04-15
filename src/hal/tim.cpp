#include "stm32rcos/hal.hpp"

#include "macro.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

static void **tim_context(TIM_HandleTypeDef *htim) {
#ifdef TIM1
  DECLARE_CONTEXT(htim, TIM1);
#endif
#ifdef TIM2
  DECLARE_CONTEXT(htim, TIM2);
#endif
#ifdef TIM3
  DECLARE_CONTEXT(htim, TIM3);
#endif
#ifdef TIM4
  DECLARE_CONTEXT(htim, TIM4);
#endif
#ifdef TIM5
  DECLARE_CONTEXT(htim, TIM5);
#endif
#ifdef TIM6
  DECLARE_CONTEXT(htim, TIM6);
#endif
#ifdef TIM7
  DECLARE_CONTEXT(htim, TIM7);
#endif
#ifdef TIM8
  DECLARE_CONTEXT(htim, TIM8);
#endif
#ifdef TIM9
  DECLARE_CONTEXT(htim, TIM9);
#endif
#ifdef TIM10
  DECLARE_CONTEXT(htim, TIM10);
#endif
#ifdef TIM11
  DECLARE_CONTEXT(htim, TIM11);
#endif
#ifdef TIM12
  DECLARE_CONTEXT(htim, TIM12);
#endif
#ifdef TIM13
  DECLARE_CONTEXT(htim, TIM13);
#endif
#ifdef TIM14
  DECLARE_CONTEXT(htim, TIM14);
#endif
#ifdef TIM15
  DECLARE_CONTEXT(htim, TIM15);
#endif
#ifdef TIM16
  DECLARE_CONTEXT(htim, TIM16);
#endif
#ifdef TIM17
  DECLARE_CONTEXT(htim, TIM17);
#endif
  __builtin_unreachable();
}

void *stm32rcos::hal::get_tim_context(TIM_HandleTypeDef *htim) {
  return *tim_context(htim);
}

void stm32rcos::hal::set_tim_context(TIM_HandleTypeDef *htim, void *context) {
  *tim_context(htim) = context;
}

#endif
