#include "stm32rcos/hal.hpp"

#ifdef HAL_CAN_MODULE_ENABLED

static void **bxcan_context(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

void *stm32rcos::get_bxcan_context(CAN_HandleTypeDef *hcan) {
  return *bxcan_context(hcan);
}

void stm32rcos::set_bxcan_context(CAN_HandleTypeDef *hcan, void *context) {
  *bxcan_context(hcan) = context;
}

#endif
