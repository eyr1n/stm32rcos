#include "stm32rcos/hal.hpp"

#ifdef HAL_FDCAN_MODULE_ENABLED

static void **fdcan_context(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan->Instance == FDCAN1) {
    static void *context;
    return &context;
  }
  if (hfdcan->Instance == FDCAN2) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

void *stm32rcos::hal::get_fdcan_context(FDCAN_HandleTypeDef *hfdcan) {
  return *fdcan_context(hfdcan);
}

void stm32rcos::hal::set_fdcan_context(FDCAN_HandleTypeDef *hfdcan,
                                       void *context) {
  *fdcan_context(hfdcan) = context;
}

#endif
