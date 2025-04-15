#include "stm32rcos/hal.hpp"

#include "macro.hpp"

#ifdef HAL_FDCAN_MODULE_ENABLED

static void **fdcan_context(FDCAN_HandleTypeDef *hfdcan) {
#ifdef FDCAN1
  DECLARE_CONTEXT(hcan, FDCAN1);
#endif
#ifdef FDCAN2
  DECLARE_CONTEXT(hcan, FDCAN2);
#endif
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
