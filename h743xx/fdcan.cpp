#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "stm32rcos/peripheral/fdcan.hpp"

using stm32rcos::peripheral::FDCAN;

FDCAN &FDCAN::get_instance(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan->Instance == FDCAN1) {
    static FDCAN can1{hfdcan};
    return can1;
  }
  if (hfdcan->Instance == FDCAN2) {
    static FDCAN can2{hfdcan};
    return can2;
  }
  __builtin_unreachable();
}

#endif
