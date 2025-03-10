#include "main.h"

#ifdef HAL_CAN_MODULE_ENABLED

#include "stm32rcos/peripheral/bxcan.hpp"

using stm32rcos::peripheral::BxCAN;

BxCAN &BxCAN::get_instance(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN) {
    static BxCAN can{hcan};
    return can;
  }
  __builtin_unreachable();
}

#endif
