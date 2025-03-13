#include "stm32rcos.hpp"

#ifdef HAL_CAN_MODULE_ENABLED

#include "stm32rcos/peripheral/bxcan.hpp"

using stm32rcos::peripheral::BxCAN;

BxCAN &BxCAN::get_instance(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    static BxCAN can1{hcan};
    return can1;
  }
  if (hcan->Instance == CAN2) {
    static BxCAN can2{hcan};
    return can2;
  }
  __builtin_unreachable();
}

#endif
