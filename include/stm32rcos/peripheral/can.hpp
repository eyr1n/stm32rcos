#pragma once

#include "can/can_base.hpp"

#ifdef HAL_CAN_MODULE_ENABLED
#include "can/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "can/fdcan.hpp"
#endif