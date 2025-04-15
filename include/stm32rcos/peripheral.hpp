#pragma once

#include "peripheral/can_base.hpp"
#include "peripheral/uart_base.hpp"

#ifdef HAL_CAN_MODULE_ENABLED
#include "peripheral/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "peripheral/fdcan.hpp"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "peripheral/uart_dma.hpp"
#include "peripheral/uart_it.hpp"
#endif