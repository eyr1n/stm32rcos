#pragma once

#ifdef HAL_CAN_MODULE_ENABLED
#include "peripheral/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "peripheral/fdcan.hpp"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "peripheral/uart.hpp"
#include "peripheral/uart_dma.hpp"
#endif