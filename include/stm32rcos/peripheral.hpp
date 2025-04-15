#pragma once

#include "peripheral/can.hpp"
#include "peripheral/uart.hpp"

#ifdef HAL_CAN_MODULE_ENABLED
#include "peripheral/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "peripheral/fdcan.hpp"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "peripheral/uart_dma.hpp"
#include "peripheral/uart_interrupt.hpp"
#include "peripheral/uart_polling.hpp"
#endif