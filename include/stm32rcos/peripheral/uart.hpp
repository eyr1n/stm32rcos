#pragma once

#include "uart/uart.hpp"

#ifdef HAL_UART_MODULE_ENABLED
#include "uart/uart_dma.hpp"
#include "uart/uart_interrupt.hpp"
#include "uart/uart_polling.hpp"
#endif