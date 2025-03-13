#pragma once

#include <stm32h7xx_hal.h>

namespace stm32rcos {

#ifdef HAL_FDCAN_MODULE_ENABLED
void *get_fdcan_context(FDCAN_HandleTypeDef *hfdcan);
void set_fdcan_context(FDCAN_HandleTypeDef *hfdcan, void *context);
#endif

#ifdef HAL_UART_MODULE_ENABLED
void *get_uart_context(UART_HandleTypeDef *huart);
void set_uart_context(UART_HandleTypeDef *huart, void *context);
#endif

} // namespace stm32rcos