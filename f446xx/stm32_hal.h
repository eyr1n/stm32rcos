#pragma once

#include <stm32f4xx_hal.h>

namespace stm32rcos {

#ifdef HAL_UART_MODULE_ENABLED
void *get_uart_context(UART_HandleTypeDef *huart);
void set_uart_context(UART_HandleTypeDef *huart, void *context);
#endif

} // namespace stm32rcos