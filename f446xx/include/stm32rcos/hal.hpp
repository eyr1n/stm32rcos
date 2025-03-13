#pragma once

#include <stm32f4xx_hal.h>

namespace stm32rcos {

#ifdef HAL_CAN_MODULE_ENABLED
void *get_bxcan_context(CAN_HandleTypeDef *hcan);
void set_bxcan_context(CAN_HandleTypeDef *hcan, void *context);
#endif

#ifdef HAL_UART_MODULE_ENABLED
void *get_uart_context(UART_HandleTypeDef *huart);
void set_uart_context(UART_HandleTypeDef *huart, void *context);
#endif

} // namespace stm32rcos