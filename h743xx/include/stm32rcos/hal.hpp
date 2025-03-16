#pragma once

#include <stm32h7xx_hal.h>

namespace stm32rcos {

#ifdef HAL_FDCAN_MODULE_ENABLED
void *get_fdcan_context(FDCAN_HandleTypeDef *hfdcan);
void set_fdcan_context(FDCAN_HandleTypeDef *hfdcan, void *context);
#endif

#ifdef HAL_SPI_MODULE_ENABLED
void *get_spi_context(SPI_HandleTypeDef *hspi);
void set_spi_context(SPI_HandleTypeDef *hspi, void *context);
#endif

#ifdef HAL_TIM_MODULE_ENABLED
void *get_tim_context(TIM_HandleTypeDef *htim);
void set_tim_context(TIM_HandleTypeDef *htim, void *context);
#endif

#ifdef HAL_UART_MODULE_ENABLED
void *get_uart_context(UART_HandleTypeDef *huart);
void set_uart_context(UART_HandleTypeDef *huart, void *context);
#endif

} // namespace stm32rcos