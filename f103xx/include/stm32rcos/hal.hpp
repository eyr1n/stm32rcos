#pragma once

#include <stm32f1xx_hal.h>

namespace stm32rcos {

#ifdef HAL_CAN_MODULE_ENABLED
void *get_bxcan_context(CAN_HandleTypeDef *hcan);
void set_bxcan_context(CAN_HandleTypeDef *hcan, void *context);
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