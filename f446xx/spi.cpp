#include "stm32rcos/hal.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

static void **spi_context(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == USART1) {
    static void *context;
    return &context;
  }
  if (hspi->Instance == USART2) {
    static void *context;
    return &context;
  }
  if (hspi->Instance == USART3) {
    static void *context;
    return &context;
  }
  if (hspi->Instance == UART4) {
    static void *context;
    return &context;
  }
  if (hspi->Instance == UART5) {
    static void *context;
    return &context;
  }
  if (hspi->Instance == USART6) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

void *stm32rcos::get_uart_context(SPI_HandleTypeDef *hspi) {
  return *uart_context(hspi);
}

void stm32rcos::set_uart_context(SPI_HandleTypeDef *hspi, void *context) {
  *uart_context(hspi) = context;
}

#endif
