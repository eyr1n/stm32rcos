#include "stm32rcos/hal.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

static void **spi_context(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    static void *context;
    return &context;
  }
  __builtin_unreachable();
}

void *stm32rcos::get_spi_context(SPI_HandleTypeDef *hspi) {
  return *spi_context(hspi);
}

void stm32rcos::set_spi_context(SPI_HandleTypeDef *hspi, void *context) {
  *spi_context(hspi) = context;
}

#endif
