#include "stm32rcos/hal.hpp"

#include "macro.hpp"

#ifdef HAL_SPI_MODULE_ENABLED

static void **spi_context(SPI_HandleTypeDef *hspi) {
#ifdef SPI1
  DECLARE_CONTEXT(hspi, SPI1);
#endif
#ifdef SPI2
  DECLARE_CONTEXT(hspi, SPI2);
#endif
#ifdef SPI3
  DECLARE_CONTEXT(hspi, SPI3);
#endif
#ifdef SPI4
  DECLARE_CONTEXT(hspi, SPI4);
#endif
#ifdef SPI5
  DECLARE_CONTEXT(hspi, SPI5);
#endif
#ifdef SPI6
  DECLARE_CONTEXT(hspi, SPI6);
#endif
  __builtin_unreachable();
}

void *stm32rcos::hal::get_spi_context(SPI_HandleTypeDef *hspi) {
  return *spi_context(hspi);
}

void stm32rcos::hal::set_spi_context(SPI_HandleTypeDef *hspi, void *context) {
  *spi_context(hspi) = context;
}

#endif
