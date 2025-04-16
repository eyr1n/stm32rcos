import re
import sys
from pathlib import Path


def get_defines(src):
    headers = "\n".join(map(lambda header: header.read_text(), src.glob("**/*.h")))
    return sorted(set(re.findall(r"^\s*#define\s+(.+?)\s+.+$", headers, re.MULTILINE)))


def get_matched_defines(defines, regex):
    return list(filter(lambda x: re.match(regex, x), defines))


def declare_context(handle, instance):
    return f"""#ifdef {instance}
  if ({handle}->Instance == {instance}) {{
    static void *context;
    return &context;
  }}
#endif"""


def declare_contexts(handle, instances):
    return "\n".join(map(lambda instance: declare_context(handle, instance), instances))


src = Path(sys.argv[1])
dest = Path(sys.argv[2])
dest.mkdir(parents=True, exist_ok=True)

defines = get_defines(src)


# BxCAN
(dest / "bxcan.cpp").write_text(f"""#include <utility>
#include "stm32rcos/hal.hpp"
#ifdef HAL_CAN_MODULE_ENABLED
static void **bxcan_context(CAN_HandleTypeDef *hcan) {{
{declare_contexts("hcan", get_matched_defines(defines, "^CAN[0-9]*$"))}
  std::unreachable();
}}
void *stm32rcos::hal::get_bxcan_context(CAN_HandleTypeDef *hcan) {{
  return *bxcan_context(hcan);
}}
void stm32rcos::hal::set_bxcan_context(CAN_HandleTypeDef *hcan, void *context) {{
  *bxcan_context(hcan) = context;
}}
#endif
""")


# FDCAN
(dest / "fdcan.cpp").write_text(f"""#include <utility>
#include "stm32rcos/hal.hpp"
#ifdef HAL_FDCAN_MODULE_ENABLED
static void **fdcan_context(FDCAN_HandleTypeDef *hfdcan) {{
{declare_contexts("hfdcan", get_matched_defines(defines, "^FDCAN[0-9]*$"))}
  std::unreachable();
}}
void *stm32rcos::hal::get_fdcan_context(FDCAN_HandleTypeDef *hfdcan) {{
  return *fdcan_context(hfdcan);
}}
void stm32rcos::hal::set_fdcan_context(FDCAN_HandleTypeDef *hfdcan, void *context) {{
  *fdcan_context(hfdcan) = context;
}}
#endif
""")


# SPI
(dest / "spi.cpp").write_text(f"""#include <utility>
#include "stm32rcos/hal.hpp"
#ifdef HAL_SPI_MODULE_ENABLED
static void **spi_context(SPI_HandleTypeDef *hspi) {{
{declare_contexts("hspi", get_matched_defines(defines, "^SPI[0-9]*$"))}
  std::unreachable();
}}
void *stm32rcos::hal::get_spi_context(SPI_HandleTypeDef *hspi) {{
  return *spi_context(hspi);
}}
void stm32rcos::hal::set_spi_context(SPI_HandleTypeDef *hspi, void *context) {{
  *spi_context(hspi) = context;
}}
#endif
""")


# TIM
(dest / "tim.cpp").write_text(f"""#include <utility>
#include "stm32rcos/hal.hpp"
#ifdef HAL_TIM_MODULE_ENABLED
static void **tim_context(TIM_HandleTypeDef *htim) {{
{declare_contexts("htim", get_matched_defines(defines, "^TIM[0-9]*$"))}
  std::unreachable();
}}
void *stm32rcos::hal::get_tim_context(TIM_HandleTypeDef *htim) {{
  return *tim_context(htim);
}}
void stm32rcos::hal::set_tim_context(TIM_HandleTypeDef *htim, void *context) {{
  *tim_context(htim) = context;
}}
#endif
""")


# UART
(dest / "uart.cpp").write_text(f"""#include <utility>
#include "stm32rcos/hal.hpp"
#ifdef HAL_UART_MODULE_ENABLED
static void **uart_context(UART_HandleTypeDef *huart) {{
{declare_contexts("huart", get_matched_defines(defines, "^US?ART[0-9]*$"))}
  std::unreachable();
}}
void *stm32rcos::hal::get_uart_context(UART_HandleTypeDef *huart) {{
  return *uart_context(huart);
}}
void stm32rcos::hal::set_uart_context(UART_HandleTypeDef *huart, void *context) {{
  *uart_context(huart) = context;
}}
#endif
""")
