#pragma once

#include "uart.hpp"

namespace stm32rcos {
namespace peripheral {

bool enable_stdout(UartBase &uart);

bool disable_stdout();

} // namespace peripheral
} // namespace stm32rcos
