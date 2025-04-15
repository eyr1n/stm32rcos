#pragma once

#include "stm32rcos/hal.hpp"

#include "impl/can_base.hpp"

#ifdef HAL_CAN_MODULE_ENABLED
#include "impl/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "impl/fdcan.hpp"
#endif

namespace stm32rcos {
namespace peripheral {

template <class Handle> class Can;
template <class Handle> Can(Handle) -> Can<Handle>;

} // namespace peripheral
} // namespace stm32rcos