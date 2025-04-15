#pragma once

#include "can/can.hpp"

#ifdef HAL_CAN_MODULE_ENABLED
#include "can/bxcan.hpp"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "can/fdcan.hpp"
#endif

namespace stm32rcos {
namespace peripheral {

template <class Handle> class Can;
template <class Handle> Can(Handle) -> Can<Handle>;

} // namespace peripheral
} // namespace stm32rcos