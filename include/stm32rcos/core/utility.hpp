#pragma once

#include <cstdint>

#include <FreeRTOS.h>
#include <task.h>

namespace stm32rcos {
namespace core {

class TimeoutHelper {
public:
  TimeoutHelper() { vTaskSetTimeOutState(&timeout_state_); }

  bool is_timeout(uint32_t &timeout) {
    return xTaskCheckForTimeOut(&timeout_state_, &timeout) == pdTRUE;
  }

private:
  TimeOut_t timeout_state_;
};

} // namespace core
} // namespace stm32rcos