#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <type_traits>

#include <FreeRTOS.h>
#include <task.h>

namespace stm32rcos {
namespace core {

template <class T>
constexpr std::underlying_type_t<T> to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}

template <class... Args> std::string format(const char *fmt, Args... args) {
  size_t size = std::snprintf(nullptr, 0, fmt, args...);
  std::string buf(size, '\0');
  std::snprintf(buf.data(), size + 1, fmt, args...);
  return buf;
}

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