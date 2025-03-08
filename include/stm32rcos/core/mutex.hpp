#pragma once

#include <cstdint>
#include <memory>
#include <type_traits>

#include <cmsis_os2.h>

namespace stm32rcos {
namespace core {

class Mutex {
private:
  struct Deleter {
    void operator()(osMutexId_t mutex_id) { osMutexDelete(mutex_id); }
  };

  using MutexId = std::unique_ptr<std::remove_pointer_t<osMutexId_t>, Deleter>;

public:
  Mutex(uint32_t attr_bits = 0) {
    osMutexAttr_t attr{};
    attr.attr_bits = attr_bits;
    mutex_id_ = MutexId{osMutexNew(&attr)};
  }

  bool try_lock(uint32_t timeout) {
    return osMutexAcquire(mutex_id_.get(), timeout) == osOK;
  }

  void lock() { try_lock(osWaitForever); }

  void unlock() { osMutexRelease(mutex_id_.get()); }

private:
  MutexId mutex_id_;
};

} // namespace core
} // namespace stm32rcos
