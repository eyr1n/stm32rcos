#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <type_traits>

#include <cmsis_os2.h>

namespace stm32rcos {
namespace core {

class Thread {
private:
  struct Deleter {
    void operator()(osThreadId_t thread_id) { osThreadTerminate(thread_id); }
  };

  using ThreadId =
      std::unique_ptr<std::remove_pointer_t<osThreadId_t>, Deleter>;

public:
  Thread(std::function<void()> &&func, size_t stack_size, osPriority_t priority,
         uint32_t attr_bits = 0)
      : func_{std::move(func)} {
    osThreadAttr_t attr{};
    attr.stack_size = stack_size;
    attr.priority = priority;
    attr.attr_bits = attr_bits;
    thread_id_ = ThreadId{osThreadNew(func_internal, this, &attr)};
  }

  bool detach() { return osThreadDetach(thread_id_.get()) == osOK; }

  bool join() { return osThreadJoin(thread_id_.get()) == osOK; }

private:
  ThreadId thread_id_;
  std::function<void()> func_;

  static inline void func_internal(void *thread) {
    reinterpret_cast<stm32rcos::core::Thread *>(thread)->func_();
    osThreadExit();
  }
};

} // namespace core
} // namespace stm32rcos
