#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

template <class T, size_t N> class RingBuffer {
public:
  bool push(const T &value) {
    size_t write_idx = write_idx_.load(std::memory_order_relaxed);
    size_t next_idx = (write_idx + 1) % buf_.size();
    if (next_idx == read_idx_.load(std::memory_order_acquire)) {
      return false;
    }
    buf_[write_idx] = value;
    write_idx_.store(next_idx, std::memory_order_release);
    return true;
  }

  std::optional<T> pop() {
    size_t write_idx = write_idx_.load(std::memory_order_acquire);
    size_t read_idx = read_idx_.load(std::memory_order_relaxed);
    if (write_idx == read_idx) {
      return std::nullopt;
    }
    std::optional<T> value = buf_[read_idx];
    size_t next_idx = (read_idx + 1) % buf_.size();
    read_idx_.store(next_idx, std::memory_order_release);
    return value;
  }

  void clear() {
    write_idx_.store(0, std::memory_order_relaxed);
    read_idx_.store(0, std::memory_order_release);
  }

  size_t size() const {
    size_t write_idx = write_idx_.load(std::memory_order_acquire);
    size_t read_idx = read_idx_.load(std::memory_order_relaxed);
    return (write_idx + buf_.size() - read_idx) % buf_.size();
  }

  constexpr size_t capacity() const { return N; }

private:
  std::array<T, N + 1> buf_;
  std::atomic<size_t> write_idx_{0};
  std::atomic<size_t> read_idx_{0};
};
