// RubbleSense firmware - single-producer / single-consumer byte FIFO (lock-free).
#pragma once
#include <stdint.h>
#include <atomic>

template <uint16_t N>
class ByteRing {
  static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");
  static_assert(N <= 32768, "indices are 16 bit");

 public:
  // Producer side. Returns false (and counts the loss) when the ring is full.
  bool push(uint8_t b) {
    const uint16_t h = head_.load(std::memory_order_relaxed);
    const uint16_t t = tail_.load(std::memory_order_acquire);
    if (static_cast<uint16_t>(h - t) >= N) {
      overflow_++;
      return false;
    }
    buf_[h & (N - 1)] = b;
    head_.store(static_cast<uint16_t>(h + 1), std::memory_order_release);
    return true;
  }

  // Consumer side.
  bool pop(uint8_t& b) {
    const uint16_t t = tail_.load(std::memory_order_relaxed);
    const uint16_t h = head_.load(std::memory_order_acquire);
    if (h == t) return false;
    b = buf_[t & (N - 1)];
    tail_.store(static_cast<uint16_t>(t + 1), std::memory_order_release);
    return true;
  }

  uint16_t size() const {
    return static_cast<uint16_t>(head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire));
  }

  // Consumer side: discard everything that is queued.
  void clear() { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }

  uint32_t overflow() const { return overflow_; }

 private:
  uint8_t buf_[N];
  std::atomic<uint16_t> head_{0};
  std::atomic<uint16_t> tail_{0};
  volatile uint32_t overflow_ = 0;
};
