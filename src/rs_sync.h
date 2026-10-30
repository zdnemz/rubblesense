// RubbleSense firmware - tiny synchronisation helpers shared by the tasks.
//
// Threading model (see docs/ARSITEKTUR.md):
//   * acquisition task (core 0, high priority): IMU sampling, radar source polling, UART pump
//   * DSP task        (core 0, low priority)   : analyzeSegment()
//   * Arduino loop    (core 1)                 : UI, buttons, console, scan state machine
// Data crosses threads through (a) the lock-free SegmentBuffer, (b) SPSC byte rings,
// (c) small status structs copied under a spin-lock, (d) one atomic job state in DspWorker.
#pragma once
#include <stdint.h>
#include "rs_build.h"

#if RS_HOST
struct RsLock {
  void enter() {}
  void exit() {}
};
#else
#include "freertos/FreeRTOS.h"
struct RsLock {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  void enter() { portENTER_CRITICAL(&mux); }
  void exit() { portEXIT_CRITICAL(&mux); }
};
#endif

// RAII guard. Keep the guarded section to a struct copy: interrupts are off inside it.
struct RsGuard {
  RsLock& l;
  explicit RsGuard(RsLock& x) : l(x) { l.enter(); }
  ~RsGuard() { l.exit(); }
  RsGuard(const RsGuard&) = delete;
  RsGuard& operator=(const RsGuard&) = delete;
};

// Wrap-safe "is `now` at or after `t`" for 32-bit microsecond / millisecond counters.
inline bool rsReached(uint32_t now, uint32_t t) { return static_cast<int32_t>(now - t) >= 0; }
