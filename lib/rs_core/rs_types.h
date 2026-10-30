// RubbleSense core - shared data types.
#pragma once
#include <stdint.h>
#include <atomic>
#include "rs_config.h"

namespace rs {

// One complex baseband sample (one range bin, one slow-time frame).
struct IqSample {
  int16_t i;
  int16_t q;
};

// Per-frame quality flags stored next to the I/Q data.
enum FrameFlag : uint8_t {
  kFlagImuMotion  = 1u << 0,  // IMU saw sensor motion during/just before this frame
  kFlagSaturated  = 1u << 1,  // radar ADC / I-Q clipped
  kFlagDropped    = 1u << 2,  // frame lost on the link; samples are a hold of the previous frame
  kFlagImuMissing = 1u << 3,  // no IMU data: stability could not be verified
  kFlagTiming     = 1u << 4,  // front-end timestamps disagree with the sequence numbers: the sampling grid is not what we assume
};

// Static description of the radar front-end (filled in by the HAL backend).
struct RadarInfo {
  const char* name;
  float   frame_rate_hz;   // slow-time frame rate
  float   range_start_m;   // range of bin 0 (air-equivalent)
  float   range_step_m;    // range spacing between bins (air-equivalent)
  float   wavelength_mm;   // carrier wavelength (informational; used by the simulator)
  uint8_t n_bins;          // bins delivered per frame (<= kMaxBins)
};

// One slow-time frame: a complex value for every range bin.
struct RadarFrame {
  IqSample iq[kMaxBins];
  uint8_t  n_bins;
  uint16_t seq;        // increments by 1 per frame (wraps) -> used to detect drops
  uint32_t t_ms;       // front-end or local timestamp
  bool     saturated;
};

// Linear segment buffer: single producer (acquisition task) appends frames,
// consumers (DSP task) may read frames [0, n_frames) at any time. Lock-free.
struct SegmentBuffer {
  IqSample iq[kMaxBins][kSegFrames];   // [bin][frame]: contiguous per bin for the FFT
  uint8_t  flags[kSegFrames];
  uint8_t  n_bins = 0;
  std::atomic<uint16_t> n_frames{0};

  void reset(uint8_t bins) {
    n_bins = (bins > kMaxBins) ? kMaxBins : bins;
    n_frames.store(0, std::memory_order_release);
  }
  uint16_t size() const { return n_frames.load(std::memory_order_acquire); }
  bool full() const { return size() >= kSegFrames; }

  bool push(const RadarFrame& f, uint8_t flg) {
    const uint16_t n = n_frames.load(std::memory_order_relaxed);
    if (n >= kSegFrames) return false;
    for (uint8_t b = 0; b < n_bins; b++) {
      if (b < f.n_bins) iq[b][n] = f.iq[b];
      else              iq[b][n] = IqSample{0, 0};
    }
    flags[n] = flg;
    n_frames.store(static_cast<uint16_t>(n + 1), std::memory_order_release);
    return true;
  }
};

}  // namespace rs
