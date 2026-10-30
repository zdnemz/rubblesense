// RubbleSense firmware - runs rs::analyzeSegment() off the UI thread.
//
// Device: a FreeRTOS task (core 0, low priority) woken by a semaphore. Host test-bench and
// Wokwi (RS_DSP_INLINE, see rs_build.h): the job runs inline inside submit*().
// One job at a time; the caller owns the buffer and must not reset it while a job is active.
#pragma once
#include <atomic>
#include <stdint.h>
#include "rs_config.h"
#include "rs_dsp.h"
#include "rs_types.h"

class DspWorker {
 public:
  void begin(rs::SegmentBuffer* buf, rs::Workspace* ws);

  bool busy() const { return state_.load(std::memory_order_acquire) != kIdle; }

  // Full analysis of the whole segment, with the seeds of the earlier segments.
  bool submitFull(const rs::DspParams& p, const rs::Seed* seeds, uint8_t n_seeds);
  // Live preview of the most recent kPrevFrames frames of the `n_avail` frames stored so far.
  bool submitPreview(const rs::DspParams& p, uint16_t n_avail);

  // Result retrieval (non-blocking). Each result can be taken once.
  bool takeFull(rs::SegmentResult& out, uint32_t& compute_us);
  bool takePreview(rs::SegmentResult& out, rs::PreviewData& pv);

  uint32_t lastComputeUs() const { return last_us_; }

 private:
  enum State : uint8_t { kIdle = 0, kQueued, kRunning, kDone };
  enum class Kind : uint8_t { kFull, kPreview };

  bool submit(Kind k);
  void execute();
  static void taskEntry(void* arg);

  rs::SegmentBuffer* buf_ = nullptr;
  rs::Workspace*     ws_ = nullptr;

  std::atomic<uint8_t> state_{kIdle};
  Kind            kind_ = Kind::kFull;
  rs::DspParams   params_;
  rs::Seed        seeds_[rs::kMaxSeeds];
  uint8_t         n_seeds_ = 0;
  uint16_t        n_avail_ = 0;
  rs::SegmentResult result_;
  rs::PreviewData   pv_;
  uint32_t        last_us_ = 0;
  void*           sem_ = nullptr;     // SemaphoreHandle_t (device only)
};
