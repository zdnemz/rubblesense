#include "dsp_worker.h"
#include <Arduino.h>
#include <string.h>
#include "rs_build.h"

#if !RS_DSP_INLINE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif

void DspWorker::begin(rs::SegmentBuffer* buf, rs::Workspace* ws) {
  buf_ = buf;
  ws_ = ws;
  state_.store(kIdle);
#if !RS_DSP_INLINE
  SemaphoreHandle_t s = xSemaphoreCreateBinary();
  sem_ = s;
  // Core 0 next to the acquisition task, but below it: a segment analysis takes a fraction
  // of a second and must never delay the 100 Hz IMU / radar polling.
  xTaskCreatePinnedToCore(&DspWorker::taskEntry, "rs_dsp", 8192, this, 2, nullptr, 0);
#endif
}

bool DspWorker::submit(Kind k) {
  uint8_t expect = kIdle;
  if (!state_.compare_exchange_strong(expect, kQueued, std::memory_order_acq_rel)) return false;
  kind_ = k;
#if RS_DSP_INLINE
  execute();                                   // inline: deterministic host runs
#else
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(sem_));
#endif
  return true;
}

bool DspWorker::submitFull(const rs::DspParams& p, const rs::Seed* seeds, uint8_t n_seeds) {
  if (busy()) return false;
  params_ = p;
  n_seeds_ = (n_seeds > rs::kMaxSeeds) ? rs::kMaxSeeds : n_seeds;
  for (uint8_t i = 0; i < n_seeds_; i++) seeds_[i] = seeds[i];
  n_avail_ = rs::kSegFrames;
  return submit(Kind::kFull);
}

bool DspWorker::submitPreview(const rs::DspParams& p, uint16_t n_avail) {
  if (busy()) return false;
  params_ = p;
  n_seeds_ = 0;
  n_avail_ = n_avail;
  return submit(Kind::kPreview);
}

void DspWorker::execute() {
  state_.store(kRunning, std::memory_order_release);
  const uint32_t t0 = micros();
  memset(&result_, 0, sizeof(result_));
  if (kind_ == Kind::kFull) {
    pv_.have = false;
    rs::analyzeSegment(*buf_, n_avail_, params_, seeds_, n_seeds_, *ws_, result_, nullptr);
  } else {
    pv_.have = false;
    rs::analyzeSegment(*buf_, n_avail_, params_, nullptr, 0, *ws_, result_, &pv_);
  }
  last_us_ = micros() - t0;
  state_.store(kDone, std::memory_order_release);
}

bool DspWorker::takeFull(rs::SegmentResult& out, uint32_t& compute_us) {
  if (state_.load(std::memory_order_acquire) != kDone || kind_ != Kind::kFull) return false;
  out = result_;
  compute_us = last_us_;
  state_.store(kIdle, std::memory_order_release);
  return true;
}

bool DspWorker::takePreview(rs::SegmentResult& out, rs::PreviewData& pv) {
  if (state_.load(std::memory_order_acquire) != kDone || kind_ != Kind::kPreview) return false;
  out = result_;
  pv = pv_;
  state_.store(kIdle, std::memory_order_release);
  return true;
}

void DspWorker::taskEntry(void* arg) {
#if !RS_DSP_INLINE
  DspWorker* self = static_cast<DspWorker*>(arg);
  for (;;) {
    if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(self->sem_), portMAX_DELAY) == pdTRUE) self->execute();
  }
#else
  (void)arg;
#endif
}
