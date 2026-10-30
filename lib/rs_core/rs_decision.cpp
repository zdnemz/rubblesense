#include "rs_decision.h"
#include <math.h>
#include <string.h>

namespace rs {

const char* verdictName(Verdict v) {
  switch (v) {
    case Verdict::kNone:           return "SCANNING";
    case Verdict::kInvalid:        return "INVALID_SCAN";
    case Verdict::kConfounded:     return "CONFOUNDED";
    case Verdict::kLowSensitivity: return "LOW_SENSITIVITY";
    case Verdict::kNoPattern:      return "NO_RELIABLE_PATTERN";
    case Verdict::kPossible:       return "POSSIBLE_PATTERN";
    case Verdict::kCandidate:      return "CANDIDATE_PATTERN";
  }
  return "?";
}

const char* invalidReasonName(InvalidReason r) {
  switch (r) {
    case InvalidReason::kNone:       return "none";
    case InvalidReason::kTooShort:   return "too_short";
    case InvalidReason::kMotion:     return "sensor_motion";
    case InvalidReason::kDropout:    return "radar_dropout";
    case InvalidReason::kImuMissing: return "imu_missing";
    case InvalidReason::kConfig:     return "config";
    case InvalidReason::kNoSignal:   return "no_signal";
    case InvalidReason::kSaturated:  return "adc_saturated";
    case InvalidReason::kTiming:     return "radar_timing";
    case InvalidReason::kCount:      break;
  }
  return "?";
}

void ScanSession::begin(const DspParams& p) {
  p_ = p;
  memset(&out_, 0, sizeof(out_));
  out_.verdict = Verdict::kNone;
  n_seeds_ = 0; hits_ = 0; n_confounded_ = 0; confirmed_ = false; n_log_ = 0;
  for (int j = 0; j < kMaxSeeds; j++) { seed_tested_[j] = false; seed_plat_[j] = false; }
  confirm_hz_ = confirm_seed_hz_ = 0.0f;
  conf_ = Detection{};
}

// Add the segment's lines as seeds for the next segment. Lines the platform test could vet
// outrank unvetted ones, platform-coincident lines rank last (de-duplicated, best kept).
void ScanSession::addSeeds(const SegmentResult& r) {
  for (int i = 0; i < r.n_det; i++) {
    const Detection& d = r.det[i];
    const int dcls = seedClassOf(d.platform_like, d.tested);
    int dup = -1;
    for (int j = 0; j < n_seeds_; j++) {
      if (fabsf(seeds_[j].hz - d.hz) <= p_.freq_tol_hz &&
          abs(static_cast<int>(seeds_[j].bin) - static_cast<int>(d.bin)) <= p_.bin_tol) { dup = j; break; }
    }
    int slot = -1;
    if (dup >= 0) {
      if (better(dcls, d.margin_db, seedClass(dup), seeds_[dup].margin_db)) slot = dup;
    } else if (n_seeds_ < kMaxSeeds) {
      slot = n_seeds_++;
    } else {
      int weakest = 0;
      for (int j = 1; j < n_seeds_; j++)
        if (better(seedClass(weakest), seeds_[weakest].margin_db, seedClass(j), seeds_[j].margin_db)) weakest = j;
      if (better(dcls, d.margin_db, seedClass(weakest), seeds_[weakest].margin_db)) slot = weakest;
    }
    if (slot >= 0) {
      seeds_[slot] = Seed{d.bin, d.hz, d.margin_db};
      seed_tested_[slot] = d.tested;
      seed_plat_[slot] = d.platform_like;
    }
  }
}

bool ScanSession::feed(const SegmentResult& r) {
  if (out_.done) return false;
  out_.n_segments++;

  SegmentSummary sm{};
  sm.valid = r.valid; sm.reason = r.reason; sm.n_det = r.n_det; sm.n_platform = r.n_platform;
  sm.n_untested = r.n_untested; sm.confirmed = r.confirmed; sm.cpe_applied = r.cpe_applied;
  sm.vibration = r.platform_peak; sm.floor_db = r.floor_db; sm.bad_frac = r.bad_frac; sm.theta_rms_rad = r.theta_rms_rad;
  if (r.n_det > 0) { sm.best_hz = r.det[0].hz; sm.best_margin_db = r.det[0].margin_db; sm.best_bin = r.det[0].bin; }
  if (n_log_ < kMaxLog) log_[n_log_++] = sm;

  if (!r.valid) {
    out_.n_invalid++;
    out_.last_invalid = r.reason;
    return decide();
  }

  out_.n_valid++;
  if (r.floor_db > out_.floor_db || out_.n_valid == 1) out_.floor_db = r.floor_db;
  if (out_.n_valid == 1) out_.cpe_applied = r.cpe_applied; else out_.cpe_applied = out_.cpe_applied && r.cpe_applied;
  if (r.platform_peak) { out_.vibration = true; out_.vibration_hz = r.platform_hz; }
  if (r.theta_rms_rad > out_.theta_rms_rad) out_.theta_rms_rad = r.theta_rms_rad;
  if (r.n_platform > 0 && !r.cpe_applied) n_confounded_++;
  if (r.n_det > 0 || r.confirmed) hits_++;

  if (r.confirmed && !confirmed_) {
    confirmed_ = true;
    conf_ = r.conf;
    confirm_hz_ = r.conf.hz;
    confirm_seed_hz_ = (r.conf_seed < n_seeds_) ? seeds_[r.conf_seed].hz : r.conf.hz;
  }
  addSeeds(r);
  return decide();
}

bool ScanSession::decide() {
  const bool budget_left = (out_.n_invalid < kMaxInvalid) && (out_.n_valid < kMaxSegments);
  if (confirmed_) { finalize(Verdict::kCandidate); return false; }
  if (out_.n_valid >= 2) {
    if (n_seeds_ > 0 && budget_left) return true;      // unconfirmed line: take one more segment
    finalize(Verdict::kNoPattern);                     // finalize() picks the real verdict
    return false;
  }
  if (out_.n_invalid >= kMaxInvalid) { finalize(Verdict::kInvalid); return false; }
  return true;                                         // need a second valid segment
}

void ScanSession::finalize(Verdict hint) {
  out_.done = true;
  out_.sens_known = p_.have_baseline;
  out_.floor_delta_db = p_.have_baseline ? out_.floor_db - p_.baseline_floor_db : 0.0f;
  out_.low_sens = p_.have_baseline && out_.floor_delta_db > p_.low_sens_db;

  if (hint == Verdict::kCandidate) {
    out_.verdict = Verdict::kCandidate;
    out_.bin = conf_.bin;
    out_.hz = 0.5f * (confirm_hz_ + confirm_seed_hz_);
    out_.bpm = out_.hz * 60.0f;
    out_.snr_db = conf_.snr_db;
    out_.margin_db = conf_.margin_db;
    out_.agree = hits_ < 2 ? 2 : hits_;
    out_.vetted = true;
    return;
  }
  if (n_seeds_ > 0) {                                  // a line exists but could not be confirmed
    int best = 0;
    for (int j = 1; j < n_seeds_; j++)
      if (better(seedClass(j), seeds_[j].margin_db, seedClass(best), seeds_[best].margin_db)) best = j;
    out_.bin = seeds_[best].bin;
    out_.hz = seeds_[best].hz;
    out_.bpm = out_.hz * 60.0f;
    out_.margin_db = seeds_[best].margin_db;
    out_.agree = hits_;
    out_.vetted = seed_tested_[best];
    // Only lines that coincide with platform motion: neither a lead nor a negative.
    out_.verdict = seed_plat_[best] ? Verdict::kConfounded : Verdict::kPossible;
    return;
  }
  if (n_confounded_ > 0) { out_.verdict = Verdict::kConfounded; return; }
  if (out_.n_valid < 2)  { out_.verdict = Verdict::kInvalid;    return; }
  if (out_.low_sens) {
    out_.verdict = Verdict::kLowSensitivity;
    return;
  }
  out_.verdict = Verdict::kNoPattern;
}

}  // namespace rs
