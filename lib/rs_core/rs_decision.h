// RubbleSense core - scan-level decision logic.
//
// A scan is a short sequence of independent, non-overlapping 25.6 s segments.
//   * segment 1 runs the full CFAR search; every non-platform line becomes a *seed*
//   * segment 2 (and 3) re-test the seeds' neighbourhoods (guided confirmation)
//   * CANDIDATE needs a line confirmed in an independent segment AND vetted against
//     platform motion. A single-segment line is only POSSIBLE.
//
// There is deliberately NO "clear" / "no survivor" verdict. The most negative
// outcome is kNoPattern ("no reliable pattern in this scan") and it is downgraded to
// kLowSensitivity when the measured noise floor shows the scan could not have seen a
// weak signal anyway.
#pragma once
#include <stdint.h>
#include "rs_config.h"
#include "rs_dsp.h"

namespace rs {

enum class Verdict : uint8_t {
  kNone = 0,        // scan still running
  kInvalid,         // not enough valid data (sensor motion, link drops, IMU missing)
  kConfounded,      // periodic content that cannot be told apart from platform motion
  kLowSensitivity,  // noise floor far above the bench baseline: a negative would mean nothing
  kNoPattern,       // no reliable pattern in this scan. NEVER a clearance.
  kPossible,        // a line was seen but not confirmed / not vetted
  kCandidate        // line confirmed in an independent segment and vetted
};

const char* verdictName(Verdict v);           // stable ASCII token for logs / JSON
const char* invalidReasonName(InvalidReason r);

struct SegmentSummary {                       // one entry per analysed segment (telemetry)
  bool          valid;
  InvalidReason reason;
  uint8_t       n_det, n_platform, n_untested;
  bool          confirmed;
  bool          cpe_applied;
  bool          vibration;
  float         best_hz, best_margin_db, floor_db, bad_frac, theta_rms_rad;
  uint8_t       best_bin;
};

struct ScanOutcome {
  Verdict       verdict;
  bool          done;
  uint8_t       n_segments, n_valid, n_invalid;
  InvalidReason last_invalid;
  // evidence for kCandidate / kPossible, and for kConfounded when a line coincides with
  // platform motion (hz > 0 then; hz == 0 when only uncancelled platform lines were seen)
  uint8_t       bin;
  float         hz, bpm, snr_db, margin_db;
  uint8_t       agree;                        // segments that show the line
  bool          vetted;                       // platform-coherence test could vet the evidence
  // context for every verdict
  float         floor_db;                     // worst (highest) in-band floor over valid segments
  bool          sens_known;                   // bench baseline available
  float         floor_delta_db;               // floor_db - baseline
  bool          low_sens;                     // sens_known && floor_delta_db > low_sens_db: a weak signal could have been
                                              // missed. Set for every verdict; only a *negative* is downgraded to
                                              // kLowSensitivity (a line that stood out above the raised floor still stands)
  bool          cpe_applied;                  // vibration compensation active in every valid segment
  bool          vibration;                    // a common-phase line was seen in some segment
  float         vibration_hz;
  float         theta_rms_rad;                // worst platform micro-motion (rms common phase) over valid segments
};

class ScanSession {
 public:
  static constexpr int kMaxLog = kMaxSegments + kMaxInvalid;

  void begin(const DspParams& p);

  // Seeds to hand to analyzeSegment() for the next segment.
  const Seed* seeds(uint8_t& n) const { n = n_seeds_; return seeds_; }

  // Feed one analysed full segment. Returns true while another segment is needed.
  bool feed(const SegmentResult& r);

  bool done() const { return out_.done; }
  const ScanOutcome& outcome() const { return out_; }
  uint8_t logCount() const { return n_log_; }
  const SegmentSummary& log(uint8_t i) const { return log_[i]; }

 private:
  void addSeeds(const SegmentResult& r);
  void finalize(Verdict v);
  bool decide();
  int  seedClass(int j) const { return seedClassOf(seed_plat_[j], seed_tested_[j]); }
  static int seedClassOf(bool plat, bool tested) { return plat ? 0 : (tested ? 2 : 1); }
  // Is (class a, margin a) a better lead than (class b, margin b)?
  static bool better(int ca, float ma, int cb, float mb) { return ca != cb ? ca > cb : ma > mb; }

  DspParams p_{};
  ScanOutcome out_{};
  Seed    seeds_[kMaxSeeds];
  bool    seed_tested_[kMaxSeeds];
  bool    seed_plat_[kMaxSeeds];     // line coincides with platform motion
  uint8_t n_seeds_ = 0;
  uint8_t hits_ = 0;               // valid segments that showed a line
  uint8_t n_confounded_ = 0;
  bool    confirmed_ = false;
  float   confirm_hz_ = 0.0f, confirm_seed_hz_ = 0.0f;
  Detection conf_{};
  SegmentSummary log_[kMaxLog];
  uint8_t n_log_ = 0;
};

}  // namespace rs
