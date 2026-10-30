// RubbleSense firmware - one scan as one line of JSON (serial output + LittleFS log).
//
// The line is built with snprintf into a caller buffer; no heap, no JSON library. The record
// always carries the safety notice and, for a negative verdict, the mandatory sentence.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "rs_config.h"
#include "rs_decision.h"
#include "rs_types.h"

constexpr size_t kScanJsonMax = 3072;      // buffer size that always suffices for kMaxLog segments (host-tested)

enum class ImuSource : uint8_t { kHardware = 0, kSimulated, kHost };

struct ScanRecord {
  uint32_t    scan_id = 0;
  uint32_t    uptime_s = 0;
  float       duration_s = 0.0f;
  const char* source = "";               // "SIM" / "SERIAL"
  const char* preset = "";               // trial preset id for the simulator, "" otherwise
  ImuSource   imu = ImuSource::kHardware;
  rs::RadarInfo info{};
  rs::DspParams params;
  rs::ScanOutcome out{};
  uint8_t     n_seg = 0;
  rs::SegmentSummary seg[rs::ScanSession::kMaxLog];
  const char* baseline_src = "none";     // "none" | "calibrated" | "sim_default"
  uint32_t    lost_frames = 0;           // link statistics of the scan
  uint32_t    crc_errors = 0;
  uint32_t    timing_flagged = 0;        // frames with inconsistent front-end timestamps
  bool        timing_checked = false;    // the front-end sends timestamps (otherwise the check could not run)
};

// Ordinal strength of the evidence: "strong" | "moderate" | "weak" | "none". Deliberately not a
// percentage: no calibrated probability exists without real recordings.
const char* evidenceName(const rs::ScanOutcome& o);

// Sensitivity statement: "unknown" (no baseline) | "ok" | "low". "low" whenever the noise floor is above the
// calibrated level by more than the limit, whatever the verdict is (a negative then becomes LOW_SENSITIVITY).
const char* sensitivityName(const rs::ScanOutcome& o);

// Writes the JSON line (no trailing newline). Returns its length, or 0 if `cap` is too small.
size_t buildScanJson(char* buf, size_t cap, const ScanRecord& r);

// Platform micro-motion in mm rms from the rms common phase (x = theta * lambda / 4 pi).
float platformMotionMm(float theta_rms_rad, float wavelength_mm);
