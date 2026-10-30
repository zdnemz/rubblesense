#include "scan_json.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include "rs_build.h"
#include "rs_text.h"

namespace {

// Bounded appender. Once an append does not fit, `ok` stays false and nothing more is written.
struct Out {
  char*  p;
  size_t cap;
  size_t n;
  bool   ok;
  Out(char* buf, size_t c) : p(buf), cap(c), n(0), ok(c > 0) { if (ok) p[0] = '\0'; }
  void fmt(const char* f, ...) __attribute__((format(printf, 2, 3)));
};

void Out::fmt(const char* f, ...) {
  if (!ok) return;
  va_list ap;
  va_start(ap, f);
  const int w = vsnprintf(p + n, cap - n, f, ap);
  va_end(ap);
  if (w < 0 || static_cast<size_t>(w) >= cap - n) { ok = false; return; }
  n += static_cast<size_t>(w);
}

// JSON cannot carry NaN / inf.
inline double fin(float x) { return isfinite(x) ? static_cast<double>(x) : 0.0; }

// A difference that would print as "-0.0" at one decimal is written as 0.0.
inline double fin1(float x) {
  const double v = fin(x);
  return (v > -0.05 && v < 0.05) ? 0.0 : v;
}

const char* imuName(ImuSource s) {
  switch (s) {
    case ImuSource::kHardware:  return "hw";
    case ImuSource::kSimulated: return "sim";
    case ImuSource::kHost:      return "host";
  }
  return "?";
}

// Text that goes into a JSON string must not need escaping: all of ours is ASCII without quotes.
const char* safeStr(const char* s) { return s ? s : ""; }

}  // namespace

float platformMotionMm(float theta_rms_rad, float wavelength_mm) {
  return theta_rms_rad * wavelength_mm / (4.0f * 3.14159265f);
}

const char* evidenceName(const rs::ScanOutcome& o) {
  switch (o.verdict) {
    case rs::Verdict::kCandidate:
      if (o.vetted && o.margin_db >= 8.0f) return "strong";
      if (o.margin_db >= 3.0f) return "moderate";
      return "weak";
    case rs::Verdict::kPossible:
      return "weak";
    default:
      return "none";
  }
}

const char* sensitivityName(const rs::ScanOutcome& o) {
  if (!o.sens_known) return "unknown";
  return o.low_sens ? "low" : "ok";
}

size_t buildScanJson(char* buf, size_t cap, const ScanRecord& r) {
  Out w(buf, cap);
  const rs::ScanOutcome& o = r.out;
  const bool has_line = (o.verdict == rs::Verdict::kCandidate || o.verdict == rs::Verdict::kPossible ||
                         (o.verdict == rs::Verdict::kConfounded && o.hz > 0.0f));
  const float range_m = r.info.range_start_m + static_cast<float>(o.bin) * r.info.range_step_m;

  w.fmt("{\"v\":1,\"fw\":\"%s\",\"scan_id\":\"RS-%04lu\",\"uptime_s\":%lu,\"duration_s\":%.1f,",
        RS_FW_VERSION, static_cast<unsigned long>(r.scan_id), static_cast<unsigned long>(r.uptime_s), fin(r.duration_s));
  w.fmt("\"source\":\"%s\",\"preset\":\"%s\",\"imu\":\"%s\",", safeStr(r.source), safeStr(r.preset), imuName(r.imu));
  w.fmt("\"verdict\":\"%s\",\"evidence\":\"%s\",", rs::verdictName(o.verdict), evidenceName(o));

  // Human-readable result in English. A negative scan always carries the mandatory sentence.
  if (o.verdict == rs::Verdict::kNoPattern) {
    w.fmt("\"result\":\"%s\",", txt::mandatoryNegativeEn());
  } else {
    w.fmt("\"result\":\"%s\",", txt::verdictHeadlineEn(o.verdict));
  }
  w.fmt("\"operator_action\":\"%s\",", txt::verdictActionEn(o.verdict, o.last_invalid));
  w.fmt("\"invalid_reason\":\"%s\",", rs::invalidReasonName(o.last_invalid));

  if (has_line) {
    w.fmt("\"bin\":%u,\"range_m\":%.2f,\"hz\":%.3f,\"bpm\":%.1f,\"snr_db\":%.1f,\"margin_db\":%.1f,"
          "\"segments_agree\":%u,\"vetted\":%s,",
          static_cast<unsigned>(o.bin), fin(range_m), fin(o.hz), fin(o.bpm), fin(o.snr_db), fin(o.margin_db),
          static_cast<unsigned>(o.agree), o.vetted ? "true" : "false");
  }

  w.fmt("\"segments_total\":%u,\"segments_valid\":%u,\"segments_invalid\":%u,",
        static_cast<unsigned>(o.n_segments), static_cast<unsigned>(o.n_valid), static_cast<unsigned>(o.n_invalid));
  w.fmt("\"floor_db\":%.1f,\"sensitivity\":\"%s\",\"baseline_src\":\"%s\",", fin(o.floor_db), sensitivityName(o),
        safeStr(r.baseline_src));
  if (o.sens_known) w.fmt("\"baseline_db\":%.1f,\"floor_delta_db\":%.1f,", fin(r.params.baseline_floor_db), fin1(o.floor_delta_db));
  w.fmt("\"cpe_applied\":%s,\"vibration\":%s,\"vibration_hz\":%.2f,\"platform_motion_mm_rms\":%.3f,",
        o.cpe_applied ? "true" : "false", o.vibration ? "true" : "false", fin(o.vibration_hz),
        fin(platformMotionMm(o.theta_rms_rad, r.info.wavelength_mm)));

  w.fmt("\"radar\":{\"name\":\"%s\",\"fs_hz\":%.2f,\"range_start_m\":%.3f,\"range_step_m\":%.3f,\"wavelength_mm\":%.1f,\"bins\":%u},",
        safeStr(r.info.name), fin(r.info.frame_rate_hz), fin(r.info.range_start_m), fin(r.info.range_step_m),
        fin(r.info.wavelength_mm), static_cast<unsigned>(r.info.n_bins));
  w.fmt("\"link\":{\"lost_frames\":%lu,\"crc_errors\":%lu,\"timing_flagged\":%lu,\"timestamps\":%s},",
        static_cast<unsigned long>(r.lost_frames), static_cast<unsigned long>(r.crc_errors),
        static_cast<unsigned long>(r.timing_flagged), r.timing_checked ? "true" : "false");

  w.fmt("\"segments\":[");
  for (uint8_t i = 0; i < r.n_seg; i++) {
    const rs::SegmentSummary& s = r.seg[i];
    w.fmt("%s{\"valid\":%s,\"reason\":\"%s\",\"det\":%u,\"platform\":%u,\"untested\":%u,\"confirmed\":%s,"
          "\"best_bin\":%u,\"best_hz\":%.3f,\"best_margin_db\":%.1f,\"floor_db\":%.1f,\"bad_frac\":%.3f,"
          "\"cpe\":%s,\"vib\":%s}",
          i ? "," : "", s.valid ? "true" : "false", rs::invalidReasonName(s.reason), static_cast<unsigned>(s.n_det),
          static_cast<unsigned>(s.n_platform), static_cast<unsigned>(s.n_untested), s.confirmed ? "true" : "false",
          static_cast<unsigned>(s.best_bin), fin(s.best_hz), fin(s.best_margin_db), fin(s.floor_db), fin(s.bad_frac),
          s.cpe_applied ? "true" : "false", s.vibration ? "true" : "false");
  }
  w.fmt("],");

  const rs::DspParams& p = r.params;
  w.fmt("\"params\":{\"pfa\":%.4f,\"pfc\":%.4f,\"band_hz\":[%.2f,%.2f],\"min_bin\":%u,\"cpe\":%s,\"guard\":%u,\"low_sens_db\":%.1f,"
        "\"require_imu\":%s},",
        fin(p.pfa_segment), fin(p.pfa_confirm), fin(p.resp_lo_hz), fin(p.resp_hi_hz), static_cast<unsigned>(p.min_bin),
        p.cpe_enable ? "true" : "false", static_cast<unsigned>(p.cpe_guard_bins), fin(p.low_sens_db),
        p.require_imu ? "true" : "false");
  w.fmt("\"safety_notice\":\"Aid only. Never infer absence of survivor from a negative scan.\"}");

  return w.ok ? w.n : 0;
}
