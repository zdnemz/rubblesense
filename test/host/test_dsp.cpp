// Segment-level DSP tests: detection, false-alarm rate, platform vibration, confirmation,
// data-quality failures. All on the simulated scene (see rs_simgen.h for what that does NOT model).
#include <cmath>
#include <string>
#include <cstring>
#include <initializer_list>
#include "rs_test.h"
#include "sim_helpers.h"

using namespace rs;

namespace {

struct Tally {
  int n = 0, valid = 0, det = 0, bin_ok = 0, freq_ok = 0, plat = 0, plat_peak = 0, conf = 0, vetted = 0;
  double floor_sum = 0.0;
};

// Run `trials` independent segments of one scene and tally what the DSP reports.
Tally runSegments(const SimScene& sc, int trials, uint32_t seed0, const DspParams& p,
                  const Seed* seeds = nullptr, uint8_t n_seeds = 0, float exp_hz = 0.0f) {
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  Tally t;
  for (int i = 0; i < trials; i++) {
    SimRun run;
    run.begin(sc, seed0 + static_cast<uint32_t>(i));
    buf.reset(sc.n_bins);
    run.fill(buf);
    SegmentResult r;
    analyzeSegment(buf, kSegFrames, p, seeds, n_seeds, ws, r);
    t.n++;
    if (!r.valid) continue;
    t.valid++;
    t.floor_sum += static_cast<double>(r.floor_db);
    if (r.n_det > 0) {
      t.det++;
      if (r.det[0].tested) t.vetted++;
      if (std::abs(static_cast<int>(r.det[0].bin) - static_cast<int>(sc.target_bin)) <= 1) t.bin_ok++;
      if (exp_hz > 0.0f && std::fabs(r.det[0].hz - exp_hz) < 0.03f) t.freq_ok++;
    }
    if (r.n_platform > 0) t.plat++;
    if (r.platform_peak) t.plat_peak++;
    if (r.confirmed) t.conf++;
  }
  return t;
}

SimScene person(float amp = 30.0f) {
  SimScene s;
  s.target = true;
  s.target_amp = amp;
  return s;
}

}  // namespace

static void test_dead_and_flat_input() {
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  DspParams p;
  SimScene sc;
  buf.reset(sc.n_bins);

  // All zeros: a disconnected or powered-off front-end must not read as "nothing there".
  RadarFrame f{};
  f.n_bins = sc.n_bins;
  for (int i = 0; i < kSegFrames; i++) buf.push(f, 0);
  SegmentResult r;
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kNoSignal);

  // Frozen output: constant non-zero values (stuck ADC / repeated frame).
  buf.reset(sc.n_bins);
  for (int b = 0; b < sc.n_bins; b++) { f.iq[b].i = static_cast<int16_t>(1000 + 37 * b); f.iq[b].q = static_cast<int16_t>(-500 + 11 * b); }
  for (int i = 0; i < kSegFrames; i++) buf.push(f, 0);
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kNoSignal);

  // The same scene with its normal noise is fine.
  SimRun run; run.begin(sc, 5);
  buf.reset(sc.n_bins);
  run.fill(buf);
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(r.valid);
}

static void test_config_and_length() {
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  SimScene sc;
  SimRun run; run.begin(sc, 9);
  buf.reset(sc.n_bins);
  run.fill(buf);
  SegmentResult r;

  DspParams p;
  p.min_bin = 20;                                              // beyond the delivered bins
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kConfig);
  p = DspParams(); p.cfar_ref = 2;
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kConfig);

  // A band that does not fit the spectral window is refused, never silently truncated.
  p = DspParams(); p.fs_hz = 6.0f;                             // df = 6/512 Hz: 0.1-0.8 Hz needs > 64 cells
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kConfig);
  p = DspParams(); p.resp_lo_hz = 0.40f; p.resp_hi_hz = 0.41f; // contains no FFT cell at all
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kConfig);

  // A half-length window is analysed with its own (coarser) resolution.
  p = DspParams();
  analyzeSegment(buf, kPrevFrames, p, nullptr, 0, ws, r);
  CHECK(r.valid && r.n_used == kPrevFrames);
  analyzeSegment(buf, 200, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kTooShort);
}

// What the operator has to fix depends on why frames were unusable: a firmer grip cures motion,
// less gain cures clipping, a cable cures lost frames.
static void test_quality_gate_reasons() {
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  SimScene sc;
  SimRun run; run.begin(sc, 21);
  DspParams p;
  SegmentResult r;
  const uint8_t kMot = kFlagImuMotion, kSat = kFlagSaturated, kDrop = kFlagDropped, kMiss = kFlagImuMissing,
                kTim = kFlagTiming;

  struct Case { uint8_t flag_a; int n_a; uint8_t flag_b; int n_b; bool valid; InvalidReason why; const char* what; };
  const Case cases[] = {
      {kSat, 512, 0, 0, false, InvalidReason::kSaturated, "all frames clipping"},
      {kMot, 512, 0, 0, false, InvalidReason::kMotion, "all frames flagged motion"},
      {kSat, 30, kMot, 5, false, InvalidReason::kSaturated, "mostly clipping, some motion"},
      {kMot, 30, kSat, 5, false, InvalidReason::kMotion, "mostly motion, some clipping"},
      {kSat, 20, 0, 0, true, InvalidReason::kNone, "4 % clipped frames are tolerated"},
      {kDrop, 30, 0, 0, false, InvalidReason::kDropout, "6 % lost frames"},
      {kMiss, 300, 0, 0, false, InvalidReason::kImuMissing, "no IMU data for most of the segment"},
      {kTim, 512, 0, 0, false, InvalidReason::kTiming, "timestamps inconsistent for the whole segment"},
      {kTim, 30, kMot, 5, false, InvalidReason::kTiming, "mostly bad timing, some motion"},
      {kMot, 30, kTim, 5, false, InvalidReason::kMotion, "mostly motion, some bad timing"},
      {kTim, 20, 0, 0, true, InvalidReason::kNone, "4 % frames with bad timing are tolerated"},
      {kTim, 15, kMot, 15, false, InvalidReason::kMotion, "bad timing and motion add up; motion wins a tie"},
  };
  for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
    buf.reset(sc.n_bins);
    run.fill(buf);
    for (int i = 0; i < cases[c].n_a; i++) buf.flags[i] = cases[c].flag_a;
    for (int i = 0; i < cases[c].n_b; i++) buf.flags[200 + i] = cases[c].flag_b;
    analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
    if (r.valid != cases[c].valid || r.reason != cases[c].why) std::printf("  case '%s': valid=%d reason=%s\n", cases[c].what, r.valid, invalidReasonName(r.reason));
    CHECK(r.valid == cases[c].valid);
    CHECK(r.reason == cases[c].why);
  }
  CHECK(std::string(invalidReasonName(InvalidReason::kSaturated)) == "adc_saturated");
  CHECK(std::string(invalidReasonName(InvalidReason::kTiming)) == "radar_timing");
  for (int i = 0; i < static_cast<int>(InvalidReason::kCount); i++)             // every reason has a distinct, real name
    CHECK(std::string(invalidReasonName(static_cast<InvalidReason>(i))) != "?");
}

static void test_floor_calibration() {
  // floor_db is in counts^2 per complex sample: 10*log10(2*sigma^2).
  DspParams p;
  for (float sigma : {4.0f, 8.0f, 16.0f}) {
    SimScene sc; sc.noise_sigma = sigma;
    const Tally t = runSegments(sc, 60, 11, p);
    const double want = 10.0 * std::log10(2.0 * static_cast<double>(sigma) * static_cast<double>(sigma));
    CHECK(t.valid == 60);
    CHECK_NEAR(t.floor_sum / t.valid, want, 0.6);
  }
}

static void test_detection_and_false_alarms() {
  DspParams p;
  // Strong person: found in the right bin at the right rate, every time.
  {
    const Tally t = runSegments(person(100.0f), 60, 100, p, nullptr, 0, 0.28f);
    CHECK(t.det == 60 && t.bin_ok == 60);
    CHECK(t.freq_ok >= 58);                                    // breath-rate wander is part of the model
  }
  // Empty scene: segment-level false-alarm rate near the 1 % design value.
  {
    SimScene sc;
    const Tally t = runSegments(sc, 3000, 2000, p);
    CHECK(t.valid == 3000);
    CHECK(t.det <= 48);                                        // <= 1.6 %
  }
  // Band: 0.12 and 0.72 Hz are inside, 1.0 Hz is outside.
  {
    SimScene lo = person(100.0f); lo.breath_hz = 0.12f;
    const Tally a = runSegments(lo, 40, 300, p, nullptr, 0, 0.12f);
    CHECK(a.det >= 38 && a.freq_ok >= 36);
    SimScene hi = person(100.0f); hi.breath_hz = 0.72f;
    const Tally b = runSegments(hi, 40, 400, p, nullptr, 0, 0.72f);
    CHECK(b.det >= 38 && b.freq_ok >= 36);
    SimScene out = person(100.0f); out.breath_hz = 1.0f; out.heart_mm = 0.0f;
    const Tally c = runSegments(out, 200, 500, p);
    CHECK(c.det <= 6);
  }
  // Other bin counts (fewer / more range bins) still work.
  {
    SimScene a = person(100.0f); a.n_bins = 12;
    CHECK(runSegments(a, 20, 600, p).det == 20);
    SimScene b = person(100.0f); b.n_bins = kMaxBins;
    CHECK(runSegments(b, 20, 700, p).det == 20);
  }
}

static void test_platform_vibration() {
  DspParams p;
  // Sub-millimetre vibration (below what an MPU6050 resolves), nobody there: never a person.
  {
    SimScene sc; sc.vib_hz = 0.30f; sc.vib_mm = 0.2f;
    const Tally t = runSegments(sc, 200, 3000, p);
    CHECK(t.valid == 200);
    CHECK(t.det <= 4);                                         // <= 2 %
    CHECK(t.plat_peak >= 190);                                 // and it is recognised as vibration
  }
  // Same vibration with the compensation switched off: the coherence test alone must still keep
  // it from being reported as a person.
  {
    DspParams q = p; q.cpe_enable = false;
    SimScene sc; sc.vib_hz = 0.30f; sc.vib_mm = 0.2f;
    const Tally t = runSegments(sc, 200, 3500, q);
    CHECK(t.det <= 6);
  }
  // Person breathing while the platform vibrates at another rate: person still found.
  {
    SimScene sc = person(30.0f); sc.vib_hz = 0.45f; sc.vib_mm = 0.2f;
    const Tally t = runSegments(sc, 100, 4000, p, nullptr, 0, 0.28f);
    CHECK(t.det >= 80);
    CHECK(t.bin_ok >= 78);
    CHECK(t.freq_ok >= 76);
  }
  // A strong vibration line close to (but not at) the breathing rate leaks into the common phase
  // spectrum; that leakage must not be mistaken for the person's own line.
  {
    SimScene sc = person(30.0f); sc.vib_hz = 0.15f; sc.vib_mm = 0.2f;
    const Tally t = runSegments(sc, 100, 4200, p, nullptr, 0, 0.28f);
    CHECK(t.det >= 97 && t.bin_ok >= 95 && t.freq_ok >= 94);
    CHECK(t.vetted >= 95);                                     // and the line counts as vetted
  }
  // Person and vibration at the SAME rate: the line is kept (never silently dropped) but it is
  // reported as not vetted, because platform motion cannot be excluded as its cause.
  {
    SimScene sc = person(30.0f); sc.vib_hz = 0.28f; sc.vib_mm = 0.2f;
    const Tally t = runSegments(sc, 100, 4500, p, nullptr, 0, 0.28f);
    CHECK(t.det >= 95);
    CHECK(t.vetted <= 10);
  }
  // Non-common vibration (each bin sees a different amount of it): never a vetted candidate line.
  {
    SimScene sc; sc.vib_hz = 0.30f; sc.vib_mm = 0.4f; sc.cpe_mismatch = 0.3f;
    const Tally t = runSegments(sc, 200, 4800, p);
    CHECK(t.vetted <= 8);                                      // <= 4 % vetted lines
  }
}

static void test_platform_motion_estimate() {
  // theta_rms is the rms common phase: for a sinusoidal vibration of x mm it is k*x/sqrt(2).
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  DspParams p;
  for (float mm : {0.1f, 0.3f}) {
    SimScene sc; sc.vib_hz = 0.40f; sc.vib_mm = mm;
    SimRun run; run.begin(sc, 31);
    buf.reset(sc.n_bins); run.fill(buf);
    SegmentResult r;
    analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
    const double k = 4.0 * 3.14159265358979 / static_cast<double>(sc.wavelength_mm);
    CHECK_REL(static_cast<double>(r.theta_rms_rad), k * static_cast<double>(mm) / std::sqrt(2.0), 0.15);
  }
  SimScene quiet;
  SimRun run; run.begin(quiet, 32);
  buf.reset(quiet.n_bins); run.fill(buf);
  SegmentResult r;
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(r.theta_rms_rad < 0.005f);                              // thermal-noise floor of the estimate
}

static void test_guided_confirmation() {
  DspParams p;
  // Empty scene, a seed at a random place: confirmation false-alarm rate stays small.
  {
    SegmentBuffer& buf = sharedBuffer();
    Workspace& ws = sharedWorkspace();
    SimScene sc;
    Rng rng(77);
    int conf = 0;
    const int trials = 3000;
    for (int i = 0; i < trials; i++) {
      SimRun run; run.begin(sc, 8000u + static_cast<uint32_t>(i));
      buf.reset(sc.n_bins);
      run.fill(buf);
      Seed s;
      s.bin = static_cast<uint8_t>(3 + rng.next() % 11);
      s.hz = 0.15f + 0.55f * rng.uniform();
      s.margin_db = 3.0f;
      SegmentResult r;
      analyzeSegment(buf, kSegFrames, p, &s, 1, ws, r);
      if (r.confirmed) conf++;
    }
    CHECK(conf <= 90);                                         // <= 3 % against a 2 % design value
  }
  // A weak person that the full search can miss is confirmed when the seed is right ...
  {
    Seed s{9, 0.28f, 3.0f};
    const Tally t = runSegments(person(24.0f), 100, 9000, p, &s, 1);
    CHECK(t.conf >= 80);
    // ... and not when the seed points three bins away.
    Seed wrong{13, 0.28f, 3.0f};
    const Tally w = runSegments(person(24.0f), 100, 9000, p, &wrong, 1);
    CHECK(w.conf <= 4);
  }
}

static void test_preview_data() {
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  DspParams p;
  SimScene sc = person(100.0f);
  SimRun run; run.begin(sc, 21);
  buf.reset(sc.n_bins);
  run.fill(buf);
  static PreviewData pv;
  SegmentResult r;
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r, &pv);
  CHECK(pv.have && std::abs(static_cast<int>(pv.best_bin) - 9) <= 1);
  CHECK(pv.n_cells > 20 && pv.n_cells <= kMaxSpecCells);
  CHECK(pv.wave_n == kPrevFrames);
  float mx = 0.0f;
  bool finite = true;
  for (int i = 0; i < pv.wave_n; i++) { mx = std::fmax(mx, std::fabs(pv.wave[i])); finite = finite && std::isfinite(pv.wave[i]); }
  CHECK_NEAR(mx, 1.0, 1e-5);
  CHECK(finite);
  // The preview spectrum peaks at the breathing line.
  int kpk = 0;
  for (int c = 1; c < pv.n_cells; c++) if (pv.spec_db[c] > pv.spec_db[kpk]) kpk = c;
  CHECK_NEAR((pv.k0 + kpk) * pv.df_hz, 0.28, 0.05);
}

static void test_extreme_inputs() {
  // Full-scale random data (a jammed or saturated front-end) must not crash or produce NaN.
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  DspParams p;
  Rng rng(5);
  RadarFrame f{};
  f.n_bins = 16;
  buf.reset(16);
  for (int i = 0; i < kSegFrames; i++) {
    for (int b = 0; b < 16; b++) {
      f.iq[b].i = static_cast<int16_t>(static_cast<int>(rng.next() & 0xFFFF) - 32768);
      f.iq[b].q = static_cast<int16_t>(static_cast<int>(rng.next() & 0xFFFF) - 32768);
    }
    buf.push(f, 0);
  }
  SegmentResult r;
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(r.valid);
  CHECK(std::isfinite(r.floor_db) && r.floor_db > 60.0f);      // enormous floor -> would read as LOW_SENSITIVITY
  CHECK(r.n_det <= kMaxDet);
}

void test_dsp() {
  RUN(test_dead_and_flat_input);
  RUN(test_config_and_length);
  RUN(test_quality_gate_reasons);
  RUN(test_floor_calibration);
  RUN(test_detection_and_false_alarms);
  RUN(test_platform_vibration);
  RUN(test_platform_motion_estimate);
  RUN(test_guided_confirmation);
  RUN(test_preview_data);
  RUN(test_extreme_inputs);
}
