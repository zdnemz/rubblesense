// Scan-level decision logic: table-driven state machine tests plus end-to-end
// simulator scenarios (the same presets the Wokwi build offers).
#include <cstring>
#include "rs_test.h"
#include "sim_helpers.h"

using namespace rs;

namespace {

SegmentResult validSeg(float floor_db = 21.0f) {
  SegmentResult r;
  memset(&r, 0, sizeof(r));
  r.valid = true;
  r.floor_db = floor_db;
  r.cpe_applied = true;
  return r;
}
SegmentResult invalidSeg(InvalidReason why = InvalidReason::kMotion) {
  SegmentResult r;
  memset(&r, 0, sizeof(r));
  r.valid = false;
  r.reason = why;
  return r;
}
void addDet(SegmentResult& r, uint8_t bin, float hz, float margin = 6.0f, bool tested = true, bool plat = false) {
  Detection d{};
  d.bin = bin; d.hz = hz; d.k = static_cast<uint16_t>(hz / 0.039f);
  d.snr_db = margin + 9.0f; d.margin_db = margin; d.tested = tested && !plat; d.platform_like = plat;
  r.det[r.n_det++] = d;
  if (!d.tested) r.n_untested++;
}
void confirm(SegmentResult& r, uint8_t bin, float hz) {
  r.confirmed = true;
  r.conf = Detection{bin, 7, hz, 12.0f, 4.0f, true, false};
  r.conf_seed = 0;
}

}  // namespace

void test_state_machine() {
  DspParams p;
  ScanSession s;

  // Two valid empty segments -> no reliable pattern (never a "clear").
  s.begin(p);
  CHECK(s.feed(validSeg()) == true);
  CHECK(s.feed(validSeg()) == false);
  CHECK(s.outcome().done && s.outcome().verdict == Verdict::kNoPattern);

  // Line in segment 1, confirmed in segment 2 -> candidate.
  s.begin(p);
  SegmentResult a = validSeg();
  addDet(a, 9, 0.28f);
  CHECK(s.feed(a) == true);
  uint8_t ns = 0;
  const Seed* sd = s.seeds(ns);
  CHECK(ns == 1 && sd[0].bin == 9);
  SegmentResult b = validSeg();
  confirm(b, 9, 0.29f);
  CHECK(s.feed(b) == false);
  CHECK(s.outcome().verdict == Verdict::kCandidate);
  CHECK_NEAR(s.outcome().bpm, 60.0f * 0.285f, 0.05);
  CHECK(s.outcome().vetted);

  // Line in segment 1 only: third segment requested, still unconfirmed -> possible.
  s.begin(p);
  a = validSeg(); addDet(a, 9, 0.28f);
  CHECK(s.feed(a) == true);
  CHECK(s.feed(validSeg()) == true);          // unconfirmed line -> take one more segment
  CHECK(s.feed(validSeg()) == false);
  CHECK(s.outcome().verdict == Verdict::kPossible);

  // ... but confirmed in the third segment -> candidate.
  s.begin(p);
  a = validSeg(); addDet(a, 9, 0.28f);
  s.feed(a);
  s.feed(validSeg());
  b = validSeg(); confirm(b, 10, 0.27f);
  CHECK(s.feed(b) == false);
  CHECK(s.outcome().verdict == Verdict::kCandidate);

  // Untested single line is reported as possible and flagged not vetted.
  s.begin(p);
  a = validSeg(); addDet(a, 4, 0.30f, 9.0f, false);
  s.feed(a); s.feed(validSeg()); s.feed(validSeg());
  CHECK(s.outcome().verdict == Verdict::kPossible);
  CHECK(!s.outcome().vetted);

  // Lines that only coincide with platform motion are neither a lead nor a negative: CONFOUNDED.
  s.begin(p);
  a = validSeg(); addDet(a, 9, 0.28f, 12.0f, false, true);
  s.feed(a);
  s.feed(validSeg()); s.feed(validSeg());
  CHECK(s.outcome().verdict == Verdict::kConfounded);
  CHECK(s.outcome().hz > 0.27f && s.outcome().bin == 9 && !s.outcome().vetted);
  // ... but a genuine (even unvetted) lead elsewhere takes precedence over them.
  s.begin(p);
  a = validSeg();
  addDet(a, 9, 0.28f, 14.0f, false, true);
  addDet(a, 12, 0.50f, 5.0f, true, false);
  s.feed(a);
  s.feed(validSeg()); s.feed(validSeg());
  CHECK(s.outcome().verdict == Verdict::kPossible && s.outcome().bin == 12);

  // Invalid segments do not count; two of them end the scan.
  s.begin(p);
  CHECK(s.feed(invalidSeg()) == true);
  CHECK(s.feed(invalidSeg(InvalidReason::kDropout)) == false);
  CHECK(s.outcome().verdict == Verdict::kInvalid);
  CHECK(s.outcome().last_invalid == InvalidReason::kDropout);

  // One invalid segment in between is tolerated.
  s.begin(p);
  s.feed(validSeg());
  CHECK(s.feed(invalidSeg()) == true);
  CHECK(s.feed(validSeg()) == false);
  CHECK(s.outcome().verdict == Verdict::kNoPattern);

  // A lead in the only valid segment is surfaced instead of being thrown away.
  s.begin(p);
  a = validSeg(); addDet(a, 9, 0.28f);
  s.feed(a); s.feed(invalidSeg());
  CHECK(s.feed(invalidSeg()) == false);
  CHECK(s.outcome().verdict == Verdict::kPossible);

  // Platform lines that could not be cancelled -> confounded, not "no pattern".
  s.begin(p);
  a = validSeg(); a.n_platform = 3; a.cpe_applied = false;
  s.feed(a); s.feed(a);
  CHECK(s.outcome().verdict == Verdict::kConfounded);
  // ... when cancelled they only raise the vibration notice.
  s.begin(p);
  a = validSeg(); a.n_platform = 3; a.cpe_applied = true; a.platform_peak = true; a.platform_hz = 0.3f;
  s.feed(a); s.feed(a);
  CHECK(s.outcome().verdict == Verdict::kNoPattern);
  CHECK(s.outcome().vibration);

  // Negative result with a floor far above the bench baseline is inconclusive.
  DspParams pb = p;
  pb.have_baseline = true; pb.baseline_floor_db = 21.0f; pb.low_sens_db = 6.0f;
  s.begin(pb);
  s.feed(validSeg(29.0f)); s.feed(validSeg(21.5f));
  CHECK(s.outcome().verdict == Verdict::kLowSensitivity);
  CHECK_NEAR(s.outcome().floor_delta_db, 8.0, 1e-3);
  CHECK(s.outcome().low_sens);
  s.begin(pb);
  s.feed(validSeg(21.4f)); s.feed(validSeg(22.0f));
  CHECK(s.outcome().verdict == Verdict::kNoPattern && s.outcome().sens_known);
  CHECK(!s.outcome().low_sens);
  // The flag follows the noise floor, not the verdict: a line that stands out above a raised floor is still
  // reported (CFAR adapts to the floor) but the scan says that its sensitivity was reduced.
  s.begin(pb);
  a = validSeg(29.0f); addDet(a, 9, 0.28f, 14.0f, false, true);
  s.feed(a);
  b = validSeg(29.0f); confirm(b, 9, 0.28f);
  s.feed(b);
  CHECK(s.outcome().verdict == Verdict::kCandidate);
  CHECK(s.outcome().low_sens);
  CHECK_NEAR(s.outcome().floor_delta_db, 8.0, 1e-3);
  s.begin(pb);                                    // ... and an unconfirmed line at a raised floor is POSSIBLE with low_sens set
  a = validSeg(29.0f); addDet(a, 12, 0.5f, 5.0f, true, false);
  s.feed(a); s.feed(validSeg(29.0f)); s.feed(validSeg(29.0f));
  CHECK(s.outcome().verdict == Verdict::kPossible);
  CHECK(s.outcome().low_sens);

  // Seed list: duplicates merge, strongest kept, capped.
  s.begin(p);
  a = validSeg();
  for (int i = 0; i < 4; i++) addDet(a, static_cast<uint8_t>(5 + 3 * i), 0.2f + 0.1f * i, 5.0f + i);
  s.feed(a);
  b = validSeg();
  addDet(b, 5, 0.21f, 12.0f);                    // duplicate of seed 0 but stronger
  addDet(b, 20, 0.7f, 3.0f);                     // weaker than all four -> dropped
  s.feed(b);
  sd = s.seeds(ns);
  CHECK(ns == kMaxSeeds);
  bool found = false;
  for (int i = 0; i < ns; i++) if (sd[i].bin == 5 && sd[i].margin_db > 11.0f) found = true;
  CHECK(found);
}

static ScanOutcome scenario(const SimScene& sc, uint32_t seed, DspParams p = DspParams()) { return runScan(sc, seed, p); }

void test_scenarios() {
  // Trial A: person breathing behind a barrier, quiet scene.
  {
    int cand = 0;
    for (uint32_t t = 0; t < 12; t++) {
      SimScene sc; sc.target = true; sc.target_amp = 30.0f;
      const ScanOutcome o = scenario(sc, 100 + t);
      if (o.verdict == Verdict::kCandidate) {
        cand++;
        CHECK_NEAR(o.bpm, 0.28 * 60.0, 1.5);
        CHECK(abs(static_cast<int>(o.bin) - 9) <= 1);
      }
    }
    CHECK(cand >= 11);
  }
  // Trial B: empty scene must essentially never produce a candidate.
  {
    int cand = 0, nop = 0;
    for (uint32_t t = 0; t < 40; t++) {
      SimScene sc;
      const ScanOutcome o = scenario(sc, 500 + t);
      if (o.verdict == Verdict::kCandidate) cand++;
      if (o.verdict == Verdict::kNoPattern) nop++;
    }
    CHECK(cand == 0);
    CHECK(nop >= 35);
  }
  // Trial C: strong sensor motion -> invalid scan (never a negative).
  {
    SimScene sc; sc.target = true; sc.target_amp = 40.0f; sc.bump_rate_hz = 0.6f; sc.bump_mm = 1.2f;
    const ScanOutcome o = scenario(sc, 7);
    CHECK(o.verdict == Verdict::kInvalid);
    CHECK(o.last_invalid == InvalidReason::kMotion);
  }
  // Trial E: sub-millimetre vibration the IMU cannot see, no person. Must not become a candidate.
  {
    int cand = 0, vib = 0;
    for (uint32_t t = 0; t < 20; t++) {
      SimScene sc; sc.vib_hz = 0.30f; sc.vib_mm = 0.2f;
      const ScanOutcome o = scenario(sc, 900 + t);
      if (o.verdict == Verdict::kCandidate) cand++;
      if (o.vibration) vib++;
    }
    CHECK(cand == 0);
    CHECK(vib >= 19);
  }
  // Person present AND the platform vibrates: the person must still be found.
  {
    int cand = 0;
    for (uint32_t t = 0; t < 10; t++) {
      SimScene sc; sc.target = true; sc.target_amp = 30.0f; sc.vib_hz = 0.45f; sc.vib_mm = 0.2f;
      const ScanOutcome o = scenario(sc, 1300 + t);
      if (o.verdict == Verdict::kCandidate && fabsf(o.bpm - 16.8f) < 1.5f) cand++;
    }
    CHECK(cand >= 9);
  }
}

void test_missing_imu_and_dropout() {
  SegmentBuffer& buf = sharedBuffer();
  Workspace& ws = sharedWorkspace();
  DspParams p;
  SimScene sc;
  SimRun run;
  run.begin(sc, 3);
  buf.reset(sc.n_bins);
  run.fill(buf);
  SegmentResult r;

  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(r.valid);

  for (int i = 0; i < kSegFrames; i++) buf.flags[i] |= kFlagImuMissing;       // IMU absent
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kImuMissing);
  DspParams bench = p; bench.require_imu = false;                              // explicit bench override
  analyzeSegment(buf, kSegFrames, bench, nullptr, 0, ws, r);
  CHECK(r.valid);

  for (int i = 0; i < kSegFrames; i++) buf.flags[i] = 0;
  for (int i = 0; i < 20; i++) buf.flags[i * 20] |= kFlagDropped;              // 3.9 % lost frames
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kDropout);

  for (int i = 0; i < kSegFrames; i++) buf.flags[i] = 0;
  for (int i = 0; i < 40; i++) buf.flags[i] |= kFlagImuMotion;                 // 7.8 % unstable frames
  analyzeSegment(buf, kSegFrames, p, nullptr, 0, ws, r);
  CHECK(!r.valid && r.reason == InvalidReason::kMotion);

  analyzeSegment(buf, 100, p, nullptr, 0, ws, r);                              // too little data
  CHECK(!r.valid && r.reason == InvalidReason::kTooShort);
}

void test_decision() {
  RUN(test_state_machine);
  RUN(test_missing_imu_and_dropout);
  RUN(test_scenarios);
}
