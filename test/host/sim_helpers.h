// Glue between the scene simulator, the IMU processor and the DSP, shared by the
// host tests and the Monte-Carlo tool. Mirrors what the firmware acquisition task does.
#pragma once
#include <memory>
#include "rs_decision.h"
#include "rs_dsp.h"
#include "rs_imu.h"
#include "rs_simgen.h"

struct SimRun {
  rs::SimRadar radar;
  rs::ImuProcessor imu;
  rs::ImuGateParams gate;

  void begin(const rs::SimScene& sc, uint32_t seed) {
    radar.scene = sc;
    radar.reset(seed);
    imu.configure(gate, 100.0f, sc.fs_hz);
    rs::RadarFrame f;
    for (int i = 0; i < 40; i++) {            // 2 s settling so the IMU filters converge
      radar.step(f);
      radar.imuStep(imu);
      imu.takeFrame();
    }
  }

  // Append `frames` frames to buf (buf.reset() must have been called).
  void fill(rs::SegmentBuffer& buf, uint16_t frames = rs::kSegFrames) {
    rs::RadarFrame f;
    for (uint16_t i = 0; i < frames; i++) {
      radar.step(f);
      radar.imuStep(imu);
      const rs::ImuStats st = imu.takeFrame();
      uint8_t flg = 0;
      if (st.motion) flg |= rs::kFlagImuMotion;
      if (!st.valid) flg |= rs::kFlagImuMissing;
      if (f.saturated) flg |= rs::kFlagSaturated;
      buf.push(f, flg);
    }
  }
};

// Heap-allocated big objects (keeps the test stack small).
inline rs::SegmentBuffer& sharedBuffer() { static std::unique_ptr<rs::SegmentBuffer> p(new rs::SegmentBuffer()); return *p; }
inline rs::Workspace& sharedWorkspace() { static std::unique_ptr<rs::Workspace>    p(new rs::Workspace());    return *p; }

// Everything one worker needs to run scans (the tests share one, the Monte-Carlo tool
// creates one per thread).
struct ScanCtx {
  std::unique_ptr<rs::SegmentBuffer> buf{new rs::SegmentBuffer()};
  std::unique_ptr<rs::Workspace>     ws{new rs::Workspace()};
  rs::ScanSession                    sess;
};

// One complete scan on a simulated scene: segments are acquired back-to-back and fed
// to the decision logic exactly like the firmware does (seeds from earlier segments
// steer the guided confirmation of later ones). The per-segment log stays in c.sess.
inline rs::ScanOutcome runScan(ScanCtx& c, const rs::SimScene& sc, uint32_t seed, const rs::DspParams& p) {
  SimRun run;
  run.begin(sc, seed);
  c.sess.begin(p);
  for (int guard = 0; guard < rs::kMaxSegments + rs::kMaxInvalid + 1; guard++) {
    c.buf->reset(sc.n_bins);
    run.fill(*c.buf);
    uint8_t ns = 0;
    const rs::Seed* sd = c.sess.seeds(ns);
    rs::SegmentResult r;
    rs::analyzeSegment(*c.buf, rs::kSegFrames, p, sd, ns, *c.ws, r);
    if (!c.sess.feed(r)) break;
  }
  return c.sess.outcome();
}

inline rs::ScanOutcome runScan(const rs::SimScene& sc, uint32_t seed, const rs::DspParams& p) {
  static ScanCtx shared;
  return runScan(shared, sc, seed, p);
}
