// RubbleSense firmware - acquisition engine.
//
// Runs in its own high-priority task (RS_HOST: called by the harness). Responsibilities:
//   * sample the IMU at 100 Hz and reduce it to one stability record per radar frame
//   * poll the radar source and append frames, with per-frame quality flags, to the segment
//   * pad link losses with flagged hold frames so the time axis of the segment stays exact
//   * watch for a stalled radar link
// The UI thread talks to it through a small command mailbox (arm / stop / configure) and a
// status snapshot, never by touching its state directly.
#pragma once
#include <atomic>
#include <stdint.h>
#include "hal_radar.h"
#include "imu_hw.h"
#include "rs_config.h"
#include "rs_imu.h"
#include "rs_sync.h"
#include "rs_types.h"

struct AcqStatus {
  enum Mode : uint8_t { kIdle = 0, kSettling, kRunning, kFull };
  Mode     mode = kIdle;
  uint16_t frames = 0;              // frames stored in the current segment
  bool     stalled = false;         // the radar delivered nothing for kStallUs
  bool     geometry_changed = false;  // the front-end announced other settings than the running scan was started with
  bool     imu_hw = false;          // the IMU hardware answered at the last read
  bool     imu_valid = false;       // the last frame had stability information (hw / simulated / host)
  bool     imu_sim = false;         // the stability information is simulated
  bool     imu_host = false;        // ... comes from the radar side (protocol flag)
  bool     motion = false;          // the last frame was flagged unstable
  float    a_rms = 0.0f, g_rms = 0.0f, tilt_dps = 0.0f;
  uint16_t motion_frames = 0;       // in the current segment
  uint16_t dropped_frames = 0;
  uint16_t sat_frames = 0;
  uint32_t frames_total = 0;
  uint32_t imu_errors = 0;
};

class Acquisition {
 public:
  static constexpr uint16_t kSettleFrames = 30;          // 1.5 s @ 20 Hz: IMU filters converge
  static constexpr uint32_t kStallUs = 3000000;          // no radar frame for this long = stalled
  static constexpr uint32_t kImuPeriodUs = 10000;        // 100 Hz
  static constexpr int      kMaxFramesPerCall = 16;      // bounds the catch-up burst of a late call

  void attach(IRadarSource* src, ImuHw* imu, rs::SegmentBuffer* buf);

  // ---- UI-thread side (mailbox) ------------------------------------------------------
  void configure(const rs::ImuGateParams& gate, const rs::RadarInfo& info);
  void setSource(IRadarSource* src);                     // call only while idle
  // Fresh segment. `new_scan`: it is the first segment of a scan - the IMU filters get a settling
  // time and the source resets its per-scan state (and pins the geometry given to configure()).
  void requestArm(bool new_scan);
  void requestStop();
  AcqStatus status() const;

  // ---- acquisition-thread side -------------------------------------------------------
  void service(uint32_t now_us);                         // call at >= 1 kHz

 private:
  struct Cmd {
    bool configure = false, arm = false, stop = false, settle = false;
    rs::ImuGateParams gate;
    rs::RadarInfo info{};
    IRadarSource* src = nullptr;
    bool     set_src = false;
  };

  void applyCommands(uint32_t now_us);
  void tickImuTimed(uint32_t now_us, bool inject);
  bool imuSample(bool inject);
  void onFrame(const RadarPoll& p, uint32_t now_us);
  void pushHold(uint8_t flags);
  void idleStats(uint32_t now_us);
  void publish();

  IRadarSource*       src_ = nullptr;
  ImuHw*              imu_ = nullptr;
  rs::SegmentBuffer*  buf_ = nullptr;
  rs::ImuProcessor    proc_;
  rs::ImuGateParams   gate_;
  float               frame_hz_ = 20.0f;
  uint8_t             n_bins_ = 1;
  rs::RadarInfo       geometry_{};                       // what the next scan assumes (pinned in the source)

  // mailbox (UI -> acquisition)
  mutable RsLock      lock_;
  Cmd                 cmd_;
  std::atomic<bool>   cmd_pending_{false};

  // acquisition-thread state
  std::atomic<uint8_t> mode_{AcqStatus::kIdle};
  uint16_t   settle_left_ = 0;
  uint32_t   last_frame_us_ = 0;
  uint32_t   imu_next_us_ = 0;
  bool       imu_t_valid_ = false;
  uint32_t   idle_next_us_ = 0;
  bool       idle_t_valid_ = false;
  bool       hw_ok_ = false;
  bool       stalled_ = false;
  bool       have_prev_ = false;
  rs::RadarFrame prev_{};
  uint16_t   motion_frames_ = 0, dropped_frames_ = 0, sat_frames_ = 0;
  uint32_t   frames_total_ = 0;
  rs::ImuStats last_stats_{};
  bool       last_sim_ = false, last_host_ = false;

  AcqStatus  status_;                                    // snapshot, guarded by lock_
};
