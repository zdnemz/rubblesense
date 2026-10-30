#include "acq.h"
#include <string.h>

namespace {
bool sameGate(const rs::ImuGateParams& a, const rs::ImuGateParams& b) {
  return a.a_rms_max == b.a_rms_max && a.g_rms_max == b.g_rms_max &&
         a.tilt_rate_max_dps == b.tilt_rate_max_dps && a.hold_frames == b.hold_frames;
}
}  // namespace

// Out-of-class definitions (needed before C++17 if a constant is ever odr-used).
constexpr uint16_t Acquisition::kSettleFrames;
constexpr uint32_t Acquisition::kStallUs;
constexpr uint32_t Acquisition::kImuPeriodUs;
constexpr int      Acquisition::kMaxFramesPerCall;

void Acquisition::attach(IRadarSource* src, ImuHw* imu, rs::SegmentBuffer* buf) {
  src_ = src;
  imu_ = imu;
  buf_ = buf;
  proc_.configure(gate_, 100.0f, frame_hz_);
  hw_ok_ = imu_ && imu_->present();      // so the first screen does not claim "IMU missing"
  publish();
}

// ------------------------------------------------------------------ mailbox (UI thread)

void Acquisition::configure(const rs::ImuGateParams& gate, const rs::RadarInfo& info) {
  {
    RsGuard g(lock_);
    cmd_.configure = true;
    cmd_.gate = gate;
    cmd_.info = info;
  }
  cmd_pending_.store(true, std::memory_order_release);
}

void Acquisition::setSource(IRadarSource* src) {
  {
    RsGuard g(lock_);
    cmd_.set_src = true;
    cmd_.src = src;
    cmd_.arm = false;
  }
  cmd_pending_.store(true, std::memory_order_release);
}

void Acquisition::requestArm(bool new_scan) {
  {
    RsGuard g(lock_);
    cmd_.arm = true;
    cmd_.settle = new_scan;
    cmd_.stop = false;
    // The UI thread must never see what is being replaced - a full buffer, a stall flag, the geometry
    // latch of the previous scan - in the window before the acquisition thread has taken the command
    // (it would, e.g., analyse the old segment a second time as if it were a new one). So the snapshot
    // already shows the state the command will produce. publish() leaves it alone until then.
    status_.mode = new_scan ? AcqStatus::kSettling : AcqStatus::kRunning;
    status_.frames = 0;
    status_.stalled = false;
    status_.geometry_changed = false;
    status_.motion_frames = status_.dropped_frames = status_.sat_frames = 0;
  }
  cmd_pending_.store(true, std::memory_order_release);
}

void Acquisition::requestStop() {
  {
    RsGuard g(lock_);
    cmd_.stop = true;
    cmd_.arm = false;
  }
  cmd_pending_.store(true, std::memory_order_release);
}

AcqStatus Acquisition::status() const {
  RsGuard g(lock_);
  return status_;
}

// ------------------------------------------------------------------ acquisition thread

void Acquisition::applyCommands(uint32_t now_us) {
  if (!cmd_pending_.load(std::memory_order_acquire)) return;
  Cmd c;
  {
    RsGuard g(lock_);
    c = cmd_;
    cmd_ = Cmd();
    cmd_pending_.store(false, std::memory_order_relaxed);
  }
  if (c.set_src) {
    src_ = c.src;
    mode_.store(AcqStatus::kIdle);
    have_prev_ = false;
    if (src_) src_->activate(now_us);                    // drop what an earlier selection left behind
  }
  if (c.configure) {
    n_bins_ = c.info.n_bins;
    geometry_ = c.info;
    // Re-initialising the IMU filters costs ~0.5 s of settling (and would absorb a jolt as gyro
    // bias), so only do it when something that matters changed.
    if (c.info.frame_rate_hz != frame_hz_ || !sameGate(c.gate, gate_)) {
      gate_ = c.gate;
      frame_hz_ = c.info.frame_rate_hz;
      proc_.configure(gate_, 100.0f, frame_hz_);
      imu_t_valid_ = false;
      idle_t_valid_ = false;
    }
  }
  if (c.stop) mode_.store(AcqStatus::kIdle);
  if (c.arm && src_ && buf_) {
    motion_frames_ = dropped_frames_ = sat_frames_ = 0;
    stalled_ = false;
    have_prev_ = false;
    buf_->reset(n_bins_);
    if (c.settle) src_->beginScan(geometry_);
    src_->startSegment(now_us);
    last_frame_us_ = now_us;
    settle_left_ = c.settle ? kSettleFrames : 0;
    mode_.store(settle_left_ > 0 ? AcqStatus::kSettling : AcqStatus::kRunning);
  }
  publish();
}

bool Acquisition::imuSample(bool inject) {
  ImuSample s;
  if (imu_ && imu_->present() && imu_->read(s)) {
    float ax = s.ax, gx = s.gx;
    if (inject && src_) src_->imuInject(ax, gx);
    proc_.push(ax, s.ay, s.az, gx, s.gy, s.gz);
    hw_ok_ = true;
    return true;
  }
  hw_ok_ = false;
  return false;
}

void Acquisition::tickImuTimed(uint32_t now_us, bool inject) {
  if (!imu_ || !imu_->present()) return;
  if (!imu_t_valid_) {
    imu_next_us_ = now_us;
    imu_t_valid_ = true;
  }
  for (int n = 0; n < 3 && rsReached(now_us, imu_next_us_); n++) {
    imuSample(inject);
    imu_next_us_ += kImuPeriodUs;
  }
  if (rsReached(now_us, imu_next_us_)) imu_next_us_ = now_us + kImuPeriodUs;   // fell behind: resync
}

void Acquisition::pushHold(uint8_t flags) {
  rs::RadarFrame h;
  if (have_prev_) {
    h = prev_;
  } else {
    memset(&h, 0, sizeof(h));
    h.n_bins = n_bins_;
  }
  buf_->push(h, flags);
  dropped_frames_++;
}

void Acquisition::onFrame(const RadarPoll& p, uint32_t now_us) {
  last_frame_us_ = now_us;
  stalled_ = false;
  frames_total_++;

  const bool host = (p.flags & rs::kProtoFlagHostImu) != 0;
  const bool hw = imu_ && imu_->present();
  last_host_ = host;
  last_sim_ = false;

  // Stability information for this frame.
  if (hw && src_->warp() > 1.0f) {
    for (int i = 0; i < 5; i++) imuSample(true);       // simulated time: sample the IMU per frame
  } else if (!hw) {
    last_sim_ = src_->imuFrame(proc_);                 // simulator without IMU hardware
  }
  const rs::ImuStats st = proc_.takeFrame();           // always: keeps the accumulators bounded
  last_stats_ = st;
  if (host) {
    last_stats_.valid = true;
    last_stats_.motion = (p.flags & rs::kProtoFlagMotion) != 0;
  }

  uint8_t flg = 0;
  if (host) {
    if (p.flags & rs::kProtoFlagMotion) flg |= rs::kFlagImuMotion;
  } else {
    if (st.motion) flg |= rs::kFlagImuMotion;
    if (!st.valid) flg |= rs::kFlagImuMissing;
  }
  if (p.frame.saturated) flg |= rs::kFlagSaturated;
  if (p.flags & rs::kProtoFlagDropped) flg |= rs::kFlagDropped;
  if (p.timing_bad) flg |= rs::kFlagTiming;

  if (mode_.load() == AcqStatus::kSettling) {          // frames are consumed, nothing is stored
    if (settle_left_ > 0) settle_left_--;
    if (settle_left_ == 0) mode_.store(AcqStatus::kRunning);
    publish();
    return;
  }

  // Frames the link lost keep their slot in time: hold frames, flagged, never silently skipped.
  for (uint16_t i = 0; i < p.lost && !buf_->full(); i++) pushHold(rs::kFlagDropped);

  if (!buf_->full()) {
    buf_->push(p.frame, flg);
    prev_ = p.frame;
    have_prev_ = true;
    if (flg & rs::kFlagImuMotion) motion_frames_++;
    if (flg & rs::kFlagSaturated) sat_frames_++;
    if (flg & rs::kFlagDropped) dropped_frames_++;
  }
  if (buf_->full()) mode_.store(AcqStatus::kFull);
  publish();
}

void Acquisition::idleStats(uint32_t now_us) {
  const uint32_t period = static_cast<uint32_t>(1.0e6f / frame_hz_);
  if (!idle_t_valid_) {
    idle_next_us_ = now_us + period;
    idle_t_valid_ = true;
    return;
  }
  if (!rsReached(now_us, idle_next_us_)) return;
  idle_next_us_ += period;
  if (rsReached(now_us, idle_next_us_)) idle_next_us_ = now_us + period;
  last_stats_ = proc_.takeFrame();
  last_host_ = src_ && src_->providesImu();        // a front-end that vouches for stability says so in every frame
  last_sim_ = false;
  publish();
}

void Acquisition::service(uint32_t now_us) {
  applyCommands(now_us);

  const uint8_t m = mode_.load();
  const bool running = (m == AcqStatus::kSettling || m == AcqStatus::kRunning);
  const bool per_frame_imu = running && src_ && src_->warp() > 1.0f;

  if (!per_frame_imu) tickImuTimed(now_us, running);

  if (!running || !src_ || !buf_) {
    idleStats(now_us);
    if (src_) src_->idle(now_us);                        // keep asynchronous inputs drained and described
    return;
  }

  if (!stalled_ && rsReached(now_us, last_frame_us_ + kStallUs)) {
    stalled_ = true;
    publish();
  }

  for (int i = 0; i < kMaxFramesPerCall; i++) {
    RadarPoll p;
    if (!src_->poll(now_us, p)) break;
    onFrame(p, now_us);
    const uint8_t mm = mode_.load();
    if (mm != AcqStatus::kSettling && mm != AcqStatus::kRunning) break;
  }
}

void Acquisition::publish() {
  if (cmd_pending_.load(std::memory_order_acquire)) return;   // see requestArm(): the snapshot is ahead of this thread
  AcqStatus s;
  s.mode = static_cast<AcqStatus::Mode>(mode_.load());
  s.frames = buf_ ? buf_->size() : 0;
  s.stalled = stalled_;
  s.geometry_changed = src_ && src_->geometryChanged();
  const bool hw = imu_ && imu_->present();
  s.imu_hw = hw && hw_ok_;
  s.imu_host = last_host_;
  s.imu_sim = last_sim_ || (!hw && src_ && src_->canSimulateImu());
  s.imu_valid = last_stats_.valid || (s.imu_sim && !hw);
  s.motion = last_stats_.motion;
  s.a_rms = last_stats_.a_rms;
  s.g_rms = last_stats_.g_rms;
  s.tilt_dps = last_stats_.tilt_rate_dps;
  s.motion_frames = motion_frames_;
  s.dropped_frames = dropped_frames_;
  s.sat_frames = sat_frames_;
  s.frames_total = frames_total_;
  s.imu_errors = imu_ ? imu_->errors() : 0;
  RsGuard g(lock_);
  status_ = s;
}
