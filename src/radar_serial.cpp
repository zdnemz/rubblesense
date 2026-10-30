#include "radar_serial.h"
#include <Arduino.h>
#include <math.h>

constexpr uint32_t SerialIqSource::kInfoFreshUs;
constexpr uint32_t SerialIqSource::kFlagFreshUs;

namespace {
// The frame rate is judged once at least this many frames have passed since the first frame of a segment
// (5 s at 20 Hz), and may be off by this fraction: the breathing-rate estimate stays within 2 %, and
// a pause worth 2 % of a segment equals the lost-frame tolerance (DspParams::max_drop_frac).
constexpr uint16_t kTimingMinFrames = 100;
constexpr float    kTimingTolerance = 0.02f;

bool differs(const rs::ProtoInfo& a, const rs::RadarInfo& b) {
  return fabsf(a.frame_rate_hz - b.frame_rate_hz) > 1e-3f || a.n_bins != b.n_bins ||
         fabsf(a.range_start_m - b.range_start_m) > 1e-4f || fabsf(a.range_step_m - b.range_step_m) > 1e-5f ||
         fabsf(a.wavelength_mm - b.wavelength_mm) > 1e-3f;
}
}  // namespace

void SerialIqSource::resetState() {
  parser_.reset();
  idle_parser_.reset();
  ring_.clear();
  pinned_ = false;
  geo_changed_.store(false, std::memory_order_release);
  have_seq_ = host_flag_ = false;
  frames_ok_ = lost_ = dups_ = wrong_bins_ = 0;
  acc_crc_ = acc_bad_ = acc_skip_ = 0;
  t_have_ = t_seen_ = false;
  timing_flagged_ = 0;
  overflow0_ = ring_.overflow();
  RsGuard g(lock_);
  pub_have_ = false;
}

bool SerialIqSource::begin() {
  resetState();
  return true;
}

void SerialIqSource::activate(uint32_t /*now_us*/) { resetState(); }

bool SerialIqSource::ready() {
  RsGuard g(lock_);
  return pub_have_ && static_cast<uint32_t>(micros() - pub_us_) < kInfoFreshUs;
}

bool SerialIqSource::info(rs::RadarInfo& out) const {
  rs::ProtoInfo in;
  {
    RsGuard g(lock_);
    if (!pub_have_) return false;
    in = pub_;
  }
  out.name = "SERIAL";
  out.frame_rate_hz = in.frame_rate_hz;
  out.range_start_m = in.range_start_m;
  out.range_step_m = in.range_step_m;
  out.wavelength_mm = in.wavelength_mm;
  out.n_bins = in.n_bins;
  return true;
}

float SerialIqSource::frameRate() const {
  RsGuard g(lock_);
  return pub_have_ ? pub_.frame_rate_hz : 0.0f;
}

void SerialIqSource::onInfo(const rs::ProtoInfo& in) {
  if (pinned_ && differs(in, geo_)) geo_changed_.store(true, std::memory_order_release);
  RsGuard g(lock_);
  pub_ = in;
  pub_have_ = true;
  pub_us_ = micros();
}

void SerialIqSource::noteImuFlag(uint8_t flags) {
  host_flag_ = (flags & rs::kProtoFlagHostImu) != 0;
  host_seen_us_ = micros();
}

bool SerialIqSource::providesImu() const {
  return host_flag_ && static_cast<uint32_t>(micros() - host_seen_us_) < kFlagFreshUs;
}

void SerialIqSource::beginScan(const rs::RadarInfo& geometry) {
  geo_ = geometry;
  pinned_ = true;
  geo_changed_.store(false, std::memory_order_release);
  parser_.reset();
  have_seq_ = host_flag_ = false;
  frames_ok_ = lost_ = dups_ = wrong_bins_ = 0;
  acc_crc_ = acc_bad_ = acc_skip_ = 0;
  t_have_ = t_seen_ = false;
  timing_flagged_ = 0;
  overflow0_ = ring_.overflow();
}

// Between segments and while idle: drain the ring so it never overflows, and keep the front-end
// description fresh. Frames are dropped here (a segment always starts from a clean slate).
void SerialIqSource::idle(uint32_t /*now_us*/) {
  int budget = 1024;
  uint8_t b;
  while (budget-- > 0 && ring_.pop(b)) {
    rs::FrameParser::Event e = idle_parser_.push(b);
    while (e != rs::FrameParser::Event::kNone) {
      if (e == rs::FrameParser::Event::kInfo) onInfo(idle_parser_.info());
      else if (e == rs::FrameParser::Event::kFrame) noteImuFlag(idle_parser_.flags());
      e = idle_parser_.poll();
    }
  }
}

void SerialIqSource::startSegment(uint32_t /*now_us*/) {
  // Whatever piled up while the previous segment was being analysed is stale, and so is any
  // half-received packet. Keep the error counters.
  acc_crc_ += parser_.crcErrors();
  acc_bad_ += parser_.badPackets();
  acc_skip_ += parser_.bytesSkipped();
  parser_.reset();
  ring_.clear();
  have_seq_ = false;       // the first frame of a segment never counts as "after a gap"
  t_have_ = false;         // ... and the timing reference is taken again
}

// True if this frame's timestamp is inconsistent with the sequence numbers at the announced rate.
// All-zero timestamps mean the front-end does not send any: that cannot be checked (and is reported).
bool SerialIqSource::timingBad(const rs::RadarFrame& f) {
  if (f.t_ms != 0) t_seen_ = true;
  if (!t_seen_ || geo_.frame_rate_hz <= 0.0f) return false;
  if (!t_have_) {
    t_have_ = true;
    t_ref_ms_ = f.t_ms;
    t_ref_seq_ = f.seq;
    return false;
  }
  const uint16_t dseq = static_cast<uint16_t>(f.seq - t_ref_seq_);
  if (dseq < kTimingMinFrames) return false;
  const float elapsed_ms = static_cast<float>(static_cast<int32_t>(f.t_ms - t_ref_ms_));
  const float expected_ms = static_cast<float>(dseq) * (1000.0f / geo_.frame_rate_hz);
  return fabsf(elapsed_ms - expected_ms) > kTimingTolerance * expected_ms;
}

// Called for every decoded frame packet; returns false if the frame must be discarded.
bool SerialIqSource::acceptFrame(RadarPoll& out) {
  const rs::RadarFrame& f = parser_.frame();
  if (pinned_ && f.n_bins != geo_.n_bins) {         // geometry changed under us: not trustworthy
    wrong_bins_++;
    return false;
  }
  uint16_t lost = 0;
  if (have_seq_) {
    const uint16_t gap = rs::seqGap(last_seq_, f.seq);
    if (gap >= 0x8000u) {                           // same or older sequence number
      dups_++;
      return false;
    }
    lost = gap;
  }
  have_seq_ = true;
  last_seq_ = f.seq;
  lost_ += lost;

  const uint8_t flags = parser_.flags();
  noteImuFlag(flags);
  out.frame = f;
  out.flags = flags;
  out.lost = lost;
  out.timing_bad = timingBad(f);
  if (out.timing_bad) timing_flagged_++;
  frames_ok_++;
  return true;
}

bool SerialIqSource::poll(uint32_t /*now_us*/, RadarPoll& out) {
  rs::FrameParser::Event e = parser_.poll();         // packets that were already buffered
  int budget = 1024;                                 // bytes examined per call
  for (;;) {
    if (e == rs::FrameParser::Event::kNone) {
      uint8_t b;
      if (budget-- <= 0 || !ring_.pop(b)) return false;
      e = parser_.push(b);
      continue;
    }
    if (e == rs::FrameParser::Event::kInfo) {
      onInfo(parser_.info());
    } else if (acceptFrame(out)) {                   // kFrame
      return true;
    }
    e = parser_.poll();
  }
}

void SerialIqSource::linkStats(LinkStats& s) const {
  s.frames_ok = frames_ok_;
  s.lost_frames = lost_;
  s.duplicates = dups_;
  s.crc_errors = acc_crc_ + parser_.crcErrors();
  s.bad_packets = acc_bad_ + parser_.badPackets() + wrong_bins_;
  s.bytes_skipped = acc_skip_ + parser_.bytesSkipped();
  s.rx_overflow = ring_.overflow() - overflow0_;
  s.timing_flagged = timing_flagged_;
  s.timing_checked = t_seen_;
}
