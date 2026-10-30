// RubbleSense firmware - radar front-end that speaks the rs_proto serial protocol.
//
// Bytes are not read from the UART here: RxPump (acquisition thread) is the only reader of
// the physical UARTs and queues the I/Q bytes in a ring. That lets the console and the I/Q
// stream share UART0 (USB) for replay / hardware-in-the-loop work.
//
// The front-end must announce itself with an info packet at start-up and then at least every
// 2 s. The description is kept fresh while the device is idle (idle()), so ready() is true as
// soon as a front-end is connected and false again kInfoFreshUs after it goes silent.
#pragma once
#include <atomic>
#include <stdint.h>
#include "byte_ring.h"
#include "hal_radar.h"
#include "rs_proto.h"
#include "rs_sync.h"

typedef ByteRing<4096> IqRing;

class SerialIqSource : public IRadarSource {
 public:
  static constexpr uint32_t kInfoFreshUs = 5000000;      // an info packet must have arrived within this time
  static constexpr uint32_t kFlagFreshUs = 2000000;      // ... and the IMU flag of the latest frame within this time

  explicit SerialIqSource(IqRing& ring) : ring_(ring) {}

  const char* name() const override { return "SERIAL"; }
  bool begin() override;                                  // boot only (the acquisition thread does not exist yet)
  bool ready() override;
  bool info(rs::RadarInfo& out) const override;
  void activate(uint32_t now_us) override;
  void idle(uint32_t now_us) override;
  void beginScan(const rs::RadarInfo& geometry) override;
  bool geometryChanged() const override { return geo_changed_.load(std::memory_order_acquire); }
  void startSegment(uint32_t now_us) override;
  bool poll(uint32_t now_us, RadarPoll& out) override;
  bool providesImu() const override;
  void linkStats(LinkStats& s) const override;

  // Frames per second the sender announced (0 until the info packet arrived).
  float frameRate() const;

 private:
  void resetState();
  void onInfo(const rs::ProtoInfo& in);
  void noteImuFlag(uint8_t flags);
  bool acceptFrame(RadarPoll& out);
  bool timingBad(const rs::RadarFrame& f);

  IqRing&         ring_;
  rs::FrameParser parser_;          // while a segment is acquired
  rs::FrameParser idle_parser_;     // otherwise: only the front-end description matters

  // geometry the running scan was started with (acquisition thread)
  bool                pinned_ = false;
  rs::RadarInfo       geo_{};
  std::atomic<bool>   geo_changed_{false};

  // published front-end description (written by the acquisition thread, read by the UI thread)
  mutable RsLock      lock_;
  rs::ProtoInfo       pub_{};
  bool                pub_have_ = false;
  uint32_t            pub_us_ = 0;

  bool     have_seq_ = false;
  bool     host_flag_ = false;        // the latest frame carried "IMU is mounted with the radar" (kProtoFlagHostImu)
  uint32_t host_seen_us_ = 0;         // ... and arrived at this time
  uint16_t last_seq_ = 0;
  uint32_t frames_ok_ = 0, lost_ = 0, dups_ = 0, wrong_bins_ = 0;
  uint32_t overflow0_ = 0;
  // Timestamp consistency, judged per segment: the time the front-end's timestamps say has passed must
  // match what its sequence numbers and announced frame rate imply (a wrong rate would shift every
  // frequency and the reported breathing rate; a pause without a sequence gap breaks the sampling grid).
  bool     t_have_ = false;           // reference frame of this segment taken
  uint32_t t_ref_ms_ = 0;
  uint16_t t_ref_seq_ = 0;
  bool     t_seen_ = false;           // the front-end sends timestamps at all (all-zero stamps cannot be checked)
  uint32_t timing_flagged_ = 0;
  // parser counters accumulated across parser resets
  uint32_t acc_crc_ = 0, acc_bad_ = 0, acc_skip_ = 0;
};
