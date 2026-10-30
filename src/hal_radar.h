// RubbleSense firmware - radar front-end abstraction.
//
// Everything above this interface (acquisition, DSP, decision, UI) only sees complex I/Q
// frames: one int16 I/Q value per range bin per slow-time frame. Back-ends:
//   * SimSource     - the scene simulator from rs_core (Wokwi / bench demos)
//   * SerialIqSource- a front-end MCU, a hardware-in-the-loop rig or a PC replaying a
//                     recording, speaking the rs_proto serial protocol
// A real radar front-end is added by implementing this interface (or, much simpler, by
// making its MCU speak the serial protocol).
#pragma once
#include <stdint.h>
#include "rs_imu.h"
#include "rs_proto.h"
#include "rs_types.h"

struct LinkStats {
  uint32_t frames_ok = 0;       // frames handed to the acquisition
  uint32_t lost_frames = 0;     // frames the sequence numbers say are missing
  uint32_t duplicates = 0;      // repeated / out-of-order frames that were discarded
  uint32_t crc_errors = 0;
  uint32_t bad_packets = 0;     // header-level rejects, frames with a wrong bin count
  uint32_t bytes_skipped = 0;
  uint32_t rx_overflow = 0;     // bytes lost because the firmware did not drain the UART in time
  uint32_t timing_flagged = 0;  // frames whose timestamps disagreed with the announced frame rate
  bool     timing_checked = false;  // the front-end sends timestamps at all (otherwise nothing could be checked)
};

struct RadarPoll {
  rs::RadarFrame frame;
  uint8_t  flags = 0;           // rs::kProtoFlag* bits
  uint16_t lost = 0;            // frames lost on the link immediately before this one
  bool     timing_bad = false;  // the front-end's timestamps disagree with its sequence numbers (see SerialIqSource)
};

class IRadarSource {
 public:
  virtual ~IRadarSource() {}

  virtual const char* name() const = 0;                 // "SIM", "SERIAL"
  virtual bool begin() = 0;                             // hardware init; false = unusable
  virtual bool ready() = 0;                             // geometry known, a scan may start
  virtual bool info(rs::RadarInfo& out) const = 0;      // frame rate, range axis, wavelength, bins

  // ---- lifecycle hooks. All are called from the acquisition thread (activate() and begin() at boot
  // ---- excepted: they run before that thread exists), never from the UI thread.

  // The source has just been selected: drop whatever state an earlier selection left behind.
  virtual void activate(uint32_t /*now_us*/) {}

  // No segment is being acquired (idle, or between the segments of a scan while one is analysed),
  // called at >= 1 kHz. A source that receives data asynchronously uses it to keep its input
  // drained and its front-end description fresh, so that ready() can become true before a scan.
  virtual void idle(uint32_t /*now_us*/) {}

  // A new scan starts with this geometry: reset the per-scan statistics and remember what the
  // scan assumes (frame rate, bins, range axis).
  virtual void beginScan(const rs::RadarInfo& /*geometry*/) {}

  // True if the front-end announced a different geometry than the one the running scan was started
  // with. The scan's frequency axis would then be wrong, so the scan must be abandoned.
  virtual bool geometryChanged() const { return false; }

  // A new decision segment starts at `now_us`: reset pacing, sequence tracking, stale input.
  virtual void startSegment(uint32_t now_us) = 0;

  // Non-blocking. Returns true and fills `out` when a frame is available at `now_us`.
  virtual bool poll(uint32_t now_us, RadarPoll& out) = 0;

  // True if frames carry the sensor-stability verdict of an IMU that is mounted with the radar
  // (protocol flag kProtoFlagHostImu). The local IMU is then not consulted for those frames.
  virtual bool providesImu() const { return false; }

  // Simulator only: speed-up factor (> 1 = faster than real time) and IMU hooks.
  virtual float warp() const { return 1.0f; }
  virtual void imuInject(float& /*ax*/, float& /*gx*/) {}
  // Simulator only: supply the 5 IMU samples of a frame when no IMU hardware is attached.
  // Returns true if samples were pushed.
  virtual bool canSimulateImu() const { return false; }
  virtual bool imuFrame(rs::ImuProcessor& /*proc*/) { return false; }

  virtual void linkStats(LinkStats& s) const { s = LinkStats(); }
};
