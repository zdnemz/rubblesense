// RubbleSense core - sensor-stability monitor (IMU post-processing).
//
// The IMU is sampled faster than the radar frame rate (e.g. 100 Hz) and reduced to
// one record per radar frame. Compared with the original prototype this
//   * uses physical units with separate thresholds (m/s^2, rad/s, deg/s)
//   * removes the gyro zero-rate bias (MPU6050 offsets reach tens of deg/s)
//   * watches the gravity direction for slow tilt/slip, which falls straight into
//     the respiration band and is invisible to a high-passed accelerometer
//   * keeps a frame flagged for a short hold time after each hit
//
// Limitation (documented, not hidden): an MPU6050 cannot see sub-millimetre
// micro-vibration at breathing frequencies. That case is handled in the radar
// domain by the common-phase-error compensation in rs_dsp.
#pragma once
#include <stdint.h>

namespace rs {

struct ImuGateParams {
  float   a_rms_max         = 0.15f;  // m/s^2   dynamic acceleration RMS per frame
  float   g_rms_max         = 0.05f;  // rad/s   bias-removed gyro RMS per frame (~2.9 deg/s)
  float   tilt_rate_max_dps = 1.5f;   // deg/s   change of gravity direction over ~1 s
  uint8_t hold_frames       = 10;     // frames kept flagged after a hit (0.5 s @ 20 Hz)
};

struct ImuStats {
  float a_rms;          // m/s^2
  float g_rms;          // rad/s
  float tilt_rate_dps;  // deg/s
  bool  motion;         // frame must be treated as unstable
  bool  valid;          // false when no IMU sample arrived during this frame
};

class ImuProcessor {
 public:
  void configure(const ImuGateParams& gp, float imu_hz, float frame_hz);
  void reset();
  // Call at the IMU sample rate (m/s^2 and rad/s).
  void push(float ax, float ay, float az, float gx, float gy, float gz);
  // Call once per radar frame: returns the statistics since the previous call.
  ImuStats takeFrame();

 private:
  static constexpr int kHist = 64;   // tilt history ring (>= frame_hz)

  ImuGateParams gp_{};
  float imu_hz_ = 100.0f, frame_hz_ = 20.0f;
  float a_alpha_ = 0.02f, b_alpha_ = 0.0033f;
  int   hist_len_ = 20;

  bool  init_ = false;
  int   init_n_ = 0;
  float acc_lp_[3] = {0, 0, 0};
  float gbias_[3] = {0, 0, 0};
  bool  update_bias_ = true;

  float  sum_a2_ = 0, sum_g2_ = 0;
  int    n_ = 0;

  float hist_[kHist][3];
  int   hist_n_ = 0, hist_head_ = 0;
  uint8_t hold_ = 0;
};

}  // namespace rs
