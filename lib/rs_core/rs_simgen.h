// RubbleSense core - physically motivated radar scene simulator.
//
// Used by (a) the Wokwi build, where no radar exists, and (b) the host test-bench
// that calibrates thresholds. It produces what a real front-end would hand to the
// HAL: one complex value per range bin per slow-time frame (int16 I/Q).
//
// Model:  z_b[n] = ( S_b + T_b * exp(j k x_t[n]) ) * exp(j k x_p[n]) + noise
//   S_b   static reflectors (rubble / barrier) - rigid with the scene
//   T_b   weak target return spread over neighbouring range bins
//   x_t   chest displacement (breathing + small harmonic + heartbeat)
//   x_p   platform displacement (micro-vibration, bumps) common to ALL external
//         reflectors; the antenna-leakage bins 0..1 are rigid with the sensor
//   k     4*pi/lambda
// This is a *model*, not a measurement: real rubble adds multipath, dispersion,
// wind-blown debris and non-Gaussian clutter that are not represented here.
#pragma once
#include <stdint.h>
#include "rs_imu.h"
#include "rs_types.h"

namespace rs {

// PCG32 + Box-Muller. Deterministic across platforms, period 2^64.
class Rng {
 public:
  explicit Rng(uint64_t seed = 1) { reseed(seed); }
  void reseed(uint64_t seed) {
    state_ = 0; inc_ = (seed << 1u) | 1u; next(); state_ += seed ^ 0x9E3779B97F4A7C15ull; next();
    have_ = false;
  }
  uint32_t next() {
    const uint64_t old = state_;
    state_ = old * 6364136223846793005ull + inc_;
    const uint32_t xs = static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
    const uint32_t rot = static_cast<uint32_t>(old >> 59u);
    return (xs >> rot) | (xs << ((32u - rot) & 31u));
  }
  float uniform() { return (static_cast<float>(next() >> 8) + 0.5f) * (1.0f / 16777216.0f); }  // (0,1)
  float normal();
 private:
  uint64_t state_ = 0, inc_ = 1;
  bool have_ = false;
  float spare_ = 0.0f;
};

struct SimScene {
  uint8_t n_bins        = 16;
  float   fs_hz         = 20.0f;
  float   wavelength_mm = 125.0f;   // ~2.4 GHz class; use ~5 for 60 GHz

  // static returns (ADC counts)
  float   leak_amp      = 12000.0f; // bins 0..1, rigid with the sensor
  float   wall_amp      = 9000.0f;  // first external reflector (barrier / rubble face)
  uint8_t wall_bin      = 4;
  float   clutter_amp   = 1500.0f;  // weaker random static reflectors beyond the wall

  // target
  bool    target        = false;
  uint8_t target_bin    = 9;
  float   target_amp    = 30.0f;    // complex reflection amplitude in its main bin (counts)
  float   breath_hz     = 0.28f;
  float   breath_mm     = 5.0f;     // peak chest displacement
  float   breath_jitter = 0.04f;    // relative rate wander (Ornstein-Uhlenbeck, tau = 30 s)
  float   heart_hz      = 1.2f;
  float   heart_mm      = 0.3f;

  // noise
  float   noise_sigma   = 8.0f;     // counts rms on each of I and Q
  float   drift_mm_rms  = 0.0f;     // slow random-walk platform creep

  // platform motion (common phase for all external reflectors)
  float   vib_hz        = 0.0f;
  float   vib_mm        = 0.0f;     // sinusoidal micro-vibration, invisible to an MPU6050 when small
  float   bump_rate_hz  = 0.0f;
  float   bump_mm       = 0.0f;     // impulsive bumps (decay time ~0.6 s)
  float   cpe_mismatch  = 0.0f;     // relative std-dev of the platform phase seen by each bin
                                    // (0 = perfectly common-mode; real scenes are not)

  // what the IMU would report in addition to its own noise
  float   imu_noise_a   = 0.02f;    // m/s^2
  float   imu_noise_g   = 0.004f;   // rad/s
};

class SimRadar {
 public:
  SimRadar() { reset(1); }
  void reset(uint32_t seed);          // rebuilds the static scene from `scene` and restarts time
  void rebuild();                     // call after editing `scene` while running
  void step(RadarFrame& f);           // next slow-time frame
  // Feed the IMU samples (5 per frame @ 100 Hz) that belong to the frame just generated.
  void imuStep(ImuProcessor& imu);
  RadarInfo info() const;
  // Size (mm) of the latest impulsive platform step; the firmware turns it into IMU kicks.
  float lastBumpMm() const { return last_dbump_; }

  SimScene scene;

  // Nominal peak-to-floor ratio (dB) of the folded spectrum for the *target*, small-arc
  // approximation, for an N-frame Hann-windowed segment. Used only to label test results.
  static float nominalToneSnrDb(const SimScene& s, uint16_t n_frames);

 private:
  Rng   rng_;
  float ph_[kMaxBins], gre_[kMaxBins], gim_[kMaxBins];   // random draws that define the scene
  float gmm_[kMaxBins];                   // per-bin platform-phase mismatch draws (own generator)
  float ph_t_ = 0.0f;
  float sre_[kMaxBins], sim_[kMaxBins];   // static complex returns
  float tre_ = 1.0f, tim_ = 0.0f;         // target phase reference
  float bphase_ = 0, hphase_ = 0, vphase_ = 0, wander_ = 0, bump_ = 0, drift_ = 0;
  float last_dbump_ = 0;                  // size of the latest bump step (mm), drives the IMU
  uint16_t seq_ = 0;
  uint32_t t_ms_ = 0;
};

}  // namespace rs
