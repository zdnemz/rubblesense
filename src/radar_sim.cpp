#include "radar_sim.h"
#include "rs_sync.h"

namespace {

const TrialPreset kPresets[kNumPresets] = {
    {"MANUAL", "MANUAL (pots + IMU)",       "MANUAL (pot + IMU)",       true},
    {"A",      "A: Person, quiet",          "A: Ada orang, tenang",     false},
    {"B",      "B: Empty (control)",        "B: Kosong (kontrol)",      false},
    {"C",      "C: Sensor knocked",         "C: Sensor terguncang",     false},
    {"D",      "D: Person + vibration",     "D: Orang + getaran",       false},
    {"E",      "E: Vibration = breathing",  "E: Getaran = napas",       false},
    {"F",      "F: Empty, very noisy",      "F: Kosong, sangat bising", false},
};

}  // namespace

const TrialPreset& trialPreset(uint8_t idx) { return kPresets[idx < kNumPresets ? idx : 0]; }

// The numbers below come from the Monte-Carlo experiments in docs/results (white-noise model,
// breathing 0.28 Hz = 16.8 /min, chest excursion 5 mm, wavelength 125 mm).
void applyPreset(uint8_t idx, rs::SimScene& s) {
  s = rs::SimScene();
  switch (idx) {
    case 1:  // A - a person breathing behind the barrier, sensor held steady
      s.target = true; s.target_amp = 30.0f;
      break;
    case 2:  // B - control: nobody there. The best possible outcome is "no reliable pattern".
      break;
    case 3:  // C - the sensor is knocked / handled during the scan -> scan must be rejected
      s.target = true; s.target_amp = 30.0f;
      s.bump_rate_hz = 0.5f; s.bump_mm = 2.0f;
      break;
    case 4:  // D - platform micro-vibration the IMU cannot see, at a different rate than breathing
      s.target = true; s.target_amp = 30.0f;
      s.vib_mm = 0.2f; s.vib_hz = 0.45f;
      break;
    case 5:  // E - the vibration sits exactly at the breathing rate: cannot be told apart -> CONFOUNDED
      s.target = true; s.target_amp = 30.0f;
      s.vib_mm = 0.2f; s.vib_hz = 0.28f;
      break;
    case 6:  // F - nobody there but the receiver is 3x noisier than at calibration: a negative means nothing
      s.noise_sigma = 24.0f;
      break;
    default:  // MANUAL - the pots decide target and noise
      break;
  }
}

SimSource::SimSource(float warp) : warp_(warp < 1.0f ? 1.0f : warp) { setPreset(0); }

void SimSource::setPreset(uint8_t idx) {
  preset_ = (idx < kNumPresets) ? idx : 0;
  rs::SimScene sc;
  applyPreset(preset_, sc);
  radar_.scene = sc;
  radar_.rebuild();
}

void SimSource::setManual(float target_amp, float noise_sigma) {
  rs::SimScene& s = radar_.scene;
  if (target_amp < 0.0f) target_amp = 0.0f;
  if (noise_sigma < 1.0f) noise_sigma = 1.0f;
  s.target = target_amp >= 2.0f;
  s.target_amp = target_amp;
  s.noise_sigma = noise_sigma;
}

void SimSource::restart(uint32_t seed) {
  radar_.reset(seed);          // keeps `scene`, redraws the random static clutter
  kick_rng_.reseed(seed ^ 0xC0FFEEu);
}

bool SimSource::info(rs::RadarInfo& out) const {
  out = radar_.info();
  return true;
}

void SimSource::startSegment(uint32_t now_us) {
  t0_us_ = now_us;
  n_ = 0;
}

bool SimSource::poll(uint32_t now_us, RadarPoll& out) {
  const double period_us = 1.0e6 / (static_cast<double>(radar_.scene.fs_hz) * static_cast<double>(warp_));
  const uint32_t due = t0_us_ + static_cast<uint32_t>(static_cast<double>(n_) * period_us);
  if (!rsReached(now_us, due)) return false;
  radar_.step(out.frame);
  out.flags = out.frame.saturated ? rs::kProtoFlagSaturated : 0;
  out.lost = 0;
  n_++;
  return true;
}

// An impulsive platform step shakes a real IMU too: ~0.4 m/s^2 and ~0.1 rad/s per mm.
void SimSource::imuInject(float& ax, float& gx) {
  const float d = radar_.lastBumpMm();
  if (d <= 0.0f) return;
  const float sign = (kick_rng_.uniform() < 0.5f) ? -1.0f : 1.0f;
  ax += 0.4f * d * sign;
  gx += 0.1f * d * sign;
}

bool SimSource::imuFrame(rs::ImuProcessor& proc) {
  radar_.imuStep(proc);
  return true;
}
