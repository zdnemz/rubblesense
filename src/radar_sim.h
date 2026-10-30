// RubbleSense firmware - simulated radar front-end (Wokwi and bench demos).
//
// Wraps rs::SimRadar. The scene comes from a "trial preset" or, in MANUAL, from two
// potentiometers. This is a MODEL of ideal point reflectors in white noise - it exists so
// the whole signal chain (acquisition -> DSP -> decision -> UI) can be exercised without a
// radar. Nothing here says anything about real rubble.
#pragma once
#include "hal_radar.h"
#include "rs_build.h"
#include "rs_simgen.h"

struct TrialPreset {
  const char* id;        // short tag shown on screen / in the log
  const char* name_en;
  const char* name_id;
  bool        manual;    // scene follows the potentiometers
};

constexpr uint8_t kNumPresets = 7;
constexpr uint8_t kPresetManual = 0;

// idx is clamped into range.
const TrialPreset& trialPreset(uint8_t idx);

// Fill `s` with the scene of preset `idx` (MANUAL gives the neutral scene).
void applyPreset(uint8_t idx, rs::SimScene& s);

class SimSource : public IRadarSource {
 public:
  explicit SimSource(float warp = RS_SIM_WARP_DEFAULT);

  void setPreset(uint8_t idx);
  uint8_t preset() const { return preset_; }
  // MANUAL preset: target reflection amplitude (counts) and receiver noise (counts rms).
  void setManual(float target_amp, float noise_sigma);
  void setWarp(float w) { warp_ = (w < 1.0f) ? 1.0f : w; }
  // New random scene (the operator "re-positions" the sensor). Deterministic for a given seed.
  void restart(uint32_t seed);
  rs::SimScene& scene() { return radar_.scene; }

  // IRadarSource
  const char* name() const override { return "SIM"; }
  bool begin() override { return true; }
  bool ready() override { return true; }
  bool info(rs::RadarInfo& out) const override;
  void startSegment(uint32_t now_us) override;
  bool poll(uint32_t now_us, RadarPoll& out) override;
  float warp() const override { return warp_; }
  void imuInject(float& ax, float& gx) override;
  bool canSimulateImu() const override { return true; }
  bool imuFrame(rs::ImuProcessor& proc) override;

 private:
  rs::SimRadar radar_;
  rs::Rng kick_rng_{7};
  uint8_t  preset_ = 0;
  float    warp_ = 1.0f;
  uint32_t t0_us_ = 0;
  uint32_t n_ = 0;
};
