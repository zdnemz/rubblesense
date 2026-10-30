// RubbleSense firmware - persistent settings (NVS via Preferences).
//
// Flash writes stall the whole chip for milliseconds, so every save*() call must be made
// while no scan is running (the application does that).
#pragma once
#include <stdint.h>
#include "rs_config.h"

class Preferences;

enum class SourceKind : uint8_t { kSim = 0, kSerial = 1 };

struct Settings {
  SourceKind source = SourceKind::kSim;
  uint8_t    preset = 1;              // trial preset (SIM source)
  uint8_t    lang = 0;                // 0 = English, 1 = Indonesian
  bool       sound = true;            // buzzer on/off (rescue teams often need silence on site)
  uint32_t   scan_counter = 0;

  // Noise-floor baselines, one per front-end (a baseline is meaningless for any other radar).
  bool  bl_sim_ok = false;
  float bl_sim_db = 0.0f;
  bool  bl_ser_ok = false;
  float bl_ser_db = 0.0f;

  rs::DspParams params;               // only the tunable subset is persisted (see settings.cpp)
};

// Check a parameter set against what the DSP can actually run with. On failure `why` (if given)
// points to a short static reason. `fs_hz` is the frame rate of the active front-end.
bool paramsValid(const rs::DspParams& p, float fs_hz, const char** why);

class SettingsStore {
 public:
  SettingsStore();
  ~SettingsStore();

  bool begin();                      // open the NVS namespace and load (defaults if empty)
  Settings& get() { return s_; }
  const Settings& get() const { return s_; }

  void saveCore();                   // source, preset, language, sound
  void saveBaselines();
  void saveParams();
  void saveCounter();
  void factoryReset();               // erase everything, back to defaults

 private:
  Settings     s_;
  Preferences* prefs_;
  bool         open_ = false;
};
