// dump_sim - write a simulated scene as an I/Q recording (CSV) that tools/send_iq_replay.py can replay
// into the firmware (Wokwi, a real ESP32, or test/host/fw_replay on the host).
//
//   dump_sim --preset A [--frames 1800] [--seed 1] [--out scene_a.csv]
//   dump_sim --preset manual --amp 30 --sigma 8 --frames 1800 > manual.csv
//
// Presets are the firmware's trial presets: A person quiet, B empty, C sensor knocked, D person +
// platform vibration, E vibration at breathing rate, F empty with 3x noise (see src/radar_sim.cpp).
// The recording carries the verdict of a stability sensor mounted with the radar, frame by frame
// (flags bit1 = motion), exactly what a real front-end with its own IMU would send.
//
// CSV: '#' lines carry metadata (key=value); every other line is one frame:  flags,i0,q0,i1,q1,...
// It is a MODEL of ideal reflectors in white noise (lib/rs_core/rs_simgen.h), not a measurement.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "radar_sim.h"
#include "rs_imu.h"
#include "rs_proto.h"
#include "rs_simgen.h"

namespace {
int parsePreset(const char* s) {
  if (!s || !s[0]) return -1;
  if (!strcasecmp(s, "manual")) return 0;
  if (s[1] == '\0') {
    if (s[0] >= '0' && s[0] <= '9') return (s[0] - '0' < static_cast<int>(kNumPresets)) ? s[0] - '0' : -1;
    const char c = static_cast<char>(s[0] | 0x20);
    if (c >= 'a' && c <= 'f') return 1 + (c - 'a');
  }
  return -1;
}
}  // namespace

int main(int argc, char** argv) {
  int preset = 1, frames = 1800;
  unsigned seed = 1;
  float amp = -1.0f, sigma = -1.0f;
  const char* out_path = nullptr;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    const bool more = i + 1 < argc;
    if (a == "--preset" && more) preset = parsePreset(argv[++i]);
    else if (a == "--frames" && more) frames = std::atoi(argv[++i]);
    else if (a == "--seed" && more) seed = static_cast<unsigned>(std::strtoul(argv[++i], nullptr, 10));
    else if (a == "--amp" && more) amp = static_cast<float>(std::atof(argv[++i]));
    else if (a == "--sigma" && more) sigma = static_cast<float>(std::atof(argv[++i]));
    else if (a == "--out" && more) out_path = argv[++i];
    else {
      std::fprintf(stderr, "usage: %s --preset A..F|0..6|manual [--frames N] [--seed S] [--amp A --sigma X] [--out FILE]\n", argv[0]);
      return 2;
    }
  }
  if (preset < 0 || frames <= 0) {
    std::fprintf(stderr, "bad preset or frame count\n");
    return 2;
  }

  rs::SimRadar radar;
  applyPreset(static_cast<uint8_t>(preset), radar.scene);
  if (preset == 0) {                                   // MANUAL: the potentiometers become options
    radar.scene.target = amp > 0.0f;
    if (amp > 0.0f) radar.scene.target_amp = amp;
    if (sigma > 0.0f) radar.scene.noise_sigma = sigma;
  }
  radar.reset(seed);

  rs::ImuProcessor imu;
  imu.configure(rs::ImuGateParams(), 100.0f, radar.scene.fs_hz);
  rs::RadarFrame f;
  for (int i = 0; i < 40; i++) {                        // settle the model IMU, like a unit that has been on for a while
    radar.step(f);
    radar.imuStep(imu);
    imu.takeFrame();
  }

  FILE* out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) {
    std::fprintf(stderr, "cannot write %s\n", out_path);
    return 1;
  }
  const rs::RadarInfo info = radar.info();
  const TrialPreset& tp = trialPreset(static_cast<uint8_t>(preset));
  std::fprintf(out, "# rs_iq_csv v1\n");
  std::fprintf(out, "# preset=%s seed=%u frames=%d\n", tp.id, seed, frames);
  std::fprintf(out, "# fs_hz=%.6f range_start_m=%.6f range_step_m=%.6f wavelength_mm=%.6f n_bins=%u\n",
               static_cast<double>(info.frame_rate_hz), static_cast<double>(info.range_start_m),
               static_cast<double>(info.range_step_m), static_cast<double>(info.wavelength_mm), static_cast<unsigned>(info.n_bins));
  std::fprintf(out, "# columns: flags,i0,q0,i1,q1,...  (flags: bit0 saturated, bit1 motion seen by an IMU mounted with the radar)\n");
  for (int n = 0; n < frames; n++) {
    radar.step(f);
    radar.imuStep(imu);
    const rs::ImuStats st = imu.takeFrame();
    unsigned flags = (f.saturated ? rs::kProtoFlagSaturated : 0u) | (st.motion ? rs::kProtoFlagMotion : 0u);
    std::fprintf(out, "%u", flags);
    for (uint8_t b = 0; b < f.n_bins; b++) std::fprintf(out, ",%d,%d", f.iq[b].i, f.iq[b].q);
    std::fprintf(out, "\n");
  }
  if (out_path) std::fclose(out);
  return 0;
}
