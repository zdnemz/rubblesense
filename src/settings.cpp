#include "settings.h"
#include <Preferences.h>
#include <math.h>
#include "radar_sim.h"

namespace {
const char* kNs = "rubblesense";

template <typename T>
T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

bool paramsValid(const rs::DspParams& p, float fs_hz, const char** why) {
  const char* w = nullptr;
  if (!(fs_hz >= 5.0f && fs_hz <= 200.0f)) w = "frame rate";
  else if (!(p.pfa_segment >= 1e-6f && p.pfa_segment <= 0.2f)) w = "pfa out of 1e-6..0.2";
  else if (!(p.pfa_confirm >= 1e-6f && p.pfa_confirm <= 0.2f)) w = "pfc out of 1e-6..0.2";
  else if (!(p.resp_lo_hz >= 0.03f && p.resp_lo_hz < p.resp_hi_hz)) w = "band: lo must be >= 0.03 and < hi";
  else if (!(p.resp_hi_hz <= 1.5f && p.resp_hi_hz <= 0.45f * fs_hz)) w = "band: hi must be <= 1.5 and <= 0.45*fs";
  else if (!(p.min_bin <= 8)) w = "minbin > 8";
  else if (!(p.cpe_guard_bins <= 4)) w = "guard > 4";
  else if (!(p.low_sens_db >= 0.5f && p.low_sens_db <= 20.0f)) w = "lowsens out of 0.5..20 dB";
  else {
    // Same geometry as rs::analyzeSegment (full-length segment): the search band plus the CFAR
    // reference cells on both sides must fit the spectral window.
    const float df = fs_hz / static_cast<float>(rs::kSegFrames);
    const int gr = p.cfar_guard + p.cfar_ref;
    int k_lo = static_cast<int>(ceilf(p.resp_lo_hz / df - 1e-4f));
    if (k_lo < 2) k_lo = 2;
    int k_hi = static_cast<int>(floorf(p.resp_hi_hz / df + 1e-4f));
    if (k_hi > static_cast<int>(rs::kSegFrames / 2) - 1) k_hi = static_cast<int>(rs::kSegFrames / 2) - 1;
    int kmin = k_lo - gr;
    if (kmin < 2) kmin = 2;
    const int cells = (k_hi + gr) - kmin + 1;
    if (k_hi - k_lo + 1 < 4) w = "band narrower than 4 FFT cells";
    else if (cells > static_cast<int>(rs::kMaxSpecCells)) w = "band too wide for the CFAR window";
  }
  if (why) *why = w;
  return w == nullptr;
}

SettingsStore::SettingsStore() : prefs_(new Preferences()) {}
SettingsStore::~SettingsStore() { delete prefs_; }

bool SettingsStore::begin() {
  open_ = prefs_->begin(kNs, false);
  if (!open_) return false;
  Preferences& p = *prefs_;
  Settings d;                                           // defaults
  s_.source = (p.getUChar("src", 0) == 1) ? SourceKind::kSerial : SourceKind::kSim;
  s_.preset = clampv<uint8_t>(p.getUChar("preset", d.preset), 0, kNumPresets - 1);
  s_.lang = (p.getUChar("lang", 0) == 1) ? 1 : 0;
  s_.sound = p.getUChar("sound", 1) != 0;
  s_.scan_counter = p.getUInt("scan_n", 0);
  s_.bl_sim_ok = p.getUChar("bl_sim_ok", 0) != 0;
  s_.bl_sim_db = p.getFloat("bl_sim", 0.0f);
  s_.bl_ser_ok = p.getUChar("bl_ser_ok", 0) != 0;
  s_.bl_ser_db = p.getFloat("bl_ser", 0.0f);
  if (!isfinite(s_.bl_sim_db)) s_.bl_sim_ok = false;
  if (!isfinite(s_.bl_ser_db)) s_.bl_ser_ok = false;

  rs::DspParams q;
  q.pfa_segment = p.getFloat("p_pfa", q.pfa_segment);
  q.pfa_confirm = p.getFloat("p_pfc", q.pfa_confirm);
  q.resp_lo_hz = p.getFloat("p_lo", q.resp_lo_hz);
  q.resp_hi_hz = p.getFloat("p_hi", q.resp_hi_hz);
  q.min_bin = p.getUChar("p_minbin", q.min_bin);
  q.cpe_enable = p.getUChar("p_cpe", q.cpe_enable ? 1 : 0) != 0;
  q.cpe_guard_bins = p.getUChar("p_guard", q.cpe_guard_bins);
  q.low_sens_db = p.getFloat("p_lowsens", q.low_sens_db);
  s_.params = paramsValid(q, 20.0f, nullptr) ? q : rs::DspParams();   // a corrupt store never reaches the DSP
  return true;
}

void SettingsStore::saveCore() {
  if (!open_) return;
  prefs_->putUChar("src", static_cast<uint8_t>(s_.source));
  prefs_->putUChar("preset", s_.preset);
  prefs_->putUChar("lang", s_.lang);
  prefs_->putUChar("sound", s_.sound ? 1 : 0);
}

void SettingsStore::saveBaselines() {
  if (!open_) return;
  prefs_->putUChar("bl_sim_ok", s_.bl_sim_ok ? 1 : 0);
  prefs_->putFloat("bl_sim", s_.bl_sim_db);
  prefs_->putUChar("bl_ser_ok", s_.bl_ser_ok ? 1 : 0);
  prefs_->putFloat("bl_ser", s_.bl_ser_db);
}

void SettingsStore::saveParams() {
  if (!open_) return;
  const rs::DspParams& q = s_.params;
  prefs_->putFloat("p_pfa", q.pfa_segment);
  prefs_->putFloat("p_pfc", q.pfa_confirm);
  prefs_->putFloat("p_lo", q.resp_lo_hz);
  prefs_->putFloat("p_hi", q.resp_hi_hz);
  prefs_->putUChar("p_minbin", q.min_bin);
  prefs_->putUChar("p_cpe", q.cpe_enable ? 1 : 0);
  prefs_->putUChar("p_guard", q.cpe_guard_bins);
  prefs_->putFloat("p_lowsens", q.low_sens_db);
}

void SettingsStore::saveCounter() {
  if (!open_) return;
  prefs_->putUInt("scan_n", s_.scan_counter);
}

void SettingsStore::factoryReset() {
  if (open_) prefs_->clear();
  s_ = Settings();
}
