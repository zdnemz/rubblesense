// Scan-level Monte-Carlo study of the whole decision chain on the simulated scene.
//
//   montecarlo [--quick] [--only=E1,E2,...] [--out=DIR]
//
// Writes one CSV per experiment into DIR (default: docs/results) and prints a Markdown
// summary. EVERYTHING HERE IS SIMULATION: white Gaussian noise, ideal point reflectors,
// a perfectly sinusoidal platform vibration. It calibrates and regression-tests the
// algorithm; it does not predict performance in real rubble (see docs/ARSITEKTUR.md).
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include "sim_helpers.h"

using namespace rs;

namespace {

struct Rec {
  Verdict v;
  float bpm;
  uint8_t bin;
  bool vib;
  uint8_t n_valid, n_segments;
  uint8_t seg1_det;
  float floor_db;
  float floor_delta;
  InvalidReason inv;
};

constexpr int kNV = 7;   // number of Verdict values

std::vector<Rec> runBatch(int n, uint32_t seed0, const DspParams& p, const std::function<SimScene(int)>& make) {
  std::vector<Rec> out(static_cast<size_t>(n));
  const unsigned nt = std::max(1u, std::min(4u, std::thread::hardware_concurrency()));
  std::vector<std::thread> th;
  for (unsigned t = 0; t < nt; t++) {
    th.emplace_back([&, t]() {
      ScanCtx ctx;
      for (int i = static_cast<int>(t); i < n; i += static_cast<int>(nt)) {
        const SimScene sc = make(i);
        const ScanOutcome o = runScan(ctx, sc, seed0 + static_cast<uint32_t>(i), p);
        Rec r{};
        r.v = o.verdict; r.bpm = o.bpm; r.bin = o.bin; r.vib = o.vibration;
        r.n_valid = o.n_valid; r.n_segments = o.n_segments; r.inv = o.last_invalid;
        r.floor_db = o.floor_db; r.floor_delta = o.floor_delta_db;
        r.seg1_det = ctx.sess.logCount() > 0 ? ctx.sess.log(0).n_det : 0;
        out[static_cast<size_t>(i)] = r;
      }
    });
  }
  for (std::thread& x : th) x.join();
  return out;
}

// Exact (Clopper-Pearson) 95 % interval for k events in n trials.
double binomCdf(int k, int n, double p) {
  if (k < 0) return 0.0;
  if (p <= 0.0) return 1.0;
  if (p >= 1.0) return k >= n ? 1.0 : 0.0;
  const double lp = std::log(p), lq = std::log1p(-p), lgn = std::lgamma(n + 1.0);
  double s = 0.0;
  for (int i = 0; i <= k; i++)
    s += std::exp(lgn - std::lgamma(i + 1.0) - std::lgamma(n - i + 1.0) + i * lp + (n - i) * lq);
  return std::min(1.0, s);
}
void cpInterval(int k, int n, double& lo, double& hi) {
  if (n <= 0) { lo = 0; hi = 1; return; }
  if (k >= n) hi = 1.0;
  else {
    double a = 0.0, b = 1.0;
    for (int it = 0; it < 60; it++) { const double m = 0.5 * (a + b); if (binomCdf(k, n, m) > 0.025) a = m; else b = m; }
    hi = 0.5 * (a + b);
  }
  if (k <= 0) lo = 0.0;
  else {
    double a = 0.0, b = 1.0;
    for (int it = 0; it < 60; it++) { const double m = 0.5 * (a + b); if (binomCdf(k - 1, n, m) > 0.975) a = m; else b = m; }
    lo = 0.5 * (a + b);
  }
}

struct Summary {
  int n = 0;
  int cnt[kNV] = {0};
  int ok = 0;              // target present: candidate at the right rate and place
  int seg1_alarm = 0;      // a CFAR line in the first segment
  int vib = 0;
  double dur_s = 0.0;
  std::vector<float> bpm_err;
};

Summary summarise(const std::vector<Rec>& r, bool target, float truth_bpm, int truth_bin) {
  Summary s;
  s.n = static_cast<int>(r.size());
  for (const Rec& x : r) {
    s.cnt[static_cast<int>(x.v)]++;
    if (x.seg1_det > 0) s.seg1_alarm++;
    if (x.vib) s.vib++;
    s.dur_s += 25.6 * x.n_segments;
    if (target && x.v == Verdict::kCandidate) {
      const float e = std::fabs(x.bpm - truth_bpm);
      if (e <= 2.0f && std::abs(static_cast<int>(x.bin) - truth_bin) <= 1) s.ok++;
      s.bpm_err.push_back(e);
    }
  }
  return s;
}

double frac(int k, int n) { return n ? static_cast<double>(k) / n : 0.0; }
int C(const Summary& s, Verdict v) { return s.cnt[static_cast<int>(v)]; }

std::string pct(double x, int prec = 1) { char b[32]; std::snprintf(b, sizeof b, "%.*f", prec, 100.0 * x); return b; }

std::string ci(int k, int n) {
  double lo, hi; cpInterval(k, n, lo, hi);
  char b[64]; std::snprintf(b, sizeof b, "%s (%s-%s)", pct(frac(k, n), 2).c_str(), pct(lo, 2).c_str(), pct(hi, 2).c_str());
  return b;
}

FILE* openCsv(const std::string& dir, const char* name, const char* header) {
  const std::string path = dir + "/" + name;
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) { std::fprintf(stderr, "cannot write %s\n", path.c_str()); return nullptr; }
  std::fprintf(f, "%s\n", header);
  return f;
}

void csvRow(FILE* f, const char* label, double param, const Summary& s) {
  if (!f) return;
  std::fprintf(f, "%s,%g,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", label, param, s.n, C(s, Verdict::kCandidate),
               C(s, Verdict::kPossible), C(s, Verdict::kNoPattern), C(s, Verdict::kLowSensitivity),
               C(s, Verdict::kConfounded), C(s, Verdict::kInvalid), s.ok, s.vib);
}
const char* kCsvHeader = "case,param,n,candidate,possible,no_pattern,low_sensitivity,confounded,invalid,candidate_correct,vibration_flagged";

float median(std::vector<float> v) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

struct Cfg { bool quick = false; std::string only, out = "docs/results"; int scale(int n) const { return quick ? std::max(20, n / 10) : n; } };
bool want(const Cfg& c, const char* id) { return c.only.empty() || c.only.find(id) != std::string::npos; }

// ---------------------------------------------------------------------------
void e1FalseAlarms(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(20000);
  std::printf("\n### E1 - Empty scene (no person), %d scans\n\n", n);
  SimScene sc;
  const std::vector<Rec> r = runBatch(n, 1000000, p, [&](int) { return sc; });
  const Summary s = summarise(r, false, 0, 0);
  std::printf("| Verdict | Scans | Share (95 %% CI) |\n|---|---|---|\n");
  const Verdict order[] = {Verdict::kCandidate, Verdict::kPossible, Verdict::kConfounded, Verdict::kLowSensitivity,
                           Verdict::kInvalid, Verdict::kNoPattern};
  for (Verdict v : order) std::printf("| %s | %d | %s %% |\n", verdictName(v), C(s, v), ci(C(s, v), n).c_str());
  std::printf("\nFirst-segment CFAR alarm rate (design: pfa_segment = %.1f %%): %s %%\n", 100.0 * static_cast<double>(p.pfa_segment), ci(s.seg1_alarm, n).c_str());
  std::printf("Mean scan duration: %.1f s\n", s.dur_s / n);
  FILE* f = openCsv(cfg.out, "e1_empty.csv", kCsvHeader);
  csvRow(f, "empty", 0, s);
  if (f) std::fclose(f);
}

void e2DetectionVsAmplitude(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(400);
  std::printf("\n### E2 - Person breathing at 16.8 /min, detection vs. reflection amplitude (%d scans each)\n\n", n);
  std::printf("| amplitude (counts) | nominal tone SNR (dB) | CANDIDATE correct | CANDIDATE any | POSSIBLE | NO_PATTERN | median rate error (/min) | mean scan (s) |\n|---|---|---|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e2_detection.csv", kCsvHeader);
  const float amps[] = {6, 9, 12, 15, 18, 21, 24, 30, 40, 60};
  uint32_t seed = 2000000;
  for (float a : amps) {
    SimScene sc; sc.target = true; sc.target_amp = a;
    const std::vector<Rec> r = runBatch(n, seed, p, [&](int) { return sc; });
    seed += 100000;
    const Summary s = summarise(r, true, 60.0f * sc.breath_hz, sc.target_bin);
    std::printf("| %.0f | %.1f | %s %% | %s %% | %s %% | %s %% | %.2f | %.0f |\n", static_cast<double>(a),
                static_cast<double>(SimRadar::nominalToneSnrDb(sc, kSegFrames)), ci(s.ok, n).c_str(),
                pct(frac(C(s, Verdict::kCandidate), n)).c_str(), pct(frac(C(s, Verdict::kPossible), n)).c_str(),
                pct(frac(C(s, Verdict::kNoPattern), n)).c_str(), static_cast<double>(median(s.bpm_err)), s.dur_s / n);
    csvRow(f, "amp", static_cast<double>(a), s);
  }
  if (f) std::fclose(f);
}

void e3Vibration(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(2000);
  std::printf("\n### E3 - Platform micro-vibration the IMU cannot see, NO person (%d scans each, random frequency 0.15-0.75 Hz)\n\n", n);
  std::printf("| vibration (mm) | compensation | CANDIDATE | POSSIBLE | CONFOUNDED | NO_PATTERN | vibration notice shown |\n|---|---|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e3_vibration.csv", kCsvHeader);
  const float lv[] = {0.0f, 0.05f, 0.1f, 0.2f, 0.4f};
  uint32_t seed = 3000000;
  for (int comp = 1; comp >= 0; comp--) {
    DspParams q = p; q.cpe_enable = comp != 0;
    for (float mm : lv) {
      const std::vector<Rec> r = runBatch(n, seed, q, [&](int i) {
        SimScene sc; Rng g(static_cast<uint64_t>(i) * 7919u + 13u);
        sc.vib_mm = mm; sc.vib_hz = 0.15f + 0.6f * g.uniform();
        return sc;
      });
      seed += 100000;
      const Summary s = summarise(r, false, 0, 0);
      std::printf("| %.2f | %s | %s %% | %s %% | %s %% | %s %% | %s %% |\n", static_cast<double>(mm), comp ? "on" : "off",
                  ci(C(s, Verdict::kCandidate), n).c_str(), pct(frac(C(s, Verdict::kPossible), n)).c_str(),
                  pct(frac(C(s, Verdict::kConfounded), n)).c_str(), pct(frac(C(s, Verdict::kNoPattern), n)).c_str(),
                  pct(frac(s.vib, n)).c_str());
      csvRow(f, comp ? "vib_comp_on" : "vib_comp_off", static_cast<double>(mm), s);
    }
  }
  if (f) std::fclose(f);
}

void e4PersonPlusVibration(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(500);
  std::printf("\n### E4 - Person (amplitude 30) AND platform vibration of 0.2 mm (%d scans each)\n\n", n);
  std::printf("| vibration frequency | CANDIDATE correct | POSSIBLE | CONFOUNDED | NO_PATTERN | vibration notice |\n|---|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e4_person_vibration.csv", kCsvHeader);
  const float hz[] = {0.0f, 0.15f, 0.28f, 0.45f, 0.70f};
  uint32_t seed = 4000000;
  for (float v : hz) {
    SimScene sc; sc.target = true; sc.target_amp = 30.0f; sc.vib_mm = (v > 0.0f) ? 0.2f : 0.0f; sc.vib_hz = v;
    const std::vector<Rec> r = runBatch(n, seed, p, [&](int) { return sc; });
    seed += 100000;
    const Summary s = summarise(r, true, 60.0f * sc.breath_hz, sc.target_bin);
    std::printf("| %s | %s %% | %s %% | %s %% | %s %% | %s %% |\n", v > 0.0f ? (std::to_string(v).substr(0, 4) + " Hz").c_str() : "none",
                ci(s.ok, n).c_str(), pct(frac(C(s, Verdict::kPossible), n)).c_str(),
                pct(frac(C(s, Verdict::kConfounded), n)).c_str(), pct(frac(C(s, Verdict::kNoPattern), n)).c_str(),
                pct(frac(s.vib, n)).c_str());
    csvRow(f, "person_vib", static_cast<double>(v), s);
  }
  if (f) std::fclose(f);
}

void e5Bumps(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(500);
  std::printf("\n### E5 - Impulsive bumps of the sensor (rate 0.3 /s), with and without a person (%d scans each)\n\n", n);
  std::printf("| bump size (mm) | person | INVALID_SCAN | CANDIDATE correct | CANDIDATE wrong | POSSIBLE | CONFOUNDED | NO_PATTERN |\n|---|---|---|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e5_bumps.csv", kCsvHeader);
  const float mm[] = {0.05f, 0.1f, 0.3f, 1.0f};
  uint32_t seed = 5000000;
  for (int person = 0; person < 2; person++) {
    for (float b : mm) {
      SimScene sc; sc.bump_rate_hz = 0.3f; sc.bump_mm = b; sc.target = person != 0; sc.target_amp = 30.0f;
      const std::vector<Rec> r = runBatch(n, seed, p, [&](int) { return sc; });
      seed += 100000;
      const Summary s = summarise(r, person != 0, 60.0f * sc.breath_hz, sc.target_bin);
      const int cand = C(s, Verdict::kCandidate);
      std::printf("| %.2f | %s | %s %% | %s %% | %s %% | %s %% | %s %% | %s %% |\n", static_cast<double>(b), person ? "yes" : "no",
                  pct(frac(C(s, Verdict::kInvalid), n)).c_str(), person ? pct(frac(s.ok, n)).c_str() : "-",
                  ci(person ? cand - s.ok : cand, n).c_str(), pct(frac(C(s, Verdict::kPossible), n)).c_str(),
                  pct(frac(C(s, Verdict::kConfounded), n)).c_str(), pct(frac(C(s, Verdict::kNoPattern), n)).c_str());
      csvRow(f, person ? "bump_person" : "bump_empty", static_cast<double>(b), s);
    }
  }
  if (f) std::fclose(f);
}

void e6Sensitivity(const Cfg& cfg, const DspParams& p0) {
  const int n = cfg.scale(500);
  DspParams p = p0;
  p.have_baseline = true;
  p.baseline_floor_db = 10.0f * log10f(2.0f * 8.0f * 8.0f);     // floor of the sigma = 8 bench setup
  std::printf("\n### E6 - Noisier-than-bench conditions: the negative result must lose its meaning (%d scans each)\n\n", n);
  std::printf("Baseline floor %.1f dB, LOW_SENSITIVITY above +%.0f dB. Empty scene and a person of amplitude 30.\n\n",
              static_cast<double>(p.baseline_floor_db), static_cast<double>(p.low_sens_db));
  std::printf("| noise sigma (counts) | floor over baseline (dB) | empty: LOW_SENSITIVITY | empty: NO_PATTERN | person: CANDIDATE correct | person: LOW_SENSITIVITY | person: NO_PATTERN |\n|---|---|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e6_sensitivity.csv", kCsvHeader);
  const float sg[] = {8.0f, 11.0f, 16.0f, 24.0f, 32.0f};
  uint32_t seed = 6000000;
  for (float s0 : sg) {
    SimScene e; e.noise_sigma = s0;
    SimScene t = e; t.target = true; t.target_amp = 30.0f;
    const std::vector<Rec> re = runBatch(n, seed, p, [&](int) { return e; });
    const std::vector<Rec> rt = runBatch(n, seed + 50000, p, [&](int) { return t; });
    seed += 100000;
    const Summary se = summarise(re, false, 0, 0), st = summarise(rt, true, 60.0f * t.breath_hz, t.target_bin);
    double dsum = 0; for (const Rec& x : re) dsum += static_cast<double>(x.floor_delta);
    std::printf("| %.0f | %+.1f | %s %% | %s %% | %s %% | %s %% | %s %% |\n", static_cast<double>(s0), dsum / n,
                pct(frac(C(se, Verdict::kLowSensitivity), n)).c_str(), pct(frac(C(se, Verdict::kNoPattern), n)).c_str(),
                pct(frac(st.ok, n)).c_str(), pct(frac(C(st, Verdict::kLowSensitivity), n)).c_str(),
                pct(frac(C(st, Verdict::kNoPattern), n)).c_str());
    csvRow(f, "sens_empty", static_cast<double>(s0), se);
    csvRow(f, "sens_person", static_cast<double>(s0), st);
  }
  if (f) std::fclose(f);
}

void e7RateAccuracy(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(300);
  std::printf("\n### E7 - Rate and range accuracy of confirmed candidates (amplitude 40, %d scans per rate)\n\n", n);
  std::printf("| true rate (/min) | CANDIDATE correct | median rate error (/min) | 90th pct error (/min) | bin within +-1 |\n|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e7_rate.csv", kCsvHeader);
  const float rates[] = {7.0f, 10.0f, 16.8f, 24.0f, 36.0f, 45.0f};
  uint32_t seed = 7000000;
  for (float bpm : rates) {
    SimScene sc; sc.target = true; sc.target_amp = 40.0f; sc.breath_hz = bpm / 60.0f;
    const std::vector<Rec> r = runBatch(n, seed, p, [&](int) { return sc; });
    seed += 100000;
    const Summary s = summarise(r, true, bpm, sc.target_bin);
    std::vector<float> e = s.bpm_err;
    std::sort(e.begin(), e.end());
    const float p90 = e.empty() ? NAN : e[static_cast<size_t>(0.9 * static_cast<double>(e.size() - 1))];
    int binok = 0;
    for (const Rec& x : r) if (x.v == Verdict::kCandidate && std::abs(static_cast<int>(x.bin) - sc.target_bin) <= 1) binok++;
    std::printf("| %.1f | %s %% | %.2f | %.2f | %s %% |\n", static_cast<double>(bpm), pct(frac(s.ok, n)).c_str(),
                static_cast<double>(median(s.bpm_err)), static_cast<double>(p90), pct(frac(binok, std::max(1, C(s, Verdict::kCandidate)))).c_str());
    csvRow(f, "rate", static_cast<double>(bpm), s);
  }
  if (f) std::fclose(f);
}

void e8Mismatch(const Cfg& cfg, const DspParams& p) {
  const int n = cfg.scale(1000);
  std::printf("\n### E8 - Vibration that is NOT perfectly common to all range bins (%d scans each, 0.2 mm, random frequency 0.15-0.75 Hz)\n\n", n);
  std::printf("Each bin sees the platform phase scaled by (1 + m*g), g ~ N(0,1). m = 0 is the idealised simulator; real scenes are not.\n\n");
  std::printf("| mismatch m | person | CANDIDATE correct | CANDIDATE false | POSSIBLE | CONFOUNDED | NO_PATTERN |\n|---|---|---|---|---|---|---|\n");
  FILE* f = openCsv(cfg.out, "e8_mismatch.csv", kCsvHeader);
  const float mm[] = {0.0f, 0.1f, 0.3f, 0.5f};
  uint32_t seed = 8000000;
  for (int person = 0; person < 2; person++) {
    for (float m : mm) {
      const std::vector<Rec> r = runBatch(n, seed, p, [&](int i) {
        SimScene sc; Rng g(static_cast<uint64_t>(i) * 104729u + 7u);
        sc.vib_mm = 0.2f; sc.vib_hz = 0.15f + 0.6f * g.uniform(); sc.cpe_mismatch = m;
        sc.target = person != 0; sc.target_amp = 30.0f;
        return sc;
      });
      seed += 100000;
      const Summary s = summarise(r, person != 0, 16.8f, 9);
      const int cand = C(s, Verdict::kCandidate);
      std::printf("| %.1f | %s | %s | %s %% | %s %% | %s %% | %s %% |\n", static_cast<double>(m), person ? "yes" : "no",
                  person ? (pct(frac(s.ok, n)) + " %").c_str() : "-", ci(person ? cand - s.ok : cand, n).c_str(),
                  pct(frac(C(s, Verdict::kPossible), n)).c_str(), pct(frac(C(s, Verdict::kConfounded), n)).c_str(),
                  pct(frac(C(s, Verdict::kNoPattern), n)).c_str());
      csvRow(f, person ? "mismatch_person" : "mismatch_empty", static_cast<double>(m), s);
    }
  }
  if (f) std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
  Cfg cfg;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--quick")) cfg.quick = true;
    else if (!std::strncmp(argv[i], "--only=", 7)) cfg.only = argv[i] + 7;
    else if (!std::strncmp(argv[i], "--out=", 6)) cfg.out = argv[i] + 6;
    else { std::fprintf(stderr, "usage: %s [--quick] [--only=E1,E2,...] [--out=DIR]\n", argv[0]); return 2; }
  }
  const DspParams p;
  std::printf("# RubbleSense Monte-Carlo (SIMULATION ONLY)\n\n");
  std::printf("Parameters: pfa_segment=%.3g pfa_confirm=%.3g band=%.2f-%.2f Hz cfar=%d+%d guard/ref cpe=%s\n",
              static_cast<double>(p.pfa_segment), static_cast<double>(p.pfa_confirm), static_cast<double>(p.resp_lo_hz),
              static_cast<double>(p.resp_hi_hz), p.cfar_guard, p.cfar_ref, p.cpe_enable ? "on" : "off");
  std::printf("Mode: %s\n", cfg.quick ? "quick (reduced trial counts)" : "full");
  if (want(cfg, "E1")) e1FalseAlarms(cfg, p);
  if (want(cfg, "E2")) e2DetectionVsAmplitude(cfg, p);
  if (want(cfg, "E3")) e3Vibration(cfg, p);
  if (want(cfg, "E4")) e4PersonPlusVibration(cfg, p);
  if (want(cfg, "E5")) e5Bumps(cfg, p);
  if (want(cfg, "E6")) e6Sensitivity(cfg, p);
  if (want(cfg, "E7")) e7RateAccuracy(cfg, p);
  if (want(cfg, "E8")) e8Mismatch(cfg, p);
  return 0;
}
