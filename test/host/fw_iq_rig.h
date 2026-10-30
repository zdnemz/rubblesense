// Helpers shared by the serial I/Q test suites: the device wired to an external front-end model.
//
// The front-end speaks on the dedicated UART2 (default build) or on UART0 together with the console
// (-DRS_IQ_ON_UART0=1: the hardware-in-the-loop / Wokwi replay set-up). Everything else is identical.
#pragma once
#include <string>
#include "fw_rig.h"
#include "fw_tests.h"
#include "rs_build.h"

namespace iq {

#if RS_IQ_ON_UART0
inline HardwareSerial& port() { return Serial; }
constexpr uint32_t kBaud = RS_SERIAL_BAUD;
inline const char* buildName() { return "I/Q on UART0 (USB, shared with the console)"; }
#else
inline HardwareSerial& port() { return Serial2; }
constexpr uint32_t kBaud = RS_IQ_BAUD;
inline const char* buildName() { return "I/Q on UART2 (dedicated)"; }
#endif

inline rs::SimScene person(float amp = 30.0f, float sigma = 8.0f) {
  rs::SimScene s;
  s.target = true;
  s.target_amp = amp;
  s.noise_sigma = sigma;
  return s;
}

inline rs::SimScene empty(float sigma = 8.0f) {
  rs::SimScene s;
  s.target = false;
  s.noise_sigma = sigma;
  return s;
}

// All-zero I/Q: a front-end whose ADC is dead or disconnected.
inline rs::SimScene silent() {
  rs::SimScene s;
  s.target = false;
  s.leak_amp = 0.0f;
  s.wall_amp = 0.0f;
  s.clutter_amp = 0.0f;
  s.noise_sigma = 0.0f;
  return s;
}

// The device plus an external radar front-end connected to it.
struct Rig2 {
  Rig       r;
  FrontEnd  fe;

  explicit Rig2(bool announce = true) { fe.announce = announce; }

  void boot() {
    r.boot();
    fe.attach(&port(), kBaud);
    r.tickers.push_back([this](uint64_t t) { fe.tick(t); });
  }

  // The device loses power and restarts; the front-end does not notice and goes on streaming.
  void powerCycle() {
    r.powerCycle();
    fe.rebase(stub::nowUs());
  }

  // The front-end starts streaming a scene (and announcing itself, if enabled).
  void connect(const rs::SimScene& sc, uint32_t seed) { fe.start(sc, seed, stub::nowUs()); }

  bool waitReady(uint32_t ms = 3000) {
    return r.run_until([this] { return r.app->sourceReady(); }, ms);
  }

  // One console line. With I/Q on the console UART the line must land between two packets (what a
  // careful PC tool does) and starts with a newline that flushes whatever the console had collected.
  std::string cmd(const char* line, uint32_t wait_ms = 150) {
#if RS_IQ_ON_UART0
    r.takeSerial();
    fe.queueText(std::string("\n") + line + "\n");
    r.run_ms(wait_ms);
    return r.takeSerial();
#else
    return r.say(line, wait_ms);
#endif
  }

  // Selects the SERIAL source and brings a front-end up.
  bool bringUp(const rs::SimScene& sc, uint32_t seed) {
    r.run_ms(300);
    if (!replyOk(cmd("src serial"))) return false;
    connect(sc, seed);
    return waitReady(2500);
  }

  // Result, message and menu screens: go back to idle (the white button closes / leaves them).
  void dismiss() {
    for (int i = 0; i < 4 && r.state() != App::St::kIdle; i++) {
      r.press(PIN_BTN_MENU);
      r.run_ms(100);
    }
  }
};

struct Tally {
  int  n = 0;
  int  by[7] = {0, 0, 0, 0, 0, 0, 0};          // indexed by rs::Verdict
  int  reason[16] = {0};                       // invalid scans by rs::InvalidReason
  long lost = 0, crc = 0, timing = 0;         // frames lost / CRC errors / frames flagged for inconsistent timestamps
  int  c(rs::Verdict v) const { return by[static_cast<int>(v)]; }
  int  why(rs::InvalidReason x) const { return reason[static_cast<int>(x)]; }
};

// `n` complete scans, each on a freshly started front-end scene ("the operator moved the sensor").
inline Tally runScans(Rig2& t, const rs::SimScene& sc, int n, uint32_t seed0, bool invariants = true, bool timing_clean = true) {
  Tally k;
  for (int i = 0; i < n; i++) {
    t.connect(sc, seed0 + static_cast<uint32_t>(i));
    t.r.run_ms(1100);
    const bool ok = t.r.scanByButton(900000);
    CHECK(ok);
    if (!ok) break;
    const rs::ScanOutcome o = t.r.app->lastOutcome();
    if (invariants) checkScanInvariants(t.r, o, sc.n_bins);
    k.n++;
    k.by[static_cast<int>(o.verdict)]++;
    if (o.verdict == rs::Verdict::kInvalid) k.reason[static_cast<int>(o.last_invalid)]++;
    const std::string j = t.r.app->lastJson();
    const long lost = jsonInt(j, "lost_frames"), crc = jsonInt(j, "crc_errors"), tim = jsonInt(j, "timing_flagged");
    if (lost > 0) k.lost += lost;
    if (crc > 0) k.crc += crc;
    if (tim > 0) k.timing += tim;
    if (timing_clean) CHECK(tim == 0);                // link damage of any kind must not look like a time-base problem
  }
  return k;
}

inline void report(const char* label, const Tally& k) {
  char line[512];
  snprintf(line, sizeof(line), "| %s | %d | %d | %d | %d | %d | %d | %d | %ld | %ld | %ld |", label, k.n,
           k.c(rs::Verdict::kCandidate), k.c(rs::Verdict::kPossible), k.c(rs::Verdict::kConfounded),
           k.c(rs::Verdict::kLowSensitivity), k.c(rs::Verdict::kInvalid), k.c(rs::Verdict::kNoPattern), k.lost, k.crc, k.timing);
  g_report.push_back(line);
  std::printf("  %-52s n=%-2d CAND %-2d POSS %-2d CONF %-2d LOWS %-2d INV %-2d NOPAT %-2d lost %-4ld crc %-3ld tim %-4ld\n", label, k.n,
              k.c(rs::Verdict::kCandidate), k.c(rs::Verdict::kPossible), k.c(rs::Verdict::kConfounded),
              k.c(rs::Verdict::kLowSensitivity), k.c(rs::Verdict::kInvalid), k.c(rs::Verdict::kNoPattern), k.lost, k.crc, k.timing);
}

inline void reportHeader(const std::string& title, const std::string& intro) {
  g_report.push_back("");
  g_report.push_back(title);
  g_report.push_back("");
  g_report.push_back(intro);
  g_report.push_back("");
  g_report.push_back("| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |");
  g_report.push_back("|---|---|---|---|---|---|---|---|---|---|---|");
}

// Value after "key=" in a console status line, -1 if absent.
inline long statusField(const std::string& status, const char* key) {
  const std::string k = std::string(key) + "=";
  const size_t p = status.find(k);
  if (p == std::string::npos) return -1;
  return std::strtol(status.c_str() + p + k.size(), nullptr, 10);
}

}  // namespace iq
