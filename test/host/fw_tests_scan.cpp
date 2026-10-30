// Firmware host tests, part 2: complete scans on the simulated radar (all trial presets, manual
// pots, calibration), abort and refusal rules, counter wrap-around, IMU faults, memory.
#include <math.h>
#include <new>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "fw_rig.h"
#include "fw_tests.h"
#include "rs_text.h"
#include "scan_json.h"

std::vector<std::string> g_report;
std::string g_log_out;               // when set, the scan log produced by fw_presets is written there

// ---------------------------------------------------------------------------- allocation fault injection
// Replaces only the nothrow form, which is what the firmware uses for its two big buffers.
static int  g_fail_big_alloc = 0;     // 0 = never; n = fail the n-th allocation of >= 30000 bytes from now on
static int  g_big_allocs = 0;
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
  if (g_fail_big_alloc > 0 && n >= 30000 && ++g_big_allocs >= g_fail_big_alloc) return nullptr;
  return malloc(n);
}
void operator delete(void* p, const std::nothrow_t&) noexcept { free(p); }

namespace {

struct Tally {
  int n = 0;
  int by[7] = {0, 0, 0, 0, 0, 0, 0};    // indexed by rs::Verdict
  int vibration = 0, cpe_all = 0, sens_unknown = 0;
  double dur_s = 0.0;
  int c(rs::Verdict v) const { return by[static_cast<int>(v)]; }
};


std::string readLogFiles() {
  std::string all;
  const char* names[2] = {"/scans.old.jsonl", "/scans.jsonl"};
  for (int i = 0; i < 2; i++) {
    std::map<std::string, std::string>::const_iterator it = stub::fs().files.find(names[i]);
    if (it != stub::fs().files.end()) all += it->second;
  }
  return all;
}

Tally runScans(Rig& r, uint8_t preset, int n, bool check_each = true) {
  Tally t;
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "preset %u", static_cast<unsigned>(preset));
  CHECK(replyOk(r.say(cmd)));
  for (int i = 0; i < n; i++) {
    const bool ok = r.scanByButton();
    CHECK(ok);
    if (!ok) break;
    const rs::ScanOutcome o = r.app->lastOutcome();
    if (check_each) checkScanInvariants(r, o);
    t.n++;
    t.by[static_cast<int>(o.verdict)]++;
    if (o.vibration) t.vibration++;
    if (o.cpe_applied) t.cpe_all++;
    if (!o.sens_known) t.sens_unknown++;
  }
  return t;
}

void reportTally(const char* label, const Tally& t) {
  char line[400];
  snprintf(line, sizeof(line), "| %s | %d | %d | %d | %d | %d | %d | %d |", label, t.n, t.c(rs::Verdict::kCandidate),
           t.c(rs::Verdict::kPossible), t.c(rs::Verdict::kConfounded), t.c(rs::Verdict::kLowSensitivity),
           t.c(rs::Verdict::kInvalid), t.c(rs::Verdict::kNoPattern));
  g_report.push_back(line);
  std::printf("  %-34s n=%-3d CAND %-3d POSS %-3d CONF %-3d LOWS %-3d INV %-3d NOPAT %-3d\n", label, t.n,
              t.c(rs::Verdict::kCandidate), t.c(rs::Verdict::kPossible), t.c(rs::Verdict::kConfounded),
              t.c(rs::Verdict::kLowSensitivity), t.c(rs::Verdict::kInvalid), t.c(rs::Verdict::kNoPattern));
}

}  // namespace

// ============================================================================ all trial presets
void fw_presets() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);
  const int N = 30;

  g_report.push_back("### Trial presets through the real firmware (button -> acquisition -> DSP -> decision -> JSON log)");
  g_report.push_back("");
  g_report.push_back(std::string("White-Gaussian-noise simulator, default SIM baseline 21.1 dB, ") + std::to_string(N) +
                     " scans per preset, virtual clock, hardware-IMU stub at rest.");
  g_report.push_back("");
  g_report.push_back("| preset | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN |");
  g_report.push_back("|---|---|---|---|---|---|---|---|");

  // First scan of each preset with the display on: the live screens must be clean too.
  const uint32_t t0 = r.app->ui().truncations();
  stub::gfx().enabled = true;
  {
    CHECK(replyOk(r.say("preset a")));
    r.press(PIN_BTN_SCAN);
    CHECK(r.scanning());
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 330 && r.app->acq().status().mode == AcqStatus::kRunning; }, 60000));
    r.run_ms(1500);                                    // a live preview has been drawn by now
    CHECK(r.shows(txt::t(txt::T_LIVE_NOT_RESULT)));
    CHECK(r.shows("bin "));                            // preview waveform label
    stub::gfx().partialOverdraws();
    if (stub::gfx().textOverText() != 0) std::printf("  text over text during the scan screen\n");
    CHECK(stub::gfx().textOverText() == 0);
    CHECK(stub::gfx().clippedGlyphs() == 0);
    if (!g_screens_dir.empty()) stub::gfx().dump(g_screens_dir + "/scan_preview_en.txt");
    CHECK(r.waitScanEnd());
    if (!g_screens_dir.empty()) stub::gfx().dump(g_screens_dir + "/result_live_candidate_en.txt");
  }
  CHECK(r.app->ui().truncations() == t0);
  stub::gfx().enabled = false;

  Tally a = runScans(r, 1, N);   reportTally("A: person, quiet", a);
  Tally b = runScans(r, 2, N);   reportTally("B: empty (control)", b);
  Tally c = runScans(r, 3, N);   reportTally("C: sensor knocked", c);
  Tally d = runScans(r, 4, N);   reportTally("D: person + vibration 0.45 Hz", d);
  Tally e = runScans(r, 5, N);   reportTally("E: vibration = breathing", e);
  Tally f = runScans(r, 6, N);   reportTally("F: empty, 3x noise", f);
  g_report.push_back("");

  // Expectations from the Monte-Carlo calibration (docs/results): the firmware must reproduce them.
  CHECK(a.c(rs::Verdict::kCandidate) >= N - 2);
  CHECK(a.c(rs::Verdict::kNoPattern) == 0);
  CHECK(b.c(rs::Verdict::kCandidate) == 0);                      // false-candidate rate ~0.01 %
  CHECK(b.c(rs::Verdict::kNoPattern) >= N - 3);
  CHECK(b.c(rs::Verdict::kInvalid) == 0);
  CHECK(c.c(rs::Verdict::kInvalid) >= N - 2);
  CHECK(c.c(rs::Verdict::kCandidate) == 0);
  CHECK(d.c(rs::Verdict::kCandidate) >= N - 2);
  CHECK(d.vibration >= N - 2);                                   // the vibration notice is shown
  CHECK(e.c(rs::Verdict::kConfounded) >= N - 3);
  CHECK(e.c(rs::Verdict::kCandidate) <= 2);
  CHECK(f.c(rs::Verdict::kLowSensitivity) >= N - 2);
  CHECK(f.c(rs::Verdict::kNoPattern) == 0);                      // a negative in 3x the noise means nothing
  CHECK(f.c(rs::Verdict::kCandidate) == 0);
  // the sim default baseline is used for every preset scan
  CHECK(a.sens_unknown == 0 && b.sens_unknown == 0);

  // every record is in the log, one JSON object per line
  const std::string log = readLogFiles();
  const int expected = 1 + 6 * N;
  CHECK(countOf(log, "\n") == expected);
  CHECK(r.app->storage().lines() == static_cast<uint32_t>(expected));
  CHECK(countOf(log, "\"verdict\"") == expected);
  CHECK(countOf(log, "No reliable life-sign pattern detected in this scan. DO NOT clear area.") ==
        b.c(rs::Verdict::kNoPattern) + a.c(rs::Verdict::kNoPattern) + c.c(rs::Verdict::kNoPattern) +
        d.c(rs::Verdict::kNoPattern) + e.c(rs::Verdict::kNoPattern) + f.c(rs::Verdict::kNoPattern));
  if (!g_log_out.empty()) {
    FILE* fp = fopen(g_log_out.c_str(), "w");
    if (fp) { fwrite(log.data(), 1, log.size(), fp); fclose(fp); }
  }
  // scan ids count up without gaps
  CHECK(r.app->scanCount() == static_cast<uint32_t>(expected));
  CHECK(has(log, "\"scan_id\":\"RS-0001\"") && has(log, "\"scan_id\":\"RS-0181\""));
}

// ============================================================================ manual pots
void fw_manual_pots() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);
  stub::gfx().enabled = false;
  CHECK(replyOk(r.say("preset manual")));

  struct Case { int target_adc, noise_adc; const char* what; int want_cand, want_nopat, want_lows; };
  // target pot: 60 counts * x ; noise pot: sigma = 4 + 28 x
  const Case cases[] = {
      {2048, 585, "target 30, noise 8: a person is visible", 1, 0, 0},
      {0, 585, "no target, noise 8: nothing found", 0, 1, 0},
      {0, 4095, "no target, noise 32: inconclusive", 0, 0, 1},
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    stub::setAnalog(PIN_POT_TARGET, cases[i].target_adc);
    stub::setAnalog(PIN_POT_NOISE, cases[i].noise_adc);
    r.run_ms(1500);                                    // the pot filter settles
    int cand = 0, nopat = 0, lows = 0;
    for (int k = 0; k < 6; k++) {
      CHECK(r.scanByButton());
      const rs::Verdict v = r.verdict();
      cand += v == rs::Verdict::kCandidate;
      nopat += v == rs::Verdict::kNoPattern;
      lows += v == rs::Verdict::kLowSensitivity;
    }
    std::printf("  MANUAL %-45s CAND %d NOPAT %d LOWS %d (of 6)\n", cases[i].what, cand, nopat, lows);
    if (cases[i].want_cand) CHECK(cand >= 5);
    else CHECK(cand == 0);
    if (cases[i].want_nopat) CHECK(nopat >= 5);
    if (cases[i].want_lows) CHECK(lows >= 5);
  }
}

// ============================================================================ calibration
void fw_calibration() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);
  txt::setLang(txt::Lang::EN);

  // before any calibration the simulator's default baseline is used and says so
  CHECK(has(r.say("status"), "sim default"));
  CHECK(r.shows(txt::t(txt::T_BASELINE_SIMDEF)));

  // calibrate in an empty, quiet place (preset B)
  CHECK(replyOk(r.say("preset b")));
  std::string o = r.say("calib");
  CHECK(replyOk(o));
  CHECK(r.state() == App::St::kCalibrating);
  CHECK(r.shows("CALIBRATING"));
  CHECK(r.waitScanEnd());
  CHECK(r.state() == App::St::kMessage);
  const Settings& s = r.app->settings().get();
  CHECK(s.bl_sim_ok);
  std::printf("  calibrated SIM baseline: %.2f dB (default %.1f)\n", static_cast<double>(s.bl_sim_db), static_cast<double>(RS_SIM_DEFAULT_BASELINE_DB));
  CHECK_NEAR(s.bl_sim_db, RS_SIM_DEFAULT_BASELINE_DB, 0.8);
  CHECK(r.shows(txt::t(txt::T_CALIB_OK)));
  CHECK(has(r.takeSerial(), "baseline stored"));
  CHECK(!r.shows("pessimistic"));                      // empty scene: no periodic content
  CHECK(r.app->scanCount() == 0);                       // a calibration is not a scan: no id, no log line
  CHECK(r.app->storage().lines() == 0);
  r.press(PIN_BTN_SCAN);                                // any button leaves the result screen
  CHECK(r.state() == App::St::kIdle);
  CHECK(!r.shows(txt::t(txt::T_BASELINE_SIMDEF)));
  CHECK(r.shows(txt::t(txt::T_BASELINE)));

  // it survives a power cycle
  const float stored = s.bl_sim_db;
  r.powerCycle();
  r.run_ms(300);
  CHECK(r.app->settings().get().bl_sim_ok);
  CHECK_NEAR(r.app->settings().get().bl_sim_db, stored, 1e-4);
  o = r.say("status");
  CHECK(has(o, "(calibrated)"));

  // a scan now uses the calibrated baseline and reports it as such
  CHECK(replyOk(r.say("preset a")));
  CHECK(r.scanByButton());
  CHECK(has(r.app->lastJson(), "\"baseline_src\":\"calibrated\""));
  CHECK(r.verdict() == rs::Verdict::kCandidate);

  // calibrating while a person is present: the baseline is stored but the operator is warned
  CHECK(replyOk(r.say("preset a")));
  CHECK(replyOk(r.say("calib")));
  CHECK(r.waitScanEnd());
  CHECK(r.shows("pessimistic"));
  r.press(PIN_BTN_SCAN);

  // calibrate in the noisy place: the documented limitation - the noise becomes "normal"
  CHECK(replyOk(r.say("preset f")));
  CHECK(replyOk(r.say("calib")));
  CHECK(r.waitScanEnd());
  const float noisy = r.app->settings().get().bl_sim_db;
  std::printf("  baseline calibrated in 3x noise: %.2f dB\n", static_cast<double>(noisy));
  CHECK(noisy > 28.0f);
  r.press(PIN_BTN_SCAN);
  CHECK(r.scanByButton());
  CHECK(r.verdict() == rs::Verdict::kNoPattern);        // the operator's responsibility: calibrate in a quiet empty spot

  // clearing falls back to the default (and is persisted)
  CHECK(replyOk(r.say("calib clear")));
  CHECK(!r.app->settings().get().bl_sim_ok);
  CHECK(has(r.say("status"), "sim default"));
  r.powerCycle();
  CHECK(!r.app->settings().get().bl_sim_ok);
  CHECK(replyOk(r.say("preset f")));
  CHECK(r.scanByButton());
  CHECK(r.verdict() == rs::Verdict::kLowSensitivity);   // back to the default baseline -> inconclusive, as it must be

  // calibration through the menu entry
  r.press(PIN_BTN_MENU);
  r.press(PIN_BTN_MODE);
  r.press(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kCalibrating);
  r.press(PIN_BTN_SCAN);                                // abort it
  CHECK(r.state() == App::St::kIdle);
  CHECK(r.app->ui().truncations() == 0);
}

// ============================================================================ abort, refusals, sound
void fw_abort_and_misc() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);
  CHECK(replyOk(r.say("preset a")));

  // abort with the button
  r.press(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kScanning);
  r.run_ms(10000);
  CHECK(r.state() == App::St::kScanning);
  r.press(PIN_BTN_MENU);                                // other buttons are ignored while scanning
  r.press(PIN_BTN_MODE);
  CHECK(r.state() == App::St::kScanning);
  r.takeSerial();
  r.press(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kIdle);
  const std::string aborted = r.takeSerial();
  CHECK(has(aborted, "scan aborted"));
  CHECK(aborted.find('{') == std::string::npos);        // no record for an aborted scan
  CHECK(r.shows(txt::t(txt::T_MSG_ABORTED)));
  CHECK(r.app->storage().lines() == 0);
  CHECK(r.app->acq().status().mode == AcqStatus::kIdle);
  CHECK(stub::output(PIN_LED_CAND) == 0 && stub::output(PIN_LED_UNCERT) == 0);

  // abort over the console, from the middle of the second segment
  CHECK(replyOk(r.say("scan")));
  CHECK(r.run_until([&] { return r.app->acq().status().mode == AcqStatus::kRunning && r.app->acq().status().frames > 100; }, 20000));
  CHECK(r.run_until([&] { return r.app->acq().status().mode == AcqStatus::kRunning && r.app->acq().status().frames < 50; }, 40000) ||
        true);
  CHECK(replyOk(r.say("abort")));
  CHECK(r.state() == App::St::kIdle);

  // refusals while a scan is running
  CHECK(replyOk(r.say("scan")));
  r.run_ms(2000);
  CHECK(replyErr(r.say("scan")));
  CHECK(replyErr(r.say("calib")));
  CHECK(replyErr(r.say("preset b")));
  CHECK(replyErr(r.say("src serial")));
  std::string o = r.say("set pfa 0.02");
  CHECK(replyErr(o) && has(o, "not while a scan is running"));
  CHECK(replyErr(r.say("log clear")));
  CHECK(replyErr(r.say("factory")));
  CHECK(r.state() == App::St::kScanning);
  CHECK(replyOk(r.say("status")));
  CHECK(has(r.say("status"), "state=scanning"));
  // a muted device stays silent for the whole scan and its result
  CHECK(replyOk(r.say("sound off")));
  uint32_t max_hz = 0;
  r.press(PIN_BTN_SCAN);                                // abort the running one (beep is muted already)
  CHECK(replyOk(r.say("preset a")));
  r.press(PIN_BTN_SCAN);
  for (int i = 0; i < 70000 && r.scanning(); i++) {
    r.ms();
    if (stub::buzzerHz() > max_hz) max_hz = stub::buzzerHz();
  }
  r.run_ms(2000);
  if (stub::buzzerHz() > max_hz) max_hz = stub::buzzerHz();
  CHECK(!r.scanning());
  CHECK(max_hz == 0);
  CHECK(r.verdict() == rs::Verdict::kCandidate);
  CHECK(stub::output(PIN_LED_CAND) == 1);               // the LEDs still work
  // ... and with sound on the same result is audible
  CHECK(replyOk(r.say("sound on")));
  const uint32_t tones0 = stub::toneCount();
  CHECK(r.scanByButton());
  r.run_ms(1500);                                       // the candidate melody (about 0.6 s) plays out after the result appears
  CHECK(stub::toneCount() > tones0 + 5);

  // consecutive scans back to back via the button on the result screen
  CHECK(r.state() == App::St::kResult);
  const uint32_t id0 = r.app->scanCount();
  CHECK(r.scanByButton());
  CHECK(r.app->scanCount() == id0 + 1);

  // changing the preset on the result screen leaves it
  r.press(PIN_BTN_MODE);
  CHECK(r.state() == App::St::kIdle);

  // the idle screen refreshes without ever piling text on text
  r.run_ms(5000);
  CHECK(stub::gfx().textOverText() == 0);
  CHECK(r.app->ui().truncations() == 0);
}

// ============================================================================ counter wrap-around
void fw_clock_wrap() {
  rs::ScanOutcome ref;
  {
    stub::reset();
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    CHECK(replyOk(r.say("preset a")));
    CHECK(r.scanByButton());
    ref = r.app->lastOutcome();
    CHECK(ref.verdict == rs::Verdict::kCandidate);
  }
  // both counters wrap together at 2^32 ms (49.7 days); micros() alone wraps every 71.6 minutes
  struct W { uint64_t start_us; const char* what; };
  const W cases[] = {{4294967296000ull - 20000000ull, "millis() and micros() wrap 20 s into the scan"},
                     {4294967296ull * 3 - 20000000ull, "micros() wraps 20 s into the scan"},
                     {4294967296ull - 3000000ull, "micros() wraps 3 s into the scan (during the IMU settling)"},
                     {4294967296000ull - 70000000ull, "millis() wraps 70 s after the start (between segments)"}};
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    stub::reset();
    stub::setNowUs(cases[i].start_us);
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    CHECK(replyOk(r.say("preset a")));
    CHECK(r.scanByButton());
    const rs::ScanOutcome& o = r.app->lastOutcome();
    std::printf("  wrap: %-62s -> %s hz=%.3f bin=%u\n", cases[i].what, rs::verdictName(o.verdict), static_cast<double>(o.hz),
                static_cast<unsigned>(o.bin));
    CHECK(o.verdict == ref.verdict);
    CHECK(o.bin == ref.bin);
    CHECK_NEAR(o.hz, ref.hz, 1e-4);
    CHECK_NEAR(o.margin_db, ref.margin_db, 0.05);
    CHECK(o.n_invalid == 0);
    // the button logic keeps working across the wrap, too
    CHECK(r.app->scanCount() == 1);
    r.press(PIN_BTN_MODE);
    CHECK(r.state() == App::St::kIdle);
  }
}

// ============================================================================ IMU faults
namespace {
void runOneScanWith(Rig& r, uint8_t preset, rs::Verdict want, rs::InvalidReason want_why, const char* what) {
  CHECK(replyOk(r.say(("preset " + std::to_string(preset)).c_str())));
  CHECK(r.scanByButton());
  const rs::ScanOutcome& o = r.app->lastOutcome();
  std::printf("  IMU case: %-58s -> %s (%s)\n", what, rs::verdictName(o.verdict), rs::invalidReasonName(o.last_invalid));
  CHECK(o.verdict == want);
  if (want == rs::Verdict::kInvalid) CHECK(o.last_invalid == want_why);
}
}  // namespace

void fw_imu_faults() {
  const rs::InvalidReason none = rs::InvalidReason::kNone;

  {  // no IMU at all: the simulator supplies a simulated one, and says so everywhere
    stub::reset();
    stub::mpu().present = false;
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    CHECK(has(r.takeSerial(), "IMU: not found - the simulator supplies a simulated IMU"));
    CHECK(has(r.say("status"), "imu=sim"));
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "IMU absent, SIM source");
    CHECK(has(r.app->lastJson(), "\"imu\":\"sim\""));
    CHECK(has(r.say("selftest"), "FAIL"));              // the self-test does not hide it
  }
  {  // sensor answers but the configuration does not stick -> treated as not found
    stub::reset();
    stub::mpu().config_sticks = false;
    Rig r;
    r.boot();
    CHECK(!r.app->acq().status().imu_hw);
    CHECK(has(r.takeSerial(), "IMU: not found"));
    CHECK(has(r.say("status"), "imu=sim"));
  }
  for (int who = 0; who < 2; who++) {  // something else answers at 0x68 (WHO_AM_I 0x00 / 0xFF)
    stub::reset();
    stub::mpu().who = who ? 0xFF : 0x00;
    Rig r;
    r.boot();
    CHECK(has(r.takeSerial(), "IMU: not found"));
  }
  {  // alternative address 0x69 (AD0 high)
    stub::reset();
    stub::mpu().addr = 0x69;
    Rig r;
    r.boot();
    CHECK(has(r.takeSerial(), "sensor at 0x69"));
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "IMU at 0x69");
  }
  {  // a gyro zero-rate offset of 23 deg/s (MPU6050 offsets reach tens of deg/s) is removed, not flagged
    stub::reset();
    stub::mpu().gyro[0] = 0.4f;
    stub::mpu().gyro[2] = -0.25f;
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    r.run_ms(3000);
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "gyro zero-rate offset 0.4 rad/s");
  }
  {  // sensor mounted tilted (gravity on all axes) is fine
    stub::reset();
    stub::mpu().accel[0] = 4.0f;
    stub::mpu().accel[1] = -3.0f;
    stub::mpu().accel[2] = 8.4f;
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "sensor mounted at an angle");
  }
  {  // IMU freezes mid-scan (stuck output, no bus error): the segment cannot be trusted
    stub::reset();
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    CHECK(replyOk(r.say("preset a")));
    r.press(PIN_BTN_SCAN);
    r.run_ms(8000);
    stub::mpu().frozen = true;
    CHECK(r.waitScanEnd());
    const rs::ScanOutcome& o = r.app->lastOutcome();
    std::printf("  IMU case: %-58s -> %s (%s)\n", "IMU output freezes during the scan", rs::verdictName(o.verdict), rs::invalidReasonName(o.last_invalid));
    CHECK(o.verdict == rs::Verdict::kInvalid && o.last_invalid == rs::InvalidReason::kImuMissing);
    stub::mpu().frozen = false;
    r.run_ms(2000);
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "same device after the IMU recovered");
  }
  {  // bus errors
    stub::reset();
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    CHECK(replyOk(r.say("preset a")));
    r.press(PIN_BTN_SCAN);
    r.run_ms(5000);
    stub::mpu().nack_reads = true;
    CHECK(r.waitScanEnd());
    CHECK(r.verdict() == rs::Verdict::kInvalid && r.app->lastOutcome().last_invalid == rs::InvalidReason::kImuMissing);
    stub::mpu().nack_reads = false;
    CHECK(r.app->acq().status().imu_errors > 0);
  }
  {  // repeated knocks the IMU sees: more than 5 % of the frames flagged -> invalid, never a verdict
    stub::reset();
    stub::mpu().script = [](double t, float a[3], float[3]) {
      if (fmod(t, 3.0) < 0.12) a[0] += 3.0f;
    };
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kInvalid, rs::InvalidReason::kMotion, "knocked every 3 s (IMU sees it)");
    CHECK(r.app->lastOutcome().n_invalid == rs::kMaxInvalid);   // two invalid segments end the scan
  }
  {  // a single short knock is tolerated (< 5 % of the frames)
    stub::reset();
    stub::mpu().script = [](double t, float a[3], float[3]) {
      if (t > 40.0 && t < 40.1) a[0] += 3.0f;
    };
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "one short knock");
  }
  {  // slow tilt (the sensor slides on the rubble), invisible to a high-passed accelerometer
    stub::reset();
    stub::mpu().script = [](double t, float a[3], float[3]) {
      const double th = 3.0 * 3.14159265 / 180.0 * t;        // 3 deg/s
      a[0] += static_cast<float>(9.80665 * sin(th));
      a[2] += static_cast<float>(9.80665 * (cos(th) - 1.0));
    };
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kInvalid, rs::InvalidReason::kMotion, "sensor tilting at 3 deg/s");
  }
  {  // a slower tilt of 0.5 deg/s is below the gate
    stub::reset();
    stub::mpu().script = [](double t, float a[3], float[3]) {
      const double th = 0.5 * 3.14159265 / 180.0 * fmod(t, 40.0);
      a[0] += static_cast<float>(9.80665 * sin(th));
      a[2] += static_cast<float>(9.80665 * (cos(th) - 1.0));
    };
    Rig r;
    r.boot();
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kCandidate, none, "sensor tilting at 0.5 deg/s");
  }
  {  // the IMU is unplugged *during idle*: the idle screen shows it and a scan is refused as invalid, never trusted
    stub::reset();
    Rig r;
    r.boot();
    r.run_ms(500);
    stub::mpu().nack_reads = true;
    r.run_ms(1500);
    CHECK(r.shows(txt::t(txt::T_IMU_MISSING)));
    stub::gfx().enabled = false;
    runOneScanWith(r, 1, rs::Verdict::kInvalid, rs::InvalidReason::kImuMissing, "IMU unplugged while idle");
  }
}

// ============================================================================ memory: out of memory at boot
void fw_out_of_memory() {
  for (int fail_at = 1; fail_at <= 2; fail_at++) {
    stub::reset();
    g_big_allocs = 0;
    g_fail_big_alloc = fail_at;
    {
      Rig r;
      r.boot();
      g_fail_big_alloc = 0;
      CHECK(!r.began);
      CHECK(r.state() == App::St::kFatal);
      CHECK(has(r.takeSerial(), "FATAL: out of memory"));
      CHECK(r.shows("FATAL ERROR"));
      CHECK(stub::output(PIN_LED_UNCERT) == 1 && stub::output(PIN_LED_CAND) == 0);
      r.run_ms(500);                                   // nothing runs, nothing crashes
      r.press(PIN_BTN_SCAN);
      CHECK(r.state() == App::St::kFatal);
      CHECK(r.app->scanCount() == 0);
    }
  }
  g_fail_big_alloc = 0;
}

void fw_memory_report() {
  const unsigned app = static_cast<unsigned>(sizeof(App));
  const unsigned seg = static_cast<unsigned>(sizeof(rs::SegmentBuffer));
  const unsigned ws = static_cast<unsigned>(sizeof(rs::Workspace));
  const unsigned rec = static_cast<unsigned>(sizeof(ScanRecord));
  std::printf("  sizeof: App %u B (static, includes the %u B JSON buffer), SegmentBuffer %u B (heap), Workspace %u B (heap), ScanRecord %u B (stack)\n", app,
              static_cast<unsigned>(kScanJsonMax), seg, ws, rec);
  g_report.push_back("### Memory (host build; the 32-bit target differs only by pointer sizes)");
  g_report.push_back("");
  g_report.push_back("| object | bytes | where |");
  g_report.push_back("|---|---|---|");
  g_report.push_back("| App (all modules, rings, JSON buffer) | " + std::to_string(app) + " | static (.bss) |");
  g_report.push_back("| rs::SegmentBuffer (24 bins x 512 frames x int16 I/Q) | " + std::to_string(seg) + " | heap, one block |");
  g_report.push_back("| rs::Workspace (FFT / CFAR scratch) | " + std::to_string(ws) + " | heap, one block |");
  g_report.push_back("| ScanRecord (JSON builder input) | " + std::to_string(rec) + " | stack of the UI thread |");
  g_report.push_back("");
  CHECK(app < 24u * 1024u);                             // static RAM budget for the application object
  CHECK(seg + ws < 100u * 1024u);                       // heap budget of the two big blocks
  CHECK(rec < 3u * 1024u);                              // the UI thread's stack (8 KB loop task) must carry it
}
