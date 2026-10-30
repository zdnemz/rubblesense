// Firmware host tests, part 3: the serial I/Q source against an external radar front-end model -
// readiness, clean streams, calibration, link faults, stalls.
#include <math.h>
#include <string.h>
#include <string>
#include "fw_iq_rig.h"
#include "rs_text.h"

using iq::Rig2;
using iq::Tally;

// ============================================================================ clean streams
void fw_iq_clean() {
  stub::reset();
  Rig2 t(false);                                   // the front-end has not announced itself yet
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = true;
  r.run_ms(300);
  CHECK_OK(t.cmd("src serial"));
  r.run_ms(2000);

  // ---- 1. nothing connected: the device says so and refuses to scan
  CHECK(!r.app->sourceReady());
  CHECK(r.shows(txt::t(txt::T_SRC_WAITING)));
  const uint32_t scans0 = r.app->scanCount();
  std::string out = t.cmd("scan");
  CHECK(replyErr(out));
  CHECK(has(out, "scan refused: radar source not ready"));
  CHECK(r.state() == App::St::kIdle);
  CHECK(r.app->scanCount() == scans0);
  CHECK(has(t.cmd("selftest"), "FAIL SERIAL no info"));
  CHECK_ERR(t.cmd("calib"));
  CHECK(r.state() == App::St::kIdle);

  // ---- 2. the front-end powers up: it is announced within a second and the idle screen follows
  t.fe.announce = true;
  t.connect(iq::person(), 11);
  CHECK(r.run_until([&] { return r.app->sourceReady(); }, 1500));
  r.run_ms(900);                                   // the idle status rows refresh every 400 ms
  CHECK(r.shows(txt::t(txt::T_SRC_CONNECTED)));
  CHECK(!r.shows(txt::t(txt::T_SRC_WAITING)));
  CHECK(has(t.cmd("selftest"), "OK SERIAL 20Hz 16bins"));

  // ---- 3. uncalibrated front-end: a scan works but the sensitivity is reported as unknown
  CHECK(has(t.cmd("status"), "baseline: none (sensitivity unknown)"));
  CHECK(r.scanByButton());
  const rs::ScanOutcome o = r.app->lastOutcome();
  checkScanInvariants(r, o);
  CHECK(o.verdict == rs::Verdict::kCandidate);
  CHECK_NEAR(o.bpm, 16.8, 2.0);
  CHECK(std::abs(static_cast<int>(o.bin) - 9) <= 1);
  CHECK(!o.sens_known);
  const std::string json = r.app->lastJson();
  CHECK(has(json, "\"source\":\"SERIAL\""));
  CHECK(has(json, "\"sensitivity\":\"unknown\""));
  CHECK(has(json, "\"baseline_src\":\"none\""));
  CHECK(has(json, "\"imu\":\"hw\""));
  CHECK(has(json, "\"radar\":{\"name\":\"SERIAL\",\"fs_hz\":20.00"));
  CHECK(jsonInt(json, "lost_frames") == 0);
  CHECK(jsonInt(json, "crc_errors") == 0);
  CHECK(r.app->storage().lines() >= 1);

  // ---- 4. the front-end goes silent while the device is idle: the device notices within seconds
  t.dismiss();
  CHECK(r.state() == App::St::kIdle);
  t.fe.stop();
  t.fe.announce = false;
  r.run_ms(2500);
  CHECK(r.app->sourceReady());                     // the last info packet is still fresh (< 5 s)
  r.run_ms(3500);
  CHECK(!r.app->sourceReady());
  r.run_ms(500);
  CHECK(r.shows(txt::t(txt::T_SRC_WAITING)));
  CHECK_ERR(t.cmd("scan"));
  // ... and when it comes back
  t.fe.announce = true;
  t.connect(iq::person(), 12);
  CHECK(r.run_until([&] { return r.app->sourceReady(); }, 1500));
  r.run_ms(900);
  CHECK(r.shows(txt::t(txt::T_SRC_CONNECTED)));

  iq::reportHeader(std::string("### Serial I/Q source, clean stream (") + iq::buildName() + ")",
                   "External front-end model on the serial link, 20 Hz x 16 bins, 25.6 s segments, white-Gaussian simulator, "
                   "local IMU stub at rest, no calibration (sensitivity unknown).");
  stub::gfx().enabled = false;
  const Tally tp = iq::runScans(t, iq::person(30.0f), 6, 200);
  iq::report("person (amplitude 30)", tp);
  CHECK(tp.c(rs::Verdict::kCandidate) == tp.n);
  CHECK(tp.n == 6);
  const Tally te = iq::runScans(t, iq::empty(), 6, 100);
  iq::report("empty scene", te);
  CHECK(te.c(rs::Verdict::kCandidate) == 0);
  CHECK(te.c(rs::Verdict::kNoPattern) + te.c(rs::Verdict::kPossible) == te.n);

  // ---- 5. other geometries than the Wokwi default: the firmware must not be hard-wired to 20 Hz x 16 bins
  {
    rs::SimScene s = iq::person(30.0f);
    s.fs_hz = 25.0f;                               // a faster front-end: segments are 20.5 s long, the frequency axis scales
    const Tally k = iq::runScans(t, s, 2, 300);
    iq::report("person, front-end at 25 Hz", k);
    CHECK(k.c(rs::Verdict::kCandidate) == k.n);
    CHECK_NEAR(r.app->lastOutcome().bpm, 16.8, 2.0);
    CHECK(has(r.app->lastJson(), "\"fs_hz\":25.00"));
  }
  {
    rs::SimScene s = iq::person(30.0f);
    s.n_bins = 24;                                 // all bins the firmware can store
    s.target_bin = 20;
    const Tally k = iq::runScans(t, s, 2, 400);
    iq::report("person, 24 range bins (target in bin 20)", k);
    CHECK(k.c(rs::Verdict::kCandidate) == k.n);
    CHECK(std::abs(static_cast<int>(r.app->lastOutcome().bin) - 20) <= 1);
  }
  {
    rs::SimScene s = iq::person(30.0f);
    s.n_bins = 8;
    s.wall_bin = 3;
    s.target_bin = 5;
    const Tally k = iq::runScans(t, s, 2, 500);
    iq::report("person, 8 range bins (target in bin 5)", k);
    CHECK(k.c(rs::Verdict::kCandidate) == k.n);
    CHECK(std::abs(static_cast<int>(r.app->lastOutcome().bin) - 5) <= 1);
  }
}

// ============================================================================ calibration on a serial front-end
void fw_iq_calibration() {
  stub::reset();
  Rig2 t(true);
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = false;
  CHECK(t.bringUp(iq::empty(), 1));                // quiet, empty reference spot

  // ---- a dead front-end (silent line) cannot be calibrated, and nothing is stored
  t.fe.stop();
  CHECK_OK(t.cmd("calib"));
  CHECK(r.state() == App::St::kCalibrating);
  CHECK(r.waitScanEnd());
  CHECK(r.state() == App::St::kMessage);
  CHECK(!r.app->settings().get().bl_ser_ok);
  t.dismiss();
  CHECK(r.state() == App::St::kIdle);

  // ---- a front-end that streams zeros (ADC dead / disconnected) cannot be calibrated either
  t.connect(iq::silent(), 2);
  r.run_ms(1100);
  CHECK_OK(t.cmd("calib"));
  CHECK(r.waitScanEnd());
  CHECK(r.state() == App::St::kMessage);
  CHECK(!r.app->settings().get().bl_ser_ok);
  t.dismiss();

  // ---- the real calibration on the empty reference scene
  t.connect(iq::empty(), 3);
  r.run_ms(1100);
  CHECK_OK(t.cmd("calib"));
  CHECK(r.state() == App::St::kCalibrating);
  CHECK(r.waitScanEnd());
  CHECK(r.state() == App::St::kMessage);
  {
    const Settings& s = r.app->settings().get();
    CHECK(s.bl_ser_ok);
    CHECK(s.bl_ser_db > 19.0f && s.bl_ser_db < 23.5f);      // sigma 8 counts -> about 21.1 dB
    CHECK(!s.bl_sim_ok);                                    // a separate slot from the simulator's
  }
  t.dismiss();
  CHECK(has(t.cmd("status"), "(calibrated)"));

  // ---- it survives a power cycle, and the source selection with it
  t.powerCycle();
  r.run_ms(300);
  CHECK(r.app->settings().get().source == SourceKind::kSerial);
  CHECK(r.app->settings().get().bl_ser_ok);
  CHECK(t.waitReady(2500));
  CHECK(has(t.cmd("status"), "(calibrated)"));

  iq::reportHeader(std::string("### Serial I/Q source, calibrated (") + iq::buildName() + ")",
                   "Baseline measured on an empty scene (sigma 8 counts); then scans on a quiet and on a noisy front-end.");
  const iq::Tally quiet = iq::runScans(t, iq::person(30.0f), 2, 600);
  iq::report("person, quiet front-end", quiet);
  CHECK(quiet.c(rs::Verdict::kCandidate) == quiet.n);
  {
    const std::string j = r.app->lastJson();
    CHECK(has(j, "\"sensitivity\":\"ok\""));
    CHECK(has(j, "\"baseline_src\":\"calibrated\""));
    CHECK(r.app->lastOutcome().sens_known);
    CHECK(std::fabs(r.app->lastOutcome().floor_delta_db) < 3.0f);
  }
  // A noisier front-end (+9.6 dB over the baseline): a negative result must lose its meaning.
  const iq::Tally noisy_empty = iq::runScans(t, iq::empty(24.0f), 4, 700);
  iq::report("empty, floor +9.6 dB over the baseline", noisy_empty);
  CHECK(noisy_empty.c(rs::Verdict::kNoPattern) == 0);
  CHECK(noisy_empty.c(rs::Verdict::kCandidate) == 0);
  CHECK(noisy_empty.c(rs::Verdict::kLowSensitivity) >= 3);
  {
    const std::string j = r.app->lastJson();
    if (r.app->lastOutcome().verdict == rs::Verdict::kLowSensitivity) CHECK(has(j, "\"sensitivity\":\"low\""));
  }
  const iq::Tally noisy_person = iq::runScans(t, iq::person(30.0f, 24.0f), 4, 800);
  iq::report("person, floor +9.6 dB over the baseline", noisy_person);
  CHECK(noisy_person.c(rs::Verdict::kNoPattern) == 0);
  CHECK(noisy_person.c(rs::Verdict::kCandidate) + noisy_person.c(rs::Verdict::kLowSensitivity) +
            noisy_person.c(rs::Verdict::kPossible) == noisy_person.n);

  // ---- clearing the baseline goes back to "unknown"
  CHECK_OK(t.cmd("calib clear"));
  CHECK(!r.app->settings().get().bl_ser_ok);
  CHECK(has(t.cmd("status"), "baseline: none"));
}

// ============================================================================ link faults
namespace {
struct FaultCase {
  const char* name;
  int drop, corrupt, dup, garbage;                // 0 = off, N = every N-th packet
  bool valid;                                     // the scan must be analysable
};
}  // namespace

void fw_iq_faults() {
  stub::reset();
  Rig2 t(true);
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = false;
  CHECK(t.bringUp(iq::person(), 1));

  iq::reportHeader(std::string("### Serial I/Q source, link faults (") + iq::buildName() + ")",
                   "Person of amplitude 30 unless noted. Up to 2 % lost frames are tolerated (zero-order hold, flagged); "
                   "more makes the segment invalid ('radar_dropout'), never a verdict from damaged data.");

  const FaultCase kCases[] = {
      {"1 % of the frames lost", 100, 0, 0, 0, true},
      {"1 % of the packets corrupted (CRC)", 0, 100, 0, 0, true},
      {"every 3rd packet duplicated", 0, 0, 3, 0, true},
      {"1-40 garbage bytes before every 5th packet", 0, 0, 0, 5, true},
      {"mix: 0.5 % lost, 0.5 % corrupt, dup 1/7, garbage 1/9", 200, 199, 7, 9, true},
      {"3 % of the frames lost", 33, 0, 0, 0, false},
      {"10 % of the frames lost", 10, 0, 0, 0, false},
      {"12.5 % of the packets corrupted (CRC)", 0, 8, 0, 0, false},
  };
  uint32_t seed = 1000;
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); i++) {
    const FaultCase& fc = kCases[i];
    t.fe.drop_every = fc.drop;
    t.fe.corrupt_every = fc.corrupt;
    t.fe.dup_every = fc.dup;
    t.fe.garbage_every = fc.garbage;
    const Tally k = iq::runScans(t, iq::person(), 2, seed);
    seed += 10;
    iq::report(fc.name, k);
    if (fc.valid) {
      CHECK(k.c(rs::Verdict::kCandidate) == k.n);
      CHECK(k.c(rs::Verdict::kInvalid) == 0);
    } else {
      CHECK(k.c(rs::Verdict::kInvalid) == k.n);
      CHECK(k.why(rs::InvalidReason::kDropout) == k.n);
      CHECK(r.app->lastOutcome().n_invalid == rs::kMaxInvalid);
      CHECK(r.app->lastOutcome().n_valid == 0);
    }
    if (fc.drop > 0) CHECK(k.lost > 0);
    if (fc.corrupt > 0) {
      CHECK(k.crc > 0);
      CHECK(k.lost > 0);
      CHECK(k.crc <= k.lost + 4);                   // a damaged packet is one lost frame, it does not take its neighbour along
      CHECK(k.lost <= k.crc * 2 + 6);
    }
    if (fc.drop == 0 && fc.corrupt == 0) CHECK(k.lost == 0);   // duplicates and garbage lose nothing
    if (fc.garbage > 0) {
      const std::string st = t.cmd("status");
      CHECK(iq::statusField(st, "skipped") > 0);
    }
    if (fc.dup > 0 && fc.drop == 0) {
      const std::string st = t.cmd("status");
      CHECK(iq::statusField(st, "lost") == 0);
    }
  }
  t.fe.drop_every = t.fe.corrupt_every = t.fe.dup_every = t.fe.garbage_every = 0;

  // ---- the same mild damage on an empty scene must not create a candidate
  t.fe.drop_every = 200;
  t.fe.corrupt_every = 199;
  t.fe.dup_every = 7;
  t.fe.garbage_every = 9;
  const Tally ke = iq::runScans(t, iq::empty(), 3, 2000);
  iq::report("mix as above, empty scene", ke);
  CHECK(ke.c(rs::Verdict::kCandidate) == 0);
  t.fe.drop_every = t.fe.corrupt_every = t.fe.dup_every = t.fe.garbage_every = 0;

  // ---- the acquisition task is starved while a segment is being recorded (a long flash operation, a
  // ---- priority inversion). The UART driver's ring holds a couple of seconds of this stream, so a short
  // ---- hiccup costs nothing; a longer one overflows the ring, frames are lost, and the segment that
  // ---- suffered it is rejected (sequence gap and a hole in the stability monitoring) instead of analysed.
  {
    const double pkt = static_cast<double>(rs::kProtoHeader) + 4.0 * 16.0 + 2.0;     // bytes per frame packet, 16 bins
    const double ring_ms = 1000.0 * RS_IQ_RX_RING / (pkt * 20.0);                    // stream time the ring can hold
    const uint32_t mid = static_cast<uint32_t>(0.6 * ring_ms < 900.0 ? 0.6 * ring_ms : 900.0);
    uint32_t over = static_cast<uint32_t>(ring_ms + 700.0);          // 14 frames beyond the ring (UART0 build) ...
    if (over > 2800) over = 2800;                    // ... or as long as the 3 s stall limit allows (a different test)
    struct Starve { uint32_t ms; bool lossless; bool person; };
    const Starve kStarve[] = {{300, true, true}, {mid, true, true}, {over, false, true}, {over, false, false}};
    for (size_t i = 0; i < sizeof(kStarve) / sizeof(kStarve[0]); i++) {
      const Starve& sv = kStarve[i];
      t.connect(sv.person ? iq::person() : iq::empty(), 3000 + static_cast<uint32_t>(i));
      r.run_ms(1100);
      r.press(PIN_BTN_SCAN);
      CHECK(r.scanning());
      CHECK(r.run_until([&] { return r.app->acq().status().frames > 100 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
      const uint32_t dropped0 = iq::port().rxDropped();
      r.starve_ms(sv.ms);
      const uint32_t driver_lost = iq::port().rxDropped() - dropped0;
      CHECK(r.waitScanEnd());
      const rs::ScanOutcome o = r.app->lastOutcome();
      const long lost = jsonInt(r.app->lastJson(), "lost_frames");
      std::printf("  starved acquisition %4u ms (%s): driver dropped %4u B, lost_frames=%ld, %s, valid=%d invalid=%d (%s)\n",
                  static_cast<unsigned>(sv.ms), sv.person ? "person" : "empty", static_cast<unsigned>(driver_lost), lost,
                  rs::verdictName(o.verdict), o.n_valid, o.n_invalid, rs::invalidReasonName(o.last_invalid));
      checkScanInvariants(r, o, 16);
      if (sv.lossless) {
        CHECK(driver_lost == 0);
        CHECK(lost == 0);
        CHECK(o.n_invalid == 0);
        CHECK(o.verdict == rs::Verdict::kCandidate);
      } else {
        CHECK(driver_lost > 0);                      // the driver ring really overflowed
        CHECK(lost >= 1 && lost <= static_cast<long>((sv.ms - ring_ms) / 50.0) + 6);   // ... and the receiver noticed the sequence gap
        CHECK(o.n_invalid >= 1);                     // the damaged segment was thrown away
        CHECK(o.last_invalid == rs::InvalidReason::kMotion || o.last_invalid == rs::InvalidReason::kDropout);
        if (sv.person) CHECK(o.verdict == rs::Verdict::kCandidate && o.n_valid >= 2);   // later clean segments still find it
        else CHECK(o.verdict != rs::Verdict::kCandidate);
      }
      t.dismiss();
    }
  }
}

// ============================================================================ stalls and restarts
void fw_iq_stall() {
  stub::reset();
  Rig2 t(true);
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = false;
  CHECK(t.bringUp(iq::person(), 1));
  r.run_ms(1100);

  iq::reportHeader(std::string("### Serial I/Q source, stalls (") + iq::buildName() + ")",
                   "The front-end stops delivering frames in the middle of a scan.");

  // ---- 1. it never comes back: every segment ends as 'radar_dropout' after 3 s, the scan ends invalid
  {
    t.connect(iq::person(), 21);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    CHECK(r.scanning());
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 100 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
    t.fe.stop();
    r.takeSerial();
    CHECK(r.waitScanEnd(120000));
    const std::string log = r.takeSerial();
    CHECK(countOf(log, "# radar link stalled: segment invalid") == rs::kMaxInvalid);
    const rs::ScanOutcome o = r.app->lastOutcome();
    CHECK(o.verdict == rs::Verdict::kInvalid);
    CHECK(o.last_invalid == rs::InvalidReason::kDropout);
    CHECK(o.n_valid == 0 && o.n_invalid == rs::kMaxInvalid);
    checkScanInvariants(r, o);
    CHECK(has(r.app->lastJson(), "\"invalid_reason\":\"radar_dropout\""));
    g_report.push_back("| front-end stops mid-segment and never returns | 1 | 0 | 0 | 0 | 0 | 1 | 0 | - | - | - |");
  }

  // ---- 2. it returns after a few seconds: the scan continues with a fresh segment, the damaged one is discarded
  {
    t.connect(iq::person(), 22);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    CHECK(r.scanning());
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 100 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
    t.fe.stop();
    r.takeSerial();
    r.run_ms(4000);
    CHECK(has(r.takeSerial(), "# radar link stalled: segment invalid"));
    t.fe.resume(stub::nowUs());
    CHECK(r.waitScanEnd());
    const rs::ScanOutcome o = r.app->lastOutcome();
    checkScanInvariants(r, o);
    CHECK(o.n_invalid == 1);
    CHECK(o.n_valid >= 2);
    CHECK(o.verdict == rs::Verdict::kCandidate);
    CHECK(has(r.app->lastJson(), "\"segments_invalid\":1"));
    g_report.push_back(std::string("| front-end pauses 4 s, then resumes | 1 | ") +
                       (o.verdict == rs::Verdict::kCandidate ? "1" : "0") + " | 0 | 0 | 0 | 0 | 0 | - | - | - |");
  }

  // ---- 3. the operator aborts a stalled scan: no result, clean idle, next scan works
  {
    t.connect(iq::person(), 23);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 50; }, 40000));
    t.fe.stop();
    r.run_ms(1500);
    const uint32_t lines0 = r.app->storage().lines();
    r.press(PIN_BTN_SCAN);
    CHECK(r.state() == App::St::kIdle);
    CHECK(r.app->storage().lines() == lines0);
    t.connect(iq::person(), 24);
    r.run_ms(1100);
    CHECK(r.scanByButton());
    CHECK(r.verdict() == rs::Verdict::kCandidate);
  }
}
