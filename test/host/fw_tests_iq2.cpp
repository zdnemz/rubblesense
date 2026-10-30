// Firmware host tests, part 4: serial I/Q source - stability information from the front-end's own IMU,
// geometry changes, and the console that shares the UART with the I/Q stream.
#include <math.h>
#include <string.h>
#include <string>
#include "fw_iq_rig.h"
#include "rs_text.h"

using iq::Rig2;
using iq::Tally;

// ============================================================================ IMU on the radar head
void fw_iq_host_imu() {
  iq::reportHeader(std::string("### Serial I/Q source, stability information (") + iq::buildName() + ")",
                   "The IMU must move with the antenna. A front-end that carries its own IMU says so in every frame "
                   "(protocol flag); the device then ignores its local IMU for those frames.");

  // ---- (a) no IMU on the ESP32 board at all: the front-end vouches for stability
  {
    stub::reset();
    stub::mpu().present = false;
    Rig2 t(true);
    t.fe.host_imu = true;
    t.boot();
    Rig& r = t.r;
    stub::gfx().enabled = true;
    CHECK(t.bringUp(iq::person(), 1));
    r.run_ms(1500);                                              // frames flow while idle: the idle screen already knows
    CHECK(r.shows(txt::t(txt::T_IMU_HOST)));
    CHECK(!r.shows(txt::t(txt::T_IMU_MISSING)));
    stub::gfx().enabled = false;
    const Tally k = iq::runScans(t, iq::person(), 2, 10);
    iq::report("no local IMU, front-end IMU flags (person)", k);
    CHECK(k.c(rs::Verdict::kCandidate) == k.n);
    CHECK(has(r.app->lastJson(), "\"imu\":\"host\""));
    CHECK(has(t.cmd("status"), "imu=host"));
    const Tally ke = iq::runScans(t, iq::empty(), 2, 20);
    iq::report("no local IMU, front-end IMU flags (empty)", ke);
    CHECK(ke.c(rs::Verdict::kCandidate) == 0);
    CHECK(ke.c(rs::Verdict::kInvalid) == 0);

    // the front-end stops vouching (its IMU died, flags gone): the idle screen must not keep claiming it
    t.fe.host_imu = false;
    t.dismiss();
    r.run_ms(3000);
    stub::gfx().enabled = true;
    r.run_ms(1000);
    CHECK(!r.shows(txt::t(txt::T_IMU_HOST)));
    CHECK(r.shows(txt::t(txt::T_IMU_MISSING)));
    stub::gfx().enabled = false;
    // ... and a scan without any stability information is invalid, never a verdict
    const Tally ki = iq::runScans(t, iq::person(), 1, 30);
    iq::report("no local IMU, no front-end flags (person)", ki);
    CHECK(ki.c(rs::Verdict::kInvalid) == 1);
    CHECK(ki.why(rs::InvalidReason::kImuMissing) == 1);
  }

  // ---- (b) the radar head is bumped: the front-end's IMU flags it, the scan is invalid (the device's own IMU sits still)
  {
    stub::reset();
    Rig2 t(true);
    t.fe.host_imu = true;
    t.boot();
    stub::gfx().enabled = false;
    CHECK(t.bringUp(iq::empty(), 1));
    rs::SimScene s = iq::empty();
    s.bump_rate_hz = 0.3f;
    s.bump_mm = 1.0f;
    const Tally k = iq::runScans(t, s, 3, 40);
    iq::report("radar head bumped 1.0 mm at 0.3/s, flagged by front-end IMU", k);
    CHECK(k.c(rs::Verdict::kInvalid) >= 2);
    CHECK(k.why(rs::InvalidReason::kMotion) == k.c(rs::Verdict::kInvalid));
    CHECK(k.c(rs::Verdict::kCandidate) == 0);
    CHECK(k.c(rs::Verdict::kNoPattern) == 0 || k.c(rs::Verdict::kInvalid) < k.n);   // never a negative from a bumped sensor, apart from rare quiet runs
  }

  // ---- (c) the local IMU is knocked about, the front-end (which is what the radar is on) is still: its verdict counts
  {
    stub::reset();
    stub::mpu().script = [](double t_s, float a[3], float[3]) {
      if (fmod(t_s, 3.0) < 0.12) a[0] += 3.0f;                  // a knock every 3 s on the ESP32 board only
    };
    Rig2 t(true);
    t.fe.host_imu = true;
    t.boot();
    stub::gfx().enabled = false;
    CHECK(t.bringUp(iq::person(), 1));
    const Tally k = iq::runScans(t, iq::person(), 2, 50);
    iq::report("local IMU knocked every 3 s, front-end still", k);
    CHECK(k.c(rs::Verdict::kCandidate) == k.n);
    CHECK(has(t.r.app->lastJson(), "\"imu\":\"host\""));
  }

  // ---- (d) the contrary: no front-end IMU flags, the local IMU is what the device has -> it is used
  {
    stub::reset();
    stub::mpu().script = [](double t_s, float a[3], float[3]) {
      if (fmod(t_s, 3.0) < 0.12) a[0] += 3.0f;
    };
    Rig2 t(true);
    t.boot();
    stub::gfx().enabled = false;
    CHECK(t.bringUp(iq::person(), 1));
    const Tally k = iq::runScans(t, iq::person(), 1, 60);
    iq::report("local IMU knocked every 3 s, no front-end flags", k);
    CHECK(k.c(rs::Verdict::kInvalid) == 1);
    CHECK(k.why(rs::InvalidReason::kMotion) == 1);
  }
}

// ============================================================================ geometry changes
void fw_iq_geometry() {
  stub::reset();
  Rig2 t(true);
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = true;
  CHECK(t.bringUp(iq::person(), 1));
  r.run_ms(500);

  // Starts a scan and runs into the first segment.
  auto startAndRun = [&](uint32_t seed) {
    t.connect(iq::person(), seed);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    CHECK(r.scanning());
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 60 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
  };

  struct Change {
    const char* what;
    void (*apply)(FrontEnd&);
    void (*undo)(FrontEnd&);
  };
  const Change kChanges[] = {
      {"bin count", [](FrontEnd& f) { f.announce_bins = 20; }, [](FrontEnd& f) { f.announce_bins = 0; }},
      {"frame rate", [](FrontEnd& f) { f.announce_fs = 25.0f; }, [](FrontEnd& f) { f.announce_fs = 0.0f; }},
      {"range step", [](FrontEnd& f) { f.announce_step = 0.20f; }, [](FrontEnd& f) { f.announce_step = 0.0f; }},
  };
  for (size_t i = 0; i < sizeof(kChanges) / sizeof(kChanges[0]); i++) {
    // ---- the front-end announces something else in the middle of a scan: no verdict can come out of it
    startAndRun(100 + static_cast<uint32_t>(i));
    const uint32_t lines0 = r.app->storage().lines();
    r.takeSerial();
    kChanges[i].apply(t.fe);
    CHECK(r.run_until([&] { return !r.scanning(); }, 3000));      // announced within 1 s
    const std::string log = r.takeSerial();
    std::printf("  %s changed mid-scan -> state %d\n", kChanges[i].what, static_cast<int>(r.state()));
    CHECK(has(log, "# radar front-end changed its settings during the scan - scan aborted"));
    CHECK(r.state() == App::St::kMessage);
    CHECK(r.shows("RADAR CHANGED"));
    CHECK(r.app->storage().lines() == lines0);                    // nothing is logged for it
    CHECK(!has(log, "\"verdict\""));
    r.run_ms(10);                                                 // the LEDs follow on the next UI loop
    CHECK(stub::output(PIN_LED_CAND) == 0);
    CHECK(stub::output(PIN_LED_SCAN) == 0);
    kChanges[i].undo(t.fe);
    t.dismiss();
    CHECK(r.state() == App::St::kIdle);
    // ... and a scan after the front-end is back to what it was works normally
    r.run_ms(1500);
    CHECK(r.scanByButton());
    CHECK(r.verdict() == rs::Verdict::kCandidate);
    t.dismiss();
  }

  // ---- the front-end announced 20 bins but sends 16: every frame is rejected, the segment times out as a dropout
  {
    t.fe.announce_bins = 20;
    t.connect(iq::person(), 200);
    r.run_ms(1500);
    CHECK(r.app->sourceReady());
    r.takeSerial();
    CHECK(r.scanByButton(300000));
    const rs::ScanOutcome o = r.app->lastOutcome();
    CHECK(o.verdict == rs::Verdict::kInvalid);
    CHECK(o.last_invalid == rs::InvalidReason::kDropout);
    CHECK(o.n_valid == 0);
    t.fe.announce_bins = 0;
  }

  // ---- the geometry changes between two scans (nobody is scanning): the next scan simply uses the new one
  {
    t.dismiss();
    rs::SimScene s = iq::person();
    s.fs_hz = 25.0f;
    t.connect(s, 300);
    r.run_ms(1500);
    CHECK(r.scanByButton());
    CHECK(r.verdict() == rs::Verdict::kCandidate);
    CHECK(has(r.app->lastJson(), "\"fs_hz\":25.00"));
    s.fs_hz = 20.0f;
    t.connect(s, 301);
    r.run_ms(1500);
    CHECK(r.scanByButton());
    CHECK(r.verdict() == rs::Verdict::kCandidate);
    CHECK(has(r.app->lastJson(), "\"fs_hz\":20.00"));
  }
}

// ============================================================================ front-end timing
// The front-end stamps every frame with its own clock. If those stamps do not agree with the sequence
// numbers at the announced frame rate, every frequency (and the reported breathing rate) would be wrong,
// so the frames are flagged and the segment is rejected.
void fw_iq_timing() {
  stub::reset();
  Rig2 t(true);
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = false;
  CHECK(t.bringUp(iq::person(), 1));

  iq::reportHeader(std::string("### Serial I/Q source, front-end timestamps (") + iq::buildName() + ")",
                   "Person of amplitude 30. The announced frame rate is what the info packet says; the real rate is what the "
                   "front-end's clock produces. A mismatch of more than 2 % must be rejected, a smaller one is tolerated.");

  struct Case { const char* name; float real_fs; float announced_fs; bool no_stamps; bool valid; };
  const Case kCases[] = {
      {"honest front-end, 20 Hz", 20.0f, 0.0f, false, true},
      {"clock 1 % fast (20.2 Hz, announces 20)", 20.2f, 20.0f, false, true},
      {"clock 1 % slow (19.8 Hz, announces 20)", 19.8f, 20.0f, false, true},
      {"clock 3 % fast (20.6 Hz, announces 20)", 20.6f, 20.0f, false, false},
      {"clock 3 % slow (19.4 Hz, announces 20)", 19.4f, 20.0f, false, false},
      {"runs at 25 Hz, announces 20", 25.0f, 20.0f, false, false},
      {"runs at 20 Hz, announces 25", 20.0f, 25.0f, false, false},
      {"no timestamps (t_ms = 0): cannot be checked", 20.0f, 0.0f, true, true},
  };
  uint32_t seed = 4000;
  for (size_t i = 0; i < sizeof(kCases) / sizeof(kCases[0]); i++) {
    const Case& c = kCases[i];
    rs::SimScene sc = iq::person();
    sc.fs_hz = c.real_fs;
    t.fe.announce_fs = c.announced_fs;
    t.fe.no_stamps = c.no_stamps;
    const Tally k = iq::runScans(t, sc, 2, seed, true, c.valid);
    seed += 10;
    iq::report(c.name, k);
    const std::string j = r.app->lastJson();
    if (c.valid) {
      CHECK(k.c(rs::Verdict::kCandidate) == k.n);
      CHECK(k.timing == 0);
      CHECK(has(j, c.no_stamps ? "\"timestamps\":false" : "\"timestamps\":true"));
    } else {
      CHECK(k.c(rs::Verdict::kInvalid) == k.n);
      CHECK(k.why(rs::InvalidReason::kTiming) == k.n);
      CHECK(k.timing > 100);
      CHECK(r.app->lastOutcome().n_valid == 0);
      CHECK(r.app->lastOutcome().n_invalid == rs::kMaxInvalid);
      CHECK(has(j, "\"reason\":\"radar_timing\""));
      CHECK(has(j, "\"timestamps\":true"));
    }
    t.dismiss();
  }
  t.fe.announce_fs = 0.0f;
  t.fe.no_stamps = false;

  // ---- the operator sees why: result screen text in both languages, nothing that looks like a verdict
  {
    stub::gfx().enabled = true;
    rs::SimScene sc = iq::person();
    sc.fs_hz = 25.0f;
    t.fe.announce_fs = 20.0f;
    t.connect(sc, 4500);
    r.run_ms(1100);
    CHECK(r.scanByButton(900000));
    CHECK(r.verdict() == rs::Verdict::kInvalid);
    CHECK(r.shows("INVALID SCAN"));
    CHECK(r.shows("frame-rate"));
    const std::string st = t.cmd("status");
    CHECK(has(st, "timing=inconsistent"));
    t.dismiss();
    CHECK_OK(t.cmd("lang id"));
    CHECK(r.scanByButton(900000));
    CHECK(r.shows("laju frame"));
    t.dismiss();
    CHECK_OK(t.cmd("lang en"));
    t.fe.announce_fs = 0.0f;
    stub::gfx().enabled = false;
  }

  // ---- the front-end stops for 1.5 s in the middle of a segment and goes on with consecutive sequence
  // ---- numbers: no frame is "lost", yet the sampling grid has a hole. Only the timestamps show it.
  {
    t.connect(iq::person(), 4600);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    CHECK(r.scanning());
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 150 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
    t.fe.stop();
    r.run_ms(1500);
    t.fe.resume(stub::nowUs());
    CHECK(r.waitScanEnd());
    const rs::ScanOutcome o = r.app->lastOutcome();
    const std::string j = r.app->lastJson();
    std::printf("  front-end paused 1.5 s without a sequence gap: %s, valid=%d invalid=%d (%s), lost_frames=%ld, timing_flagged=%ld\n",
                rs::verdictName(o.verdict), o.n_valid, o.n_invalid, rs::invalidReasonName(o.last_invalid), jsonInt(j, "lost_frames"),
                jsonInt(j, "timing_flagged"));
    CHECK(jsonInt(j, "lost_frames") == 0);
    CHECK(jsonInt(j, "timing_flagged") > 100);
    CHECK(o.n_invalid >= 1);                         // the damaged segment is not analysed ...
    CHECK(o.last_invalid == rs::InvalidReason::kTiming);
    CHECK(o.verdict == rs::Verdict::kCandidate && o.n_valid >= 2);   // ... the clean ones that follow still are
    checkScanInvariants(r, o, 16);
    t.dismiss();
  }
}

// ============================================================================ console + I/Q on the same UART
void fw_iq_console() {
  stub::reset();
  Rig2 t(true);
  t.boot();
  Rig& r = t.r;
  stub::gfx().enabled = false;
  CHECK(t.bringUp(iq::person(), 1));
  r.run_ms(1100);
  r.takeSerial();

  // ---- commands during a running scan
  t.connect(iq::person(), 7);
  r.run_ms(1100);
  r.press(PIN_BTN_SCAN);
  CHECK(r.scanning());
  CHECK(r.run_until([&] { return r.app->acq().status().frames > 100 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
  {
    const std::string st = t.cmd("status");
    CHECK(replyOk(st));
    CHECK(has(st, "state=scanning"));
    CHECK(has(st, "source=SERIAL"));
    CHECK(iq::statusField(st, "frames") > 50);
  }
  {
    const std::string o = t.cmd("src sim");
    CHECK(replyErr(o));
    CHECK(r.app->settings().get().source == SourceKind::kSerial);
  }
  {
    const std::string o = t.cmd("set pfa 0.02");
    CHECK(replyErr(o));
    CHECK(has(o, "not while a scan is running"));
  }
  {
    const std::string o = t.cmd("abort");
    CHECK(replyOk(o));
    CHECK(has(o, "# scan aborted"));
    CHECK(r.state() == App::St::kIdle);
  }
  // the stream keeps flowing, the next scan is clean
  r.run_ms(1100);
  CHECK_OK(t.cmd("scan"));
  CHECK(r.waitScanEnd());
  CHECK(r.verdict() == rs::Verdict::kCandidate);
  CHECK(jsonInt(r.app->lastJson(), "lost_frames") == 0);

  // ---- what an unknown line does depends on whether the I/Q stream shares the console UART
  t.dismiss();
  {
    const std::string o = t.cmd("zz");
#if RS_IQ_ON_UART0
    CHECK(o.empty());                                            // a printable fragment of the binary stream: no noise on the console
#else
    CHECK(replyErr(o));
    CHECK(has(o, "unknown command"));
#endif
    const std::string bad = t.cmd("preset 99");                  // a known command with bad arguments is always answered
    CHECK(replyErr(bad));
  }

#if RS_IQ_ON_UART0
  // ---- one whole scan with I/Q frames and an operator typing along: no console noise, every command answered
  {
    r.takeSerial();
    t.connect(iq::person(), 8);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    int asked = 0, answered = 0;
    uint32_t guard = 0;
    while (r.scanning() && guard++ < 200) {
      r.run_ms(500);
      if (guard % 10 == 0) {
        asked++;
        const std::string o = t.cmd("status", 100);
        if (replyOk(o)) answered++;
      }
    }
    const std::string all = r.takeSerial();
    CHECK(!r.scanning());
    CHECK(asked >= 3);
    CHECK(answered == asked);
    CHECK(countOf(all, "ERR unknown command") == 0);
    CHECK(r.verdict() == rs::Verdict::kCandidate);
  }
  // ---- with the simulator selected the same line IS answered (the console is not muted outside the I/Q mode)
  t.dismiss();
  CHECK_OK(t.cmd("src sim"));
  {
    const std::string o = t.cmd("zz");
    CHECK(replyErr(o));
  }
  CHECK_OK(t.cmd("src serial"));
  CHECK(t.waitReady(2500));

  // ---- a careless tool types in the middle of a packet: that line is discarded (never half-executed),
  // ---- one frame is lost, and the next command works
  {
    r.takeSerial();
    t.connect(iq::person(), 9);
    r.run_ms(1100);
    r.press(PIN_BTN_SCAN);
    CHECK(r.scanning());
    CHECK(r.run_until([&] { return r.app->acq().status().frames > 100 && r.app->acq().status().mode == AcqStatus::kRunning; }, 40000));
    bool inserted = false;
    for (int i = 0; i < 2000 && !inserted; i++) {
      r.ms();
      if (!t.fe.q.empty() && t.fe.q.size() < 60) {               // part of a packet is out, the rest is still queued
        const std::string cmdline = "abort\n";
        for (size_t k = cmdline.size(); k > 0; k--) t.fe.q.push_front(static_cast<uint8_t>(cmdline[k - 1]));
        inserted = true;
      }
    }
    CHECK(inserted);
    r.run_ms(300);
    CHECK(r.scanning());                                         // "abort" was NOT executed
    const std::string o = t.cmd("status");
    CHECK(replyOk(o));
    CHECK(r.waitScanEnd());
    CHECK(jsonInt(r.app->lastJson(), "lost_frames") >= 1);
    CHECK(r.verdict() == rs::Verdict::kCandidate);
  }
#endif
}
