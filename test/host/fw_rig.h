// Firmware test rig: runs the unmodified firmware sources (src/*.cpp) on the host under a
// virtual clock, with the Arduino / ESP32 stubs from test/stubs.
//
//   Rig       the "device": boots an App, steps virtual time (acquisition task every 1 ms,
//             UI loop every 5 ms - the same cadence as on the ESP32), presses buttons, types
//             console commands, waits for scans
//   FrontEnd  an external radar front-end MCU (or a PC replaying a recording) that talks the
//             rs_proto serial protocol to the device, with switchable faults
#pragma once
#include <stdint.h>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "app.h"
#include "rs_pins.h"
#include "rs_proto.h"
#include "rs_simgen.h"
#include "rs_test.h"
#include "stub_hw.h"

// ---------------------------------------------------------------------------- small helpers
inline bool has(const std::string& hay, const char* needle) { return hay.find(needle) != std::string::npos; }

inline int countOf(const std::string& hay, const char* needle) {
  int n = 0;
  const size_t len = std::string(needle).size();
  for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + len)) n++;
  return n;
}

// Last non-empty line of a console reply.
inline std::string lastLine(const std::string& s) {
  size_t e = s.size();
  while (e > 0 && (s[e - 1] == '\n' || s[e - 1] == '\r')) e--;
  size_t b = s.rfind('\n', e ? e - 1 : 0);
  b = (b == std::string::npos) ? 0 : b + 1;
  return s.substr(b, e - b);
}

inline bool replyOk(const std::string& s) { return lastLine(s) == "OK"; }
inline bool replyErr(const std::string& s) { return lastLine(s).compare(0, 3, "ERR") == 0; }

// Console replies that must be OK / ERR; the reply is printed when the expectation fails.
#define CHECK_OK(expr)                                                       \
  do { const std::string _rep = (expr);                                      \
       if (!replyOk(_rep)) std::printf("  reply was: [%s]\n", _rep.c_str());  \
       CHECK(replyOk(_rep)); } while (0)
#define CHECK_ERR(expr)                                                      \
  do { const std::string _rep = (expr);                                      \
       if (!replyErr(_rep)) std::printf("  reply was: [%s]\n", _rep.c_str()); \
       CHECK(replyErr(_rep)); } while (0)

inline std::string lower(std::string s) {
  for (size_t i = 0; i < s.size(); i++)
    if (s[i] >= 'A' && s[i] <= 'Z') s[i] = static_cast<char>(s[i] - 'A' + 'a');
  return s;
}

// ---------------------------------------------------------------------------- device rig
struct Rig {
  std::unique_ptr<App> app;
  bool began = false;
  uint32_t ms_count = 0;
  std::vector<std::function<void(uint64_t)> > tickers;      // run once per simulated millisecond, before the firmware

  Rig() {}
  ~Rig() { stub::clearBackground(); }

  void boot() {
    stub::clearRestart();
    app.reset(new App());
    began = app->begin();                                    // like the device: the acquisition task starts afterwards
    if (began) stub::setBackground(&Rig::bg, this, 1000);   // main.cpp creates it only if begin() succeeded
  }
  void shutdown() {
    stub::clearBackground();
    app.reset();
    began = false;
  }
  // Power cycle: RAM is lost, NVS and the file system survive.
  void powerCycle() {
    shutdown();
    stub::softReset();
    ms_count = 0;
    boot();
  }

  // The acquisition task does not run for `n` ms (a long flash operation, a priority inversion): serial
  // bytes pile up while the UI thread keeps going.
  void starve_ms(uint32_t n) {
    stub::clearBackground();
    run_ms(n);
    if (began) stub::setBackground(&Rig::bg, this, 1000);
  }

  static void bg(void* c) {
    Rig* r = static_cast<Rig*>(c);
    if (r->app) r->app->backgroundStep();
  }

  void ms() {
    const uint64_t t = stub::nowUs();
    for (size_t i = 0; i < tickers.size(); i++) tickers[i](t);
    stub::advanceUs(1000);
    ms_count++;
    if (app && (ms_count % 5) == 0) app->update();
  }
  void run_ms(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) ms();
  }
  bool run_until(const std::function<bool()>& pred, uint32_t timeout_ms) {
    for (uint32_t i = 0; i < timeout_ms; i++) {
      if (pred()) return true;
      ms();
    }
    return pred();
  }

  App::St state() const { return app->state(); }
  bool scanning() const { return state() == App::St::kScanning || state() == App::St::kCalibrating; }
  bool waitScanEnd(uint32_t timeout_ms = 900000) {
    return run_until([this] { return !scanning(); }, timeout_ms);
  }

  // ---- inputs
  void hold(uint8_t pin, uint32_t hold_ms) {
    stub::setInput(pin, 0);
    run_ms(hold_ms);
    stub::setInput(pin, 1);
    run_ms(60);
  }
  void press(uint8_t pin) { hold(pin, 90); }
  void longPress(uint8_t pin) { hold(pin, 1100); }

  // Types one console line; returns everything the device printed meanwhile.
  std::string say(const char* line, uint32_t wait_ms = 80) {
    Serial.clearTx();
    const std::string l = std::string(line) + "\n";
    Serial.injectRx(reinterpret_cast<const uint8_t*>(l.data()), l.size());
    run_ms(wait_ms);
    const std::string out = Serial.tx();
    Serial.clearTx();
    return out;
  }
  std::string takeSerial() {
    const std::string out = Serial.tx();
    Serial.clearTx();
    return out;
  }

  // ---- display
  std::string screen() { return stub::gfx().flatText(); }
  bool shows(const char* needle) { return lower(screen()).find(lower(needle)) != std::string::npos; }

  // ---- scans (simulated source)
  // Presses the scan button and runs to the end of the scan. False if no scan started or it never ended.
  bool scanByButton(uint32_t timeout_ms = 900000) {
    press(PIN_BTN_SCAN);
    if (!scanning()) return false;
    return waitScanEnd(timeout_ms);
  }
  rs::Verdict verdict() const { return app->lastOutcome().verdict; }
};

// Per-scan invariants that must hold whatever the verdict is (scan finished, result on screen).
inline void checkScanInvariants(Rig& r, const rs::ScanOutcome& o, int n_bins = 16) {
  r.run_ms(10);                         // the indicator pins follow the state on the next UI loop (5 ms)
  CHECK(r.state() == App::St::kResult);
  CHECK(o.done);
  CHECK(o.verdict != rs::Verdict::kNone);
  const std::string json = r.app->lastJson();
  CHECK(!json.empty() && json[0] == '{' && json[json.size() - 1] == '}');
  CHECK(has(json, "\"safety_notice\""));
  if (o.verdict == rs::Verdict::kNoPattern) CHECK(has(json, "No reliable life-sign pattern detected in this scan. DO NOT clear area."));
  // sensitivity statement: unknown without a baseline, otherwise "low" exactly when the floor is raised, for every
  // verdict; a negative at a raised floor must have been downgraded
  CHECK(has(json, "\"sensitivity\":\"unknown\"") == !o.sens_known);
  if (o.sens_known) CHECK(has(json, "\"sensitivity\":\"low\"") == o.low_sens);
  if (o.verdict == rs::Verdict::kNoPattern) CHECK(!o.low_sens);
  if (o.verdict == rs::Verdict::kLowSensitivity) CHECK(o.low_sens);
  // indicators: green only for a candidate; a negative lights nothing
  CHECK((stub::output(PIN_LED_CAND) == 1) == (o.verdict == rs::Verdict::kCandidate));
  const bool amber = o.verdict == rs::Verdict::kPossible || o.verdict == rs::Verdict::kConfounded ||
                     o.verdict == rs::Verdict::kLowSensitivity || o.verdict == rs::Verdict::kInvalid;
  CHECK((stub::output(PIN_LED_UNCERT) == 1) == amber);
  CHECK(stub::output(PIN_LED_SCAN) == 0);
  if (o.verdict == rs::Verdict::kCandidate) {
    CHECK(o.agree >= 2);
    CHECK(o.hz >= 0.10f && o.hz <= 0.80f);
    CHECK_NEAR(o.bpm, o.hz * 60.0f, 0.2);
    CHECK(o.bin >= 2 && o.bin < n_bins);
  }
}

// Integer value of a numeric JSON key ("lost_frames":12 -> 12); -1 if absent.
inline long jsonInt(const std::string& json, const char* key) {
  const std::string k = std::string("\"") + key + "\":";
  const size_t p = json.find(k);
  if (p == std::string::npos) return -1;
  return std::strtol(json.c_str() + p + k.size(), nullptr, 10);
}

// ---------------------------------------------------------------------------- front-end model
struct FrontEnd {
  rs::SimRadar     radar;
  rs::ImuProcessor imu;                  // an IMU mounted with the radar (only used when host_imu)
  HardwareSerial*  port = nullptr;
  uint32_t         baud = 460800;

  bool    streaming = false;
  bool    announce = true;               // periodic info packets (1 / s)
  bool    host_imu = false;              // frames carry kProtoFlagHostImu (+ kProtoFlagMotion from the model IMU)
  uint8_t extra_flags = 0;
  bool    stamp_clock = true;            // t_ms = the front-end's own clock when the frame was generated (like millis())
  bool    no_stamps = false;             // a front-end that does not implement timestamps: t_ms = 0
  int     drop_every = 0;                // generated but never sent: a sequence gap at the receiver
  int     corrupt_every = 0;             // flips one random byte inside every N-th packet
  int     dup_every = 0;                 // sends every N-th packet twice
  int     garbage_every = 0;             // 1..40 random bytes before every N-th packet
  uint8_t announce_bins = 0;             // 0 = what the scene delivers
  float   announce_fs = 0.0f;            // 0 = what the scene delivers
  float   announce_step = 0.0f;          // range step (m), 0 = what the scene delivers
  uint32_t seed_state = 12345;

  uint64_t next_frame_us = 0, next_info_us = 0;
  std::deque<uint8_t> q;
  double   credit = 0.0;
  uint32_t frames_generated = 0, frames_sent = 0;

  uint32_t rnd() {
    seed_state = seed_state * 1664525u + 1013904223u;
    return seed_state >> 8;
  }

  void attach(HardwareSerial* p, uint32_t b) {
    port = p;
    baud = b;
  }

  // Begin streaming a scene. The model IMU is settled first, like a real unit that has been on for a while.
  void start(const rs::SimScene& sc, uint32_t seed, uint64_t now_us) {
    radar.scene = sc;
    radar.reset(seed);
    imu.configure(rs::ImuGateParams(), 100.0f, sc.fs_hz);
    rs::RadarFrame f;
    for (int i = 0; i < 40; i++) {
      radar.step(f);
      radar.imuStep(imu);
      imu.takeFrame();
    }
    streaming = true;
    next_frame_us = now_us;
    next_info_us = now_us;
    frames_generated = frames_sent = 0;
    q.clear();
    credit = 0.0;
  }
  void stop() { streaming = false; }
  // The device's virtual clock restarts at 0 after a power cycle; the front-end is a separate unit that
  // keeps running, so its schedule is moved onto the new time base (stream and sequence numbers go on).
  void rebase(uint64_t now_us) {
    next_frame_us = now_us;
    next_info_us = now_us;
  }
  // Streaming resumes after a pause (no catch-up burst of the frames that were not generated meanwhile).
  void resume(uint64_t now_us) {
    streaming = true;
    next_frame_us = now_us;
  }
  // The front-end MCU resets (brown-out, watchdog): a new scene draw, sequence number and timestamps restart at 0.
  void restart(uint32_t seed) { radar.reset(seed); }
  // Bytes that are not part of a packet but arrive between two packets, e.g. a console line an operator
  // types on a UART that also carries the I/Q stream. The queue only ever holds whole packets, so this
  // lands exactly on a packet boundary (what a careful PC tool does).
  void queueText(const std::string& s) {
    for (size_t i = 0; i < s.size(); i++) q.push_back(static_cast<uint8_t>(s[i]));
  }

  void queuePacket(const uint8_t* b, size_t n) {
    for (size_t i = 0; i < n; i++) q.push_back(b[i]);
  }

  void sendInfo() {
    rs::RadarInfo info = radar.info();
    if (announce_bins) info.n_bins = announce_bins;
    if (announce_fs > 0.0f) info.frame_rate_hz = announce_fs;
    if (announce_step > 0.0f) info.range_step_m = announce_step;
    uint8_t buf[64];
    const size_t n = rs::encodeInfo(info, buf, sizeof(buf));
    queuePacket(buf, n);
  }

  void tick(uint64_t now_us) {
    if (!port) return;
    if (announce && now_us >= next_info_us) {
      sendInfo();
      next_info_us = now_us + 1000000u;
    }
    if (streaming && now_us >= next_frame_us) {
      const uint64_t period = static_cast<uint64_t>(1.0e6 / static_cast<double>(radar.scene.fs_hz));
      const uint64_t generated_us = next_frame_us;
      next_frame_us += period;
      rs::RadarFrame f;
      radar.step(f);
      if (no_stamps) f.t_ms = 0;
      else if (stamp_clock) f.t_ms = static_cast<uint32_t>(generated_us / 1000u);
      frames_generated++;
      uint8_t flags = extra_flags;
      if (host_imu) {
        radar.imuStep(imu);
        const rs::ImuStats st = imu.takeFrame();
        flags |= rs::kProtoFlagHostImu;
        if (st.motion) flags |= rs::kProtoFlagMotion;
      }
      const bool skip = drop_every > 0 && (frames_generated % static_cast<uint32_t>(drop_every)) == 0;
      if (!skip) {
        uint8_t buf[rs::kProtoMaxPacket];
        const size_t n = rs::encodeFrame(f, buf, sizeof(buf), flags);
        if (garbage_every > 0 && (frames_generated % static_cast<uint32_t>(garbage_every)) == 0) {
          const uint32_t g = 1 + rnd() % 40;
          for (uint32_t i = 0; i < g; i++) q.push_back(static_cast<uint8_t>(rnd()));
        }
        if (corrupt_every > 0 && (frames_generated % static_cast<uint32_t>(corrupt_every)) == 0)
          buf[2 + rnd() % (n - 2)] ^= static_cast<uint8_t>(1u << (rnd() % 8));
        queuePacket(buf, n);
        if (dup_every > 0 && (frames_generated % static_cast<uint32_t>(dup_every)) == 0) queuePacket(buf, n);
        frames_sent++;
      }
    }
    // bytes the UART can carry in this millisecond
    credit += static_cast<double>(baud) / 10000.0;
    std::string chunk;
    while (!q.empty() && credit >= 1.0) {
      chunk.push_back(static_cast<char>(q.front()));
      q.pop_front();
      credit -= 1.0;
    }
    if (q.empty() && credit > static_cast<double>(baud) / 10000.0) credit = static_cast<double>(baud) / 10000.0;
    if (!chunk.empty()) port->injectRx(reinterpret_cast<const uint8_t*>(chunk.data()), chunk.size());
  }
};
