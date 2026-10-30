#include "app.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <new>
#include <stdio.h>
#include <string.h>
#include "params_cli.h"
#include "rs_build.h"
#include "rs_pins.h"
#include "rs_text.h"

using namespace txt;

namespace {
constexpr uint32_t kUiRefreshMs = 250;       // scan screen
constexpr uint32_t kIdleRefreshMs = 400;     // idle status rows
constexpr uint32_t kPotPeriodMs = 100;
constexpr uint32_t kPreviewMinMs = 700;
constexpr uint16_t kPreviewFrames = 64;      // minimum new frames between two previews
constexpr uint32_t kAnalysisTimeoutMs = 15000;
constexpr uint32_t kMessageMs = 3000;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

// ============================================================================ construction

App::App()
    : tft_(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST),
      ui_(tft_),
      sim_(RS_SIM_WARP_DEFAULT),
      ser_(pump_.iq()),
      console_(*this, Serial) {
  json_[0] = '\0';
}

App::~App() {
  delete buf_;
  delete ws_;
}

IRadarSource* App::activeSource() {
  if (settings_.get().source == SourceKind::kSerial) return &ser_;
  return &sim_;
}

const char* App::sourceTag() const { return settings_.get().source == SourceKind::kSerial ? "SERIAL" : "SIM"; }

// The UART bytes feed the I/Q ring only while the SERIAL source is selected. When that stream shares
// the console UART, the console must also stay quiet about the binary fragments it cannot parse.
void App::routeInput(SourceKind k) {
  const bool serial = (k == SourceKind::kSerial);
  pump_.setIqEnabled(serial);
  console_.setQuietUnknown(serial && RS_IQ_ON_UART0 != 0);
}

void App::printBanner() {
  Serial.println();
  Serial.println("# ============================================================");
  Serial.print("# RubbleSense SAR aid - research prototype, firmware ");
  Serial.println(RS_FW_VERSION);
  Serial.println("# SAFETY NOTICE: support tool only. A negative scan never means");
  Serial.println("# that nobody is there. Never use it to declare an area clear.");
  Serial.println("# Not validated on real rubble. Type 'help' for commands.");
  Serial.println("# ============================================================");
}

bool App::begin() {
  Serial.setRxBufferSize(2048);          // the console and, in the UART0 build, the I/Q stream (RS_IQ_RX_RING)
  Serial.begin(RS_SERIAL_BAUD);
  printBanner();

  buttons_.begin();
  alerts_.begin();
  settings_ok_ = settings_.begin();
  Settings& s = settings_.get();
  txt::setLang(s.lang == 1 ? Lang::ID : Lang::EN);
  alerts_.setSound(s.sound);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  imu_ok_ = imu_.begin(Wire);
  if (imu_ok_) {
    Serial.print("# IMU: MPU6050-class sensor at 0x");
    Serial.println(imu_.address(), HEX);
  } else {
    Serial.println(sim_.canSimulateImu() && s.source == SourceKind::kSim
                       ? "# IMU: not found - the simulator supplies a simulated IMU"
                       : "# IMU: NOT FOUND - scans will be rejected (fail-safe)");
  }
  const bool fs_ok = storage_.begin();
  Serial.println(fs_ok ? "# Log: LittleFS mounted" : "# Log: LittleFS unavailable - results are only printed");

  ui_.begin();

  buf_ = new (std::nothrow) rs::SegmentBuffer();
  ws_ = new (std::nothrow) rs::Workspace();
  if (!buf_ || !ws_) {
    fatal("out of memory");
    return false;
  }

  pump_.begin();
  sim_.setPreset(s.preset);
  ser_.begin();
  dsp_.begin(buf_, ws_);
  acq_.attach(activeSource(), &imu_, buf_);
  configureAcq();
  routeInput(s.source);

  enterIdle();
  return true;
}

void App::fatal(const char* what) {
  Serial.print("# FATAL: ");
  Serial.println(what);
  ui_.drawMessage("FATAL ERROR", what, col::kDkRed, "");
  alerts_.leds(false, true);
  alerts_.play(Beep::kError);
  st_ = St::kFatal;
}

// ============================================================================ acquisition glue

void App::configureAcq() {
  rs::RadarInfo info;
  const rs::ImuGateParams gate = rs::ImuGateParams();
  if (!activeSource()->info(info)) {                    // geometry not known yet (a serial front-end that is still silent)
    info = rs::RadarInfo();
    info.name = "none";
    info.frame_rate_hz = 20.0f;
    info.range_step_m = 0.1f;
    info.wavelength_mm = 125.0f;
    info.n_bins = 16;
  }
  acq_.configure(gate, info);
}

void App::backgroundStep() {
  pump_.pump();
  acq_.service(micros());
}

void App::waitAcqIdle() {
  for (int i = 0; i < 100; i++) {
    if (acq_.status().mode == AcqStatus::kIdle) return;
    delay(1);
  }
}

// ============================================================================ baseline

bool App::baselineKnown(float& db, bool& sim_default) const {
  const Settings& s = settings_.get();
  sim_default = false;
  if (s.source == SourceKind::kSim) {
    if (s.bl_sim_ok) { db = s.bl_sim_db; return true; }
    db = RS_SIM_DEFAULT_BASELINE_DB;
    sim_default = true;
    return true;
  }
  if (s.bl_ser_ok) { db = s.bl_ser_db; return true; }
  return false;
}

void App::baselineFor(rs::DspParams& p, const char*& src_name) const {
  float db = 0.0f;
  bool def = false;
  p.have_baseline = baselineKnown(db, def);
  p.baseline_floor_db = p.have_baseline ? db : 0.0f;
  src_name = !p.have_baseline ? "none" : (def ? "sim_default" : "calibrated");
}

// ============================================================================ idle screen

ImuState App::imuState(const AcqStatus& s) const {
  if (s.imu_host) return ImuState::kHost;
  if (s.imu_hw) return s.motion ? ImuState::kMoving : ImuState::kOk;
  if (s.imu_sim) return ImuState::kSimulated;
  return ImuState::kMissing;
}

IdleView App::idleView(const char* message) {
  IdleView v;
  const Settings& s = settings_.get();
  v.source_tag = sourceTag();
  v.sim = (s.source == SourceKind::kSim);
  v.trial_name = presetName(s.preset);
  v.manual = (s.preset == kPresetManual);
  v.serial_ready = ser_.ready();
  v.imu = imuState(acq_.status());
  float db = 0.0f;
  bool def = false;
  v.baseline_known = baselineKnown(db, def);
  v.baseline_db = db;
  v.baseline_simdef = def;
  v.log_ok = storage_.ok();
  v.log_lines = storage_.lines();
  v.sound = s.sound;
  v.message = message;
  return v;
}

void App::drawIdle() {
  const IdleView v = idleView(message_);
  ui_.drawIdle(v);
}

void App::enterIdle(const char* message) {
  st_ = St::kIdle;
  strncpy(message_, message, sizeof(message_) - 1);
  message_[sizeof(message_) - 1] = '\0';
  message_until_ = message[0] ? millis() + kMessageMs : 0;
  alerts_.leds(false, false);
  alerts_.scanLed(ScanLed::kBlink);
  drawIdle();
  idle_ms_ = millis();
}

void App::refreshIdle(uint32_t now) {
  if (static_cast<uint32_t>(now - idle_ms_) < kIdleRefreshMs) return;
  idle_ms_ = now;
  if (message_[0] && message_until_ && rsReached(now, message_until_)) { message_[0] = '\0'; message_until_ = 0; }
  const IdleView v = idleView(message_);
  ui_.updateIdleStatus(v);
}

void App::pollManualPots(uint32_t now) {
  if (settings_.get().source != SourceKind::kSim || settings_.get().preset != kPresetManual) return;
  if (static_cast<uint32_t>(now - pot_ms_) < kPotPeriodMs) return;
  pot_ms_ = now;
  const float a = static_cast<float>(analogRead(PIN_POT_TARGET)) / 4095.0f;
  const float n = static_cast<float>(analogRead(PIN_POT_NOISE)) / 4095.0f;
  if (!pots_init_) { pot_amp_ = a; pot_noise_ = n; pots_init_ = true; }
  pot_amp_ += 0.3f * (a - pot_amp_);                    // light smoothing against ADC jitter
  pot_noise_ += 0.3f * (n - pot_noise_);
  sim_.setManual(60.0f * clampf(pot_amp_, 0.0f, 1.0f), 4.0f + 28.0f * clampf(pot_noise_, 0.0f, 1.0f));
}

// ============================================================================ update loop

void App::feedConsole() {
  uint8_t b;
  int n = 0;
  while (n < 256 && pump_.console().pop(b)) {
    console_.feed(b);
    n++;
  }
}

void App::update() {
  if (st_ == St::kFatal) return;
  const uint32_t now = millis();
  buttons_.update(now);
  alerts_.update(now);
  feedConsole();

  BtnEvent e;
  while (buttons_.pop(e)) handleButton(e);

  pollManualPots(now);
  if (st_ == St::kIdle) refreshIdle(now);
  else if (st_ == St::kScanning || st_ == St::kCalibrating) stepScan(now);
}

// ============================================================================ buttons

void App::handleButton(const BtnEvent& e) {
  const bool is_short = (e.evt == BtnEvt::kShort);
  switch (st_) {
    case St::kIdle:
    case St::kResult:
      if (!is_short) return;
      alerts_.play(Beep::kKey);
      if (e.id == BtnId::kScan) startScan(false);
      else if (e.id == BtnId::kMode) {
        const uint8_t next = static_cast<uint8_t>((settings_.get().preset + 1) % kNumPresets);
        selectPreset(next);
      } else if (e.id == BtnId::kMenu) {
        menu_sel_ = 0;                                   // a fresh menu starts at the top (back from a sub-screen keeps the item)
        openMenu();
      }
      break;

    case St::kScanning:
    case St::kCalibrating:
      if (is_short && e.id == BtnId::kScan) abortScan();
      break;

    case St::kMenu:
      if (!is_short) return;
      alerts_.play(Beep::kKey);
      if (e.id == BtnId::kMode) { menu_sel_ = static_cast<uint8_t>((menu_sel_ + 1) % 7); drawMenu(); }
      else if (e.id == BtnId::kScan) activateMenu();
      else enterIdle();
      break;

    case St::kSelfTest:
    case St::kAbout:
      if (is_short) { alerts_.play(Beep::kKey); openMenu(); }
      break;

    case St::kMessage:
      if (is_short) { alerts_.play(Beep::kKey); enterIdle(); }
      break;

    case St::kFatal:
      break;
  }
}

// ============================================================================ menu

void App::openMenu() {
  st_ = St::kMenu;
  alerts_.scanLed(ScanLed::kOff);
  alerts_.leds(false, false);
  drawMenu();
}

void App::drawMenu() {
  const Settings& s = settings_.get();
  MenuView v;
  v.source_tag = sourceTag();
  v.n = 7;
  v.sel = menu_sel_;
  snprintf(v.item[0], sizeof(v.item[0]), "%s: %s", t(T_MENU_SOURCE), s.source == SourceKind::kSim ? "SIM" : "SERIAL");
  snprintf(v.item[1], sizeof(v.item[1]), "%s", t(T_MENU_CALIB));
  snprintf(v.item[2], sizeof(v.item[2]), "%s", t(T_MENU_CLEARBL));
  snprintf(v.item[3], sizeof(v.item[3]), "%s: %s", t(T_MENU_LANG), s.lang == 1 ? "ID" : "EN");
  snprintf(v.item[4], sizeof(v.item[4]), "%s: %s", t(T_MENU_SOUND), s.sound ? t(T_ON) : t(T_OFF));
  snprintf(v.item[5], sizeof(v.item[5]), "%s", t(T_MENU_SELFTEST));
  snprintf(v.item[6], sizeof(v.item[6]), "%s", t(T_MENU_ABOUT));
  ui_.drawMenu(v);
}

void App::activateMenu() {
  Settings& s = settings_.get();
  switch (menu_sel_) {
    case 0:
      selectSource(s.source == SourceKind::kSim ? SourceKind::kSerial : SourceKind::kSim);
      drawMenu();
      break;
    case 1:
      startScan(true);
      break;
    case 2:
      conClearBaseline(Serial);
      drawMenu();
      break;
    case 3:
      s.lang = (s.lang == 1) ? 0 : 1;
      txt::setLang(s.lang == 1 ? Lang::ID : Lang::EN);
      settings_.saveCore();
      drawMenu();
      break;
    case 4:
      s.sound = !s.sound;
      alerts_.setSound(s.sound);
      settings_.saveCore();
      drawMenu();
      break;
    case 5: {
      SelfTestView v;
      buildSelfTest(v);
      ui_.drawSelfTest(v);
      alerts_.play(Beep::kDone);
      st_ = St::kSelfTest;
      break;
    }
    case 6:
      ui_.drawAbout(sourceTag());
      st_ = St::kAbout;
      break;
    default:
      break;
  }
}

void App::buildSelfTest(SelfTestView& v) {
  v = SelfTestView();
  v.source_tag = sourceTag();
  uint8_t n = 0;

  // IMU
  snprintf(v.name[n], sizeof(v.name[n]), "%s", t(T_ST_IMU));
  if (imu_.present()) {
    const float g = imu_.gravityMagnitude();
    v.ok[n] = g > 8.0f && g < 11.6f;
    snprintf(v.detail[n], sizeof(v.detail[n]), "|a|=%.2f m/s2", static_cast<double>(g));
  } else {
    v.ok[n] = false;
    snprintf(v.detail[n], sizeof(v.detail[n]), "not found");
  }
  n++;

  // storage
  snprintf(v.name[n], sizeof(v.name[n]), "%s", t(T_ST_STORAGE));
  v.ok[n] = storage_.ok();
  snprintf(v.detail[n], sizeof(v.detail[n]), "%lu B", static_cast<unsigned long>(storage_.size()));
  n++;

  // settings
  snprintf(v.name[n], sizeof(v.name[n]), "Settings (NVS)");
  v.ok[n] = settings_ok_;
  v.detail[n][0] = '\0';
  n++;

  // radar source
  snprintf(v.name[n], sizeof(v.name[n]), "%s", t(T_ST_RADAR));
  rs::RadarInfo info;
  IRadarSource* src = activeSource();
  if (src->ready() && src->info(info)) {
    v.ok[n] = true;
    snprintf(v.detail[n], sizeof(v.detail[n]), "%s %.0fHz %ubins", src->name(), static_cast<double>(info.frame_rate_hz),
             static_cast<unsigned>(info.n_bins));
  } else {
    v.ok[n] = false;
    snprintf(v.detail[n], sizeof(v.detail[n]), "%s no info", src->name());
  }
  n++;

  // memory
  snprintf(v.name[n], sizeof(v.name[n]), "%s", t(T_ST_MEM));
  const uint32_t heap = ESP.getFreeHeap();
  v.ok[n] = heap > 40u * 1024u;
  snprintf(v.detail[n], sizeof(v.detail[n]), "%lu KB", static_cast<unsigned long>(heap / 1024u));
  n++;

  v.n = n;
}

// ============================================================================ selection

bool App::selectSource(SourceKind k) {
  if (st_ != St::kIdle && st_ != St::kResult && st_ != St::kMenu) return false;
  Settings& s = settings_.get();
  s.source = k;
  IRadarSource* src = activeSource();
  routeInput(k);
  acq_.setSource(src);                                  // the acquisition thread activates it (drops stale state)
  configureAcq();
  settings_.saveCore();
  return true;
}

bool App::selectPreset(uint8_t idx) {
  if (st_ != St::kIdle && st_ != St::kResult) return false;
  Settings& s = settings_.get();
  if (idx >= kNumPresets) return false;
  s.preset = idx;
  sim_.setPreset(s.preset);
  settings_.saveCore();
  char m[48];
  snprintf(m, sizeof(m), "%s", presetName(s.preset));
  enterIdle(s.source == SourceKind::kSim ? m : "");
  return true;
}

// ============================================================================ scanning

bool App::startScan(bool calibration) {
  if (st_ != St::kIdle && st_ != St::kResult && st_ != St::kMenu) return false;
  Settings& s = settings_.get();
  IRadarSource* src = activeSource();

  rs::RadarInfo info;
  if (!src->ready() || !src->info(info)) {
    Serial.println("# scan refused: radar source not ready");
    alerts_.play(Beep::kError);
    if (st_ == St::kMenu) showMessage("NOT READY", t(T_MSG_NOT_READY), col::kDkRed);
    else enterIdle(t(T_MSG_NOT_READY));
    return false;
  }

  params_ = s.params;
  params_.fs_hz = info.frame_rate_hz;
  baselineFor(params_, baseline_src_);
  const char* why = nullptr;
  if (!paramsValid(params_, info.frame_rate_hz, &why) || params_.min_bin >= info.n_bins) {
    Serial.print("# scan refused: invalid DSP configuration: ");
    Serial.println(why ? why : "min_bin beyond the delivered range bins");
    alerts_.play(Beep::kError);
    showMessage("CONFIG", why ? why : "min_bin beyond the delivered range bins", col::kDkRed);
    return false;
  }

  info_ = info;
  if (!calibration) {
    s.scan_counter++;
    scan_id_ = s.scan_counter;
  } else {
    scan_id_ = 0;
  }
  if (src == &sim_) sim_.restart(1000u + (calibration ? 7777u : scan_id_));   // a new random scene: the operator "re-positions"

  session_.begin(params_);
  acq_.configure(rs::ImuGateParams(), info);
  waitAcqIdle();
  acq_.requestArm(true);

  st_ = calibration ? St::kCalibrating : St::kScanning;
  seg_no_ = 1;
  analyzing_ = false;
  preview_pending_ = false;
  last_preview_frames_ = 0;
  last_preview_ms_ = millis();
  t_scan_ms_ = millis();
  ui_ms_ = 0;
  alerts_.leds(false, false);
  alerts_.scanLed(ScanLed::kOn);
  alerts_.play(Beep::kStart);

  Serial.print("# ");
  if (calibration) Serial.println("calibration started");
  else {
    Serial.print("scan RS-");
    char b[12];
    snprintf(b, sizeof(b), "%04lu", static_cast<unsigned long>(scan_id_));
    Serial.print(b);
    Serial.print(" started, source=");
    Serial.println(src->name());
  }

  ScanView v;
  v.calibration = calibration;
  v.scan_id = scan_id_;
  v.source_tag = sourceTag();
  v.settling = true;
  ui_.drawScanFrame(v);
  return true;
}

// Stops the acquisition and discards whatever the DSP worker is still doing for this scan.
void App::stopScanCleanly() {
  acq_.requestStop();
  waitAcqIdle();
  if (dsp_.busy()) {                                    // let a running job finish and drop its result
    rs::SegmentResult r;
    rs::PreviewData pv;
    uint32_t us;
    for (int i = 0; i < 2000 && dsp_.busy(); i++) {
      if (dsp_.takeFull(r, us) || dsp_.takePreview(r, pv)) break;
      delay(1);
    }
  }
  analyzing_ = false;
  preview_pending_ = false;
}

void App::abortScan() {
  if (st_ != St::kScanning && st_ != St::kCalibrating) return;
  stopScanCleanly();
  Serial.println("# scan aborted");
  alerts_.play(Beep::kKey);
  enterIdle(t(T_MSG_ABORTED));
}

void App::armNextSegment() {
  seg_no_++;
  analyzing_ = false;
  preview_pending_ = false;
  last_preview_frames_ = 0;
  last_preview_ms_ = millis();
  acq_.requestArm(false);

  ScanView v;                                           // clear the previous segment's preview
  v.scan_id = scan_id_;
  v.segment = seg_no_;
  v.elapsed_s = static_cast<uint32_t>((millis() - t_scan_ms_) / 1000u);
  v.source_tag = sourceTag();
  ui_.drawScanFrame(v);
  ui_ms_ = millis();
}

void App::stepScan(uint32_t now) {
  const AcqStatus st = acq_.status();

  // ---- a front-end that changes its frame rate / bins / range axis under a running scan makes the
  // ---- frequency axis of that scan wrong: no verdict may come out of it (the acquisition thread pins the
  // ---- geometry when the scan starts and reports a change in its status)
  if (st.geometry_changed) {
    Serial.println("# radar front-end changed its settings during the scan - scan aborted");
    stopScanCleanly();
    alerts_.play(Beep::kError);
    showMessage("RADAR CHANGED", "The front-end changed its frame rate or range settings during the scan. No result. Rescan.", col::kDkRed);
    return;
  }

  // ---- DSP results first: a finished preview must be collected before a full job can start
  if (preview_pending_) {
    rs::SegmentResult pr;
    if (dsp_.takePreview(pr, pv_)) {
      preview_pending_ = false;
      if (pv_.have) ui_.drawPreview(pv_, pr);
    }
  }
  if (analyzing_) {
    rs::SegmentResult r;
    uint32_t us = 0;
    if (dsp_.takeFull(r, us)) {
      analyzing_ = false;
      Serial.print("# segment analysed in ");
      Serial.print(static_cast<unsigned long>(us / 1000u));
      Serial.println(" ms");
      onSegment(r);
      return;
    }
    if (static_cast<uint32_t>(now - analysis_t0_) > kAnalysisTimeoutMs) {
      Serial.println("# DSP timeout - scan aborted");
      acq_.requestStop();
      showMessage("DSP TIMEOUT", "The analysis did not finish. Restart the device.", col::kDkRed);
      return;
    }
  } else {
    // ---- a dead radar link ends the segment as an invalid one (never silently waits forever)
    if (st.stalled) {
      Serial.println("# radar link stalled: segment invalid");
      acq_.requestStop();
      rs::SegmentResult inv;
      memset(&inv, 0, sizeof(inv));
      inv.valid = false;
      inv.reason = rs::InvalidReason::kDropout;
      onSegment(inv);
      return;
    }
    if (st.mode == AcqStatus::kFull && !dsp_.busy()) {
      uint8_t ns = 0;
      const rs::Seed* sd = (st_ == St::kScanning) ? session_.seeds(ns) : nullptr;
      if (sd == nullptr) ns = 0;
      if (dsp_.submitFull(params_, sd, ns)) {
        analyzing_ = true;
        analysis_t0_ = now;
      }
    } else if (st.mode == AcqStatus::kRunning && !preview_pending_ && !dsp_.busy() &&
               st.frames >= rs::kPrevFrames && st.frames >= last_preview_frames_ + kPreviewFrames &&
               static_cast<uint32_t>(now - last_preview_ms_) >= kPreviewMinMs) {
      if (dsp_.submitPreview(params_, st.frames)) {
        preview_pending_ = true;
        last_preview_frames_ = st.frames;
        last_preview_ms_ = now;
      }
    }
  }

  // ---- screen refresh
  if (static_cast<uint32_t>(now - ui_ms_) >= kUiRefreshMs) {
    ui_ms_ = now;
    ScanView v;
    v.calibration = (st_ == St::kCalibrating);
    v.scan_id = scan_id_;
    v.segment = seg_no_;
    v.elapsed_s = static_cast<uint32_t>((now - t_scan_ms_) / 1000u);
    if (activeSource()->warp() > 1.0f) v.elapsed_s = static_cast<uint32_t>(v.elapsed_s * activeSource()->warp());
    v.frames = st.frames;
    v.drops = st.dropped_frames;
    v.imu = imuState(st);
    v.settling = (st.mode == AcqStatus::kSettling);
    v.analyzing = analyzing_;
    v.stalled = st.stalled;
    v.source_tag = sourceTag();
    ui_.updateScan(v);
  }
}

void App::onSegment(const rs::SegmentResult& r) {
  if (st_ == St::kCalibrating) {
    finishCalibration(r);
    return;
  }
  const bool more = session_.feed(r);
  if (more) armNextSegment();
  else finishScan();
}

void App::applyVerdictAlerts(rs::Verdict v) {
  alerts_.scanLed(ScanLed::kOff);
  switch (v) {
    case rs::Verdict::kCandidate:
      alerts_.leds(true, false);
      alerts_.play(Beep::kCandidate);
      break;
    case rs::Verdict::kPossible:
      alerts_.leds(false, true);
      alerts_.play(Beep::kPossible);
      break;
    case rs::Verdict::kConfounded:
      alerts_.leds(false, true);
      alerts_.play(Beep::kConfounded);
      break;
    case rs::Verdict::kLowSensitivity:
      alerts_.leds(false, true);
      alerts_.play(Beep::kLowSens);
      break;
    case rs::Verdict::kInvalid:
      alerts_.leds(false, true);
      alerts_.play(Beep::kInvalid);
      break;
    case rs::Verdict::kNoPattern:
    case rs::Verdict::kNone:
      alerts_.leds(false, false);               // never green, never "all clear"
      alerts_.play(Beep::kNegative);
      break;
  }
}

void App::finishScan() {
  acq_.requestStop();
  waitAcqIdle();

  const rs::ScanOutcome& o = session_.outcome();
  last_out_ = o;

  // ---- record: serial + flash (the acquisition is idle now, flash writes are safe)
  ScanRecord rec;
  rec.scan_id = scan_id_;
  rec.uptime_s = millis() / 1000u;
  float warp = activeSource()->warp();
  rec.duration_s = static_cast<float>(millis() - t_scan_ms_) / 1000.0f * warp;
  rec.source = activeSource()->name();
  rec.preset = (settings_.get().source == SourceKind::kSim) ? trialPreset(settings_.get().preset).id : "";
  const AcqStatus as = acq_.status();
  rec.imu = as.imu_host ? ImuSource::kHost : ((imu_.present()) ? ImuSource::kHardware : ImuSource::kSimulated);
  rec.info = info_;
  rec.params = params_;
  rec.out = o;
  rec.n_seg = session_.logCount();
  for (uint8_t i = 0; i < rec.n_seg && i < rs::ScanSession::kMaxLog; i++) rec.seg[i] = session_.log(i);
  rec.baseline_src = baseline_src_;
  LinkStats ls;
  activeSource()->linkStats(ls);
  rec.lost_frames = ls.lost_frames;
  rec.crc_errors = ls.crc_errors;
  rec.timing_flagged = ls.timing_flagged;
  rec.timing_checked = ls.timing_checked;

  const size_t n = buildScanJson(json_, sizeof(json_), rec);
  if (n > 0) {
    Serial.println(json_);
    if (!storage_.append(json_)) Serial.println("# warning: scan record not stored");
  } else {
    json_[0] = '\0';
    Serial.println("# ERR scan record too large for the buffer");
  }
  settings_.saveCounter();

  // ---- operator
  ResultView rv;
  rv.scan_id = scan_id_;
  rv.out = o;
  rv.range_m = info_.range_start_m + static_cast<float>(o.bin) * info_.range_step_m;
  rv.platform_mm = platformMotionMm(o.theta_rms_rad, info_.wavelength_mm);
  rv.evidence = evidenceName(o);
  rv.source_tag = sourceTag();
  ui_.drawResult(rv);
  applyVerdictAlerts(o.verdict);
  st_ = St::kResult;
}

void App::finishCalibration(const rs::SegmentResult& r) {
  acq_.requestStop();
  waitAcqIdle();
  Settings& s = settings_.get();
  CalibView cv;
  cv.source_tag = sourceTag();
  const bool good = r.valid && isfinite(r.floor_db) && r.floor_db > 0.0f;
  cv.ok = good;
  cv.floor_db = r.floor_db;
  cv.why = r.reason;
  cv.lines_seen = r.valid && (r.n_det > 0 || r.n_platform > 0 || r.platform_peak);
  if (good) {
    if (s.source == SourceKind::kSim) { s.bl_sim_ok = true; s.bl_sim_db = r.floor_db; }
    else { s.bl_ser_ok = true; s.bl_ser_db = r.floor_db; }
    settings_.saveBaselines();
    Serial.print("# baseline stored: ");
    Serial.print(static_cast<double>(r.floor_db), 1);
    Serial.println(" dB");
    alerts_.play(Beep::kDone);
  } else {
    Serial.print("# calibration failed: ");
    Serial.println(rs::invalidReasonName(r.reason));
    alerts_.play(Beep::kError);
  }
  alerts_.scanLed(ScanLed::kOff);
  alerts_.leds(false, !good);
  ui_.drawCalibResult(cv);
  st_ = St::kMessage;
}

void App::showMessage(const char* title, const char* text, uint16_t color) {
  ui_.drawMessage(title, text, color, sourceTag());
  alerts_.scanLed(ScanLed::kOff);
  st_ = St::kMessage;
}

// ============================================================================ console host

void App::conStatus(Print& out) {
  const Settings& s = settings_.get();
  static const char* const kStateName[] = {"idle", "scanning", "calibrating", "result", "menu", "selftest", "about", "message", "fatal"};
  const AcqStatus a = acq_.status();
  out.print("# RubbleSense ");
  out.print(RS_FW_VERSION);
  out.print(" state=");
  out.print(kStateName[static_cast<uint8_t>(st_)]);
  out.print(" source=");
  out.print(sourceTag());
  out.print(" preset=");
  out.print(trialPreset(s.preset).id);
  out.print(" imu=");
  out.println(a.imu_host ? "host" : (imu_.present() ? "hw" : (a.imu_sim ? "sim" : "missing")));
  float db = 0.0f;
  bool def = false;
  const bool known = baselineKnown(db, def);
  out.print("# baseline: ");
  if (known) {
    out.print(static_cast<double>(db), 1);
    out.println(def ? " dB (sim default)" : " dB (calibrated)");
  } else {
    out.println("none (sensitivity unknown)");
  }
  out.print("# scans: ");
  out.print(static_cast<unsigned long>(s.scan_counter));
  out.print("  log lines: ");
  out.print(static_cast<unsigned long>(storage_.lines()));
  out.print("  storage: ");
  out.println(storage_.ok() ? "ok" : "off");
  out.print("# heap free: ");
  out.print(static_cast<unsigned long>(ESP.getFreeHeap() / 1024u));
  out.print(" KB  last DSP: ");
  out.print(static_cast<unsigned long>(dsp_.lastComputeUs() / 1000u));
  out.print(" ms  sound=");
  out.print(s.sound ? "on" : "off");
  out.print(" lang=");
  out.println(s.lang == 1 ? "id" : "en");
  LinkStats ls;
  activeSource()->linkStats(ls);
  out.print("# link: frames=");
  out.print(static_cast<unsigned long>(ls.frames_ok));
  out.print(" lost=");
  out.print(static_cast<unsigned long>(ls.lost_frames));
  out.print(" crc=");
  out.print(static_cast<unsigned long>(ls.crc_errors));
  out.print(" skipped=");
  out.print(static_cast<unsigned long>(ls.bytes_skipped));
  out.print(" overflow=");
  out.print(static_cast<unsigned long>(ls.rx_overflow));
  out.print(" timing=");
  out.print(ls.timing_flagged > 0 ? "inconsistent" : (ls.frames_ok > 0 && !ls.timing_checked ? "unchecked" : "ok"));
  out.print(" timing_flagged=");
  out.println(static_cast<unsigned long>(ls.timing_flagged));
}

bool App::conScan(Print& out) {
  (void)out;
  return startScan(false);
}

void App::conAbort(Print& out) {
  (void)out;
  abortScan();
}

bool App::conCalibrate(Print& out) {
  out.println("# calibrating: use a quiet, EMPTY reference spot");
  return startScan(true);
}

void App::conClearBaseline(Print& out) {
  Settings& s = settings_.get();
  if (s.source == SourceKind::kSim) s.bl_sim_ok = false;
  else s.bl_ser_ok = false;
  if (st_ != St::kScanning && st_ != St::kCalibrating) settings_.saveBaselines();
  out.println("# baseline cleared");
  if (st_ == St::kIdle) enterIdle(t(T_MSG_BL_CLEARED));
}

bool App::conSource(Print& out, SourceKind k) {
  if (!selectSource(k)) return false;
  out.print("# source = ");
  out.println(k == SourceKind::kSim ? "SIM" : "SERIAL");
  if (st_ == St::kIdle || st_ == St::kResult) enterIdle(k == SourceKind::kSim ? t(T_MSG_SRC_SIM) : t(T_MSG_SRC_SER));
  return true;
}

bool App::conPreset(Print& out, int idx) {
  if (!selectPreset(static_cast<uint8_t>(idx))) return false;
  out.print("# preset = ");
  out.println(presetName(static_cast<uint8_t>(idx)));
  return true;
}

void App::conLang(Print& out, uint8_t lang) {
  Settings& s = settings_.get();
  s.lang = lang ? 1 : 0;
  txt::setLang(s.lang == 1 ? Lang::ID : Lang::EN);
  if (st_ != St::kScanning && st_ != St::kCalibrating) {
    settings_.saveCore();
    if (st_ == St::kIdle || st_ == St::kResult) enterIdle();
  }
  out.print("# language = ");
  out.println(lang ? "id" : "en");
}

void App::conSound(Print& out, bool on) {
  Settings& s = settings_.get();
  s.sound = on;
  alerts_.setSound(on);
  if (st_ != St::kScanning && st_ != St::kCalibrating) settings_.saveCore();
  out.print("# sound = ");
  out.println(on ? "on" : "off");
}

void App::conLogDump(Print& out) {
  out.println("# --- scan log begin ---");
  storage_.dump(out);
  out.println("# --- scan log end ---");
}

bool App::conLogClear(Print& out) {
  if (st_ == St::kScanning || st_ == St::kCalibrating) return false;
  const bool ok = storage_.clear();
  out.println(ok ? "# log cleared" : "# log not available");
  return ok;
}

void App::conParams(Print& out) { paramList(settings_.get().params, out); }

bool App::conGet(Print& out, const char* name) {
  char v[24];
  if (!paramGet(settings_.get().params, name, v, sizeof(v))) return false;
  out.print("# ");
  out.print(name);
  out.print(" = ");
  out.println(v);
  return true;
}

bool App::conSet(Print& out, const char* name, const char* value) {
  if (st_ == St::kScanning || st_ == St::kCalibrating) {
    out.println("# not while a scan is running");
    return false;
  }
  rs::DspParams cand = settings_.get().params;
  const char* err = nullptr;
  if (!paramSet(cand, name, value, &err)) {
    out.print("# ");
    out.println(err ? err : "bad value");
    return false;
  }
  rs::RadarInfo info;
  float fs = 20.0f;
  if (activeSource()->info(info)) fs = info.frame_rate_hz;
  if (!paramsValid(cand, fs, &err)) {
    out.print("# rejected: ");
    out.println(err ? err : "invalid combination");
    return false;
  }
  settings_.get().params = cand;
  settings_.saveParams();
  char v[24];
  paramGet(cand, name, v, sizeof(v));
  out.print("# ");
  out.print(name);
  out.print(" = ");
  out.println(v);
  return true;
}

void App::conSelfTest(Print& out) {
  SelfTestView v;
  buildSelfTest(v);
  for (uint8_t i = 0; i < v.n; i++) {
    out.print("# selftest ");
    out.print(v.name[i]);
    out.print(": ");
    out.print(v.ok[i] ? "OK " : "FAIL ");
    out.println(v.detail[i]);
  }
}

void App::conFactoryReset(Print& out) {
  if (st_ == St::kScanning || st_ == St::kCalibrating) abortScan();
  settings_.factoryReset();
  storage_.clear();
  txt::setLang(Lang::EN);
  alerts_.setSound(true);
  sim_.setPreset(settings_.get().preset);
  acq_.setSource(activeSource());
  routeInput(SourceKind::kSim);
  configureAcq();
  out.println("# factory reset done");
  enterIdle();
}

void App::conReboot() {
  reboot_req_ = true;
#if !RS_HOST
  delay(100);
  ESP.restart();
#endif
}
