// Firmware host tests, part 1: boot, display texts and layout, buttons and menu, console,
// persistence, scan log. Everything here runs the real firmware sources on the stubs.
#include <string.h>
#include <strings.h>
#include <string>
#include <vector>
#include <Preferences.h>
#include <Wire.h>
#include "fw_rig.h"
#include "fw_tests.h"
#include "rs_text.h"
#include "scan_json.h"

using txt::Lang;

std::string g_screens_dir;

namespace {

void dumpScreen(const char* name) {
  if (g_screens_dir.empty()) return;
  stub::gfx().dump(g_screens_dir + "/" + name + ".txt");
}

// A claim of clearance / absence must never appear without a negation in the same screen text.
bool dangerousClaim(const std::string& text_in, std::string* which = nullptr) {
  const std::string t = lower(text_in);
  static const char* const kClaimEn[] = {"all clear", "all-clear", "area clear", "area is clear", "is clear", "are clear",
                                         "area safe", "is safe", "no survivor", "no victim", "no one", "nobody", "no person",
                                         "no life", "all safe", "nothing there"};
  static const char* const kClaimId[] = {"area aman", "area bersih", "tidak ada korban", "tidak ada orang", "tidak ada yang selamat",
                                         "bebas bahaya", "aman."};
  static const char* const kNegEn[] = {" not ", "n't", "never", "do not"};
  static const char* const kNegId[] = {"bukan", "jangan", "tidak boleh"};
  bool neg = false;
  const std::string padded = " " + t + " ";
  for (size_t i = 0; i < sizeof(kNegEn) / sizeof(kNegEn[0]); i++) neg = neg || has(padded, kNegEn[i]);
  for (size_t i = 0; i < sizeof(kNegId) / sizeof(kNegId[0]); i++) neg = neg || has(padded, kNegId[i]);
  if (neg) return false;
  for (size_t i = 0; i < sizeof(kClaimEn) / sizeof(kClaimEn[0]); i++)
    if (has(t, kClaimEn[i])) { if (which) *which = kClaimEn[i]; return true; }
  for (size_t i = 0; i < sizeof(kClaimId) / sizeof(kClaimId[0]); i++)
    if (has(t, kClaimId[i])) { if (which) *which = kClaimId[i]; return true; }
  return false;
}

// Conversion specifiers of a printf format, e.g. "SCAN #%lu SEG %u %02u:%02u" -> "lu,u,u,u".
std::string specsOf(const char* f) {
  std::string out;
  for (const char* p = f; *p; p++) {
    if (*p != '%') continue;
    p++;
    if (*p == '%') continue;
    while (*p && strchr("+-# 0123456789.", *p)) p++;
    std::string conv;
    while (*p && strchr("lhzjt", *p)) conv += *p++;
    if (*p) conv += *p;
    if (!out.empty()) out += ',';
    out += conv;
    if (!*p) break;
  }
  return out;
}

bool asciiOnly(const char* s) {
  for (; *s; s++)
    if (static_cast<unsigned char>(*s) < 0x20 || static_cast<unsigned char>(*s) > 0x7E) return false;
  return true;
}

rs::ScanOutcome sampleOutcome(rs::Verdict v, bool sens_known, bool vibration, rs::InvalidReason why) {
  rs::ScanOutcome o;
  memset(&o, 0, sizeof(o));
  o.verdict = v;
  o.done = true;
  o.n_segments = 3;
  o.n_valid = (v == rs::Verdict::kInvalid) ? 0 : 3;
  o.n_invalid = (v == rs::Verdict::kInvalid) ? 3 : 0;
  o.last_invalid = why;
  o.bin = 9;
  o.hz = (v == rs::Verdict::kCandidate || v == rs::Verdict::kPossible || v == rs::Verdict::kConfounded) ? 0.28f : 0.0f;
  o.bpm = 16.8f;
  o.snr_db = 21.5f;
  o.margin_db = 12.3f;
  o.agree = 2;
  o.vetted = true;
  o.floor_db = 21.3f;
  o.sens_known = sens_known;
  o.floor_delta_db = sens_known ? (v == rs::Verdict::kLowSensitivity ? 9.6f : 0.2f) : 0.0f;
  o.low_sens = sens_known && v == rs::Verdict::kLowSensitivity;
  o.cpe_applied = true;
  o.vibration = vibration;
  o.vibration_hz = 0.45f;
  o.theta_rms_rad = 0.02f;
  return o;
}

// Layout invariants of what one drawing call put on the screen, measured against the state before it.
struct UiCheck {
  uint32_t trunc0, clip0, over0, part0, tol0, lot0;
  explicit UiCheck(Rig& r)
      : trunc0(r.app->ui().truncations()), clip0(stub::gfx().clippedGlyphs()), over0(stub::gfx().textOverText()),
        part0(stub::gfx().partialOverdraws()), tol0(stub::gfx().textOverLine()), lot0(stub::gfx().lineOverText()) {}
  void expectClean(Rig& r, const char* what) const { gfxClean(r, what, *this); }
  static void gfxClean(Rig& r, const char* what, const UiCheck& c) {
    const uint32_t over = stub::gfx().textOverText() - c.over0;
    const uint32_t part = stub::gfx().partialOverdraws() - c.part0;
    const bool ok_t = r.app->ui().truncations() == c.trunc0;
    const bool ok_c = stub::gfx().clippedGlyphs() == c.clip0;
    const bool ok_o = over == 0;
    const bool ok_p = part == 0;
    const uint32_t tol = stub::gfx().textOverLine() - c.tol0, lot = stub::gfx().lineOverText() - c.lot0;
    const bool ok_l = tol == 0 && lot == 0;                       // text must not touch a box outline or a line
    if (!ok_t || !ok_c || !ok_o || !ok_p || !ok_l)
      std::printf("  layout problem on '%s': truncated=%d clipped=%d text-over-text=%u partial-overdraw=%u text-over-line=%u line-over-text=%u  %s\n", what,
                  static_cast<int>(r.app->ui().truncations() - c.trunc0), static_cast<int>(stub::gfx().clippedGlyphs() - c.clip0),
                  over, part, tol, lot,
                  over ? stub::gfx().lastOverlap().c_str()
                       : (!ok_c ? stub::gfx().lastClip().c_str()
                                : (!ok_l ? stub::gfx().lastLineOverlap().c_str() : (!ok_t ? r.screen().c_str() : ""))));
    CHECK(ok_t);
    CHECK(ok_c);
    CHECK(ok_o);
    CHECK(ok_p);
    CHECK(ok_l);
  }
};

const char* verdictTag(rs::Verdict v) { return rs::verdictName(v); }

}  // namespace

// ============================================================================ boot
void fw_boot() {
  stub::reset();
  Rig r;
  r.boot();
  CHECK(r.began);
  CHECK(r.state() == App::St::kIdle);
  const std::string out = r.takeSerial();
  CHECK(has(out, "SAFETY NOTICE"));
  CHECK(has(out, "never means"));
  CHECK(has(out, "IMU: MPU6050-class sensor at 0x68"));
  CHECK(has(out, "LittleFS mounted"));
  CHECK(stub::gfx().rotation == 1);
  CHECK(stub::gfx().beginCalls == 1);
  CHECK(Wire.began() && Wire.sda() == PIN_I2C_SDA && Wire.scl() == PIN_I2C_SCL);

  r.run_ms(1500);
  const std::string scr = r.screen();
  CHECK(r.shows("negative scan does NOT mean nobody is there"));
  CHECK(r.shows("verify on site"));
  CHECK(r.shows("RUBBLESENSE"));
  CHECK(!dangerousClaim(scr));
  CHECK(r.app->ui().truncations() == 0);
  CHECK(stub::gfx().clippedGlyphs() == 0);
  CHECK(stub::gfx().textOverText() == 0);
  CHECK(stub::gfx().partialOverdraws() == 0);
  dumpScreen("idle_en");

  // the green LED means "candidate pattern" and nothing else
  CHECK(stub::output(PIN_LED_CAND) == 0);
  // the blue LED blinks while idle ("ready")
  bool saw_on = false, saw_off = false;
  for (int i = 0; i < 40; i++) {
    r.run_ms(50);
    (stub::output(PIN_LED_SCAN) ? saw_on : saw_off) = true;
  }
  CHECK(saw_on && saw_off);
  CHECK(stub::buzzerHz() == 0);                       // silent unless something happened
  CHECK(!r.app->rebootRequested());

  // a heap that is "free" must stay above the self-test limit with both big buffers allocated
  CHECK(ESP.getFreeHeap() > 40u * 1024u);
}

// ============================================================================ display texts and layout
void fw_text_tables() {
  // 1. every format string has the same conversion specifiers in both languages, and they are the
  //    ones the UI code passes arguments for
  struct F { txt::Id id; const char* want; };
  const F fmts[] = {
      {txt::T_F_SCAN_HDR, "lu,u,u,u"}, {txt::T_F_CALIB_HDR, "u,u"}, {txt::T_F_FRAMES, "u,u,u"},
      {txt::T_F_EVID_RANGE, "f,f"},    {txt::T_F_EVID_SNR, "f,f,u"}, {txt::T_F_FLOOR_BASE, "f,f,f"},
      {txt::T_F_FLOOR_ONLY, "f"},      {txt::T_F_VIB, "f,f"},       {txt::T_F_BASELINE, "f"},
      {txt::T_F_LOG, "lu"},            {txt::T_F_CALIB_VAL, "f"},
  };
  for (size_t i = 0; i < sizeof(fmts) / sizeof(fmts[0]); i++) {
    txt::setLang(Lang::EN);
    const std::string en = specsOf(txt::t(fmts[i].id));
    txt::setLang(Lang::ID);
    const std::string id = specsOf(txt::t(fmts[i].id));
    if (en != fmts[i].want || id != fmts[i].want)
      std::printf("  format id %d: en='%s' id='%s' want='%s'\n", static_cast<int>(fmts[i].id), en.c_str(), id.c_str(), fmts[i].want);
    CHECK(en == fmts[i].want);
    CHECK(id == fmts[i].want);
  }

  // 2. plain ASCII only (the built-in TFT font has no accents), nothing empty, nothing that claims clearance
  for (int l = 0; l < 2; l++) {
    txt::setLang(l == 0 ? Lang::EN : Lang::ID);
    for (int i = 0; i < static_cast<int>(txt::T_COUNT); i++) {
      const char* s = txt::t(static_cast<txt::Id>(i));
      if (!asciiOnly(s) || s[0] == '\0') std::printf("  text %d (lang %d) is empty or not ASCII\n", i, l);
      CHECK(s != nullptr && s[0] != '\0' && asciiOnly(s));
    }
    for (int v = 0; v <= static_cast<int>(rs::Verdict::kCandidate); v++) {
      const rs::Verdict vv = static_cast<rs::Verdict>(v);
      CHECK(asciiOnly(txt::verdictHeadline(vv)));
      for (int why = 0; why < static_cast<int>(rs::InvalidReason::kCount); why++) {
        const char* a = txt::verdictAction(vv, static_cast<rs::InvalidReason>(why));
        CHECK(asciiOnly(a));
        std::string w;
        if (dangerousClaim(std::string(txt::verdictHeadline(vv)) + " " + a, &w))
          std::printf("  verdict %d reason %d lang %d: claim '%s'\n", v, why, l, w.c_str());
        CHECK(!dangerousClaim(std::string(txt::verdictHeadline(vv)) + " " + a));
      }
    }
    for (uint8_t p = 0; p < kNumPresets; p++) CHECK(asciiOnly(txt::presetName(p)) && txt::presetName(p)[0] != '\0');
  }
  txt::setLang(Lang::EN);

  // 3. the negative verdict: the mandatory English sentence, verbatim
  CHECK(std::string(txt::mandatoryNegativeEn()) == "No reliable life-sign pattern detected in this scan. DO NOT clear area.");
  CHECK(lower(txt::verdictHeadlineEn(rs::Verdict::kNoPattern)) == "no reliable life-sign pattern detected in this scan.");
  CHECK(std::string(txt::negativeBannerLineEn()) == "DO NOT CLEAR AREA.");
  // no verdict headline may read as an all-clear and there is no verdict named like one
  for (int v = 0; v <= static_cast<int>(rs::Verdict::kCandidate); v++) {
    const std::string n = lower(rs::verdictName(static_cast<rs::Verdict>(v)));
    CHECK(!has(n, "clear") && !has(n, "safe") && !has(n, "absent") && !has(n, "empty") && !has(n, "none_found"));
  }
}

void fw_ui_screens() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(200);
  Ui& ui = r.app->ui();
  const rs::Verdict kV[] = {rs::Verdict::kInvalid, rs::Verdict::kConfounded, rs::Verdict::kLowSensitivity,
                            rs::Verdict::kNoPattern, rs::Verdict::kPossible, rs::Verdict::kCandidate};

  for (int lang = 0; lang < 2; lang++) {
    txt::setLang(lang == 0 ? Lang::EN : Lang::ID);
    const char* lt = lang == 0 ? "en" : "id";

    // ---- result screens: every verdict x every invalid reason x sensitivity known/unknown x vibration notice
    for (size_t vi = 0; vi < sizeof(kV) / sizeof(kV[0]); vi++) {
      const int n_why = (kV[vi] == rs::Verdict::kInvalid) ? static_cast<int>(rs::InvalidReason::kCount) : 1;
      for (int why = 0; why < n_why; why++) {
        for (int sens = 0; sens < 2; sens++) {
          for (int vib = 0; vib < 2; vib++) {
            ResultView rv;
            rv.scan_id = 4242;
            rv.out = sampleOutcome(kV[vi], sens != 0, vib != 0, kV[vi] == rs::Verdict::kInvalid ? static_cast<rs::InvalidReason>(why) : rs::InvalidReason::kNone);
            rv.range_m = 2.7f;
            rv.platform_mm = 0.2f;
            rv.evidence = evidenceName(rv.out);
            rv.source_tag = "SIM";
            const UiCheck chk(r);
            ui.drawResult(rv);
            char name[96];
            snprintf(name, sizeof(name), "%s result %s why=%d sens=%d vib=%d", lt, verdictTag(kV[vi]), why, sens, vib);
            UiCheck::gfxClean(r, name, chk);
            const std::string scr = r.screen();
            std::string w;
            if (dangerousClaim(scr, &w)) std::printf("  '%s' makes a claim: %s\n  screen: %s\n", name, w.c_str(), scr.c_str());
            CHECK(!dangerousClaim(scr));
            // the permanent reminder is on every result screen
            CHECK(r.shows(txt::t(txt::T_NEGATIVE_NOTE)));
            if (kV[vi] == rs::Verdict::kNoPattern) {
              CHECK(r.shows(txt::verdictHeadline(rs::Verdict::kNoPattern)));
              CHECK(r.shows(txt::negativeBannerLine()));
              if (lang == 0) CHECK(r.shows("No reliable life-sign pattern detected in this scan. DO NOT CLEAR AREA."));
            }
            if (kV[vi] == rs::Verdict::kCandidate) CHECK(r.shows(txt::verdictHeadline(rs::Verdict::kCandidate)));
            // evidence numbers only where a line exists
            if (kV[vi] == rs::Verdict::kNoPattern || kV[vi] == rs::Verdict::kInvalid || kV[vi] == rs::Verdict::kLowSensitivity)
              CHECK(!r.shows("bpm") && !r.shows("/min"));
            if (sens == 0 && kV[vi] != rs::Verdict::kInvalid) CHECK(r.shows(txt::t(txt::T_SENS_UNKNOWN)));
            if (vib == 1) CHECK(r.shows("0.45 Hz"));
            if (lang == 0 && why == 0 && sens == 1 && vib == 1) {
              char n[64];
              snprintf(n, sizeof(n), "result_%s_en", lower(verdictTag(kV[vi])).c_str());
              dumpScreen(n);
            }
            if (lang == 1 && why == 0 && sens == 1 && vib == 0 && kV[vi] == rs::Verdict::kNoPattern) dumpScreen("result_no_pattern_id");
          }
        }
      }
    }

    // ---- idle: source x IMU state x baseline x log x sound x message
    const ImuState imus[] = {ImuState::kOk, ImuState::kMoving, ImuState::kMissing, ImuState::kSimulated, ImuState::kHost};
    for (int serial = 0; serial < 2; serial++)
      for (size_t k = 0; k < 5; k++)
        for (int bl = 0; bl < 3; bl++)
          for (int logok = 0; logok < 2; logok++)
            for (int sound = 0; sound < 2; sound++) {
              IdleView v;
              v.sim = !serial;
              v.source_tag = serial ? "SERIAL" : "SIM";
              v.trial_name = txt::presetName(static_cast<uint8_t>(1 + (k % 6)));
              v.manual = (k == 0);
              v.serial_ready = (bl != 0);
              v.imu = imus[k];
              v.baseline_known = (bl != 0);
              v.baseline_simdef = (bl == 1);
              v.baseline_db = 21.1f;
              v.log_ok = logok != 0;
              v.log_lines = 123456;
              v.sound = sound != 0;
              v.message = (k == 2) ? txt::t(txt::T_MSG_NOT_READY) : "";
              const UiCheck chk(r);
              ui.drawIdle(v);
              char name[96];
              snprintf(name, sizeof(name), "%s idle serial=%d imu=%d bl=%d log=%d snd=%d", lt, serial, static_cast<int>(k), bl, logok, sound);
              UiCheck::gfxClean(r, name, chk);
              CHECK(!dangerousClaim(r.screen()));
              CHECK(r.shows(txt::t(txt::T_SAFETY_1)));
              if (bl == 0) CHECK(r.shows(txt::t(txt::T_BASELINE_NONE)));
              // live rows update in place without piling text on text
              v.imu = ImuState::kMoving;
              v.message = txt::t(txt::T_MSG_ABORTED);
              ui.updateIdleStatus(v);
              UiCheck::gfxClean(r, name, chk);
            }
    {
      IdleView v;
      v.manual = true;
      v.trial_name = txt::presetName(0);
      ui.drawIdle(v);
      dumpScreen(lang == 0 ? "idle_manual_en" : "idle_manual_id");
      v.sim = false;
      v.source_tag = "SERIAL";
      v.serial_ready = false;
      v.imu = ImuState::kMissing;
      ui.drawIdle(v);
      if (lang == 0) dumpScreen("idle_serial_waiting_en");
    }

    // ---- scanning: settling / collecting / analysing / stalled, with drops, long scan ids
    for (int mode = 0; mode < 5; mode++) {
      ScanView v;
      v.calibration = (mode == 4);
      v.scan_id = 99999;
      v.segment = 3;
      v.elapsed_s = 3599;
      v.frames = static_cast<uint16_t>(rs::kSegFrames);
      v.drops = 65535;
      v.imu = ImuState::kMoving;
      v.settling = (mode == 0);
      v.analyzing = (mode == 1);
      v.stalled = (mode == 2);
      v.source_tag = "SERIAL";
      const UiCheck chk(r);
      ui.drawScanFrame(v);
      v.frames = 200;
      v.elapsed_s = 61;
      ui.updateScan(v);
      UiCheck::gfxClean(r, "scan frame", chk);
      CHECK(!dangerousClaim(r.screen()));
      if (mode == 2) CHECK(r.shows(txt::t(txt::T_STALLED)));
      if (mode == 0 && lang == 0) dumpScreen("scan_settling_en");
    }

    // ---- calibration results, menu, self-test, about, messages
    for (int ok = 0; ok < 2; ok++)
      for (int seen = 0; seen < 2; seen++) {
        CalibView cv;
        cv.ok = ok != 0;
        cv.floor_db = 21.09f;
        cv.lines_seen = seen != 0;
        cv.why = rs::InvalidReason::kNoSignal;
        const UiCheck chk(r);
        ui.drawCalibResult(cv);
        UiCheck::gfxClean(r, "calib result", chk);
        CHECK(!dangerousClaim(r.screen()));
        if (lang == 0 && ok && seen) dumpScreen("calib_ok_lines_en");
        if (lang == 0 && !ok) dumpScreen("calib_failed_en");
      }
    // ---- the real menu in this language, built by the application itself: every selection, both sources,
    // ---- sound on / off, then the self-test and about screens it opens
    CHECK(replyOk(r.say(lang == 0 ? "lang en" : "lang id")));
    for (int variant = 0; variant < 4; variant++) {
      CHECK(replyOk(r.say((variant & 1) ? "src serial" : "src sim")));
      CHECK(replyOk(r.say((variant & 2) ? "sound off" : "sound on")));
      CHECK(r.state() == App::St::kIdle);
      char name[64];
      {
        const UiCheck chk(r);
        r.press(PIN_BTN_MENU);
        CHECK(r.state() == App::St::kMenu);
        snprintf(name, sizeof(name), "%s menu open (variant %d)", lt, variant);
        UiCheck::gfxClean(r, name, chk);
      }
      for (int sel = 1; sel < 7; sel++) {
        const UiCheck chk(r);
        r.press(PIN_BTN_MODE);
        snprintf(name, sizeof(name), "%s menu item %d (variant %d)", lt, sel, variant);
        UiCheck::gfxClean(r, name, chk);
        CHECK(!dangerousClaim(r.screen()));
        if (sel == 3 && variant == 0 && lang == 0) dumpScreen("menu_en");
        if (sel == 3 && variant == 1 && lang == 1) dumpScreen("menu_id_serial");
      }
      // selection is now on the last item (about); walk to the self-test and open both
      r.press(PIN_BTN_MODE);                                   // wraps to item 0
      for (int sel = 1; sel <= 5; sel++) r.press(PIN_BTN_MODE);
      {
        const UiCheck chk(r);
        r.press(PIN_BTN_SCAN);                                 // item 5: self-test
        CHECK(r.state() == App::St::kSelfTest);
        snprintf(name, sizeof(name), "%s selftest (variant %d)", lt, variant);
        UiCheck::gfxClean(r, name, chk);
        if (lang == 0 && variant == 0) dumpScreen("selftest_en");
      }
      r.press(PIN_BTN_MODE);                                   // any key: back to the menu
      CHECK(r.state() == App::St::kMenu);
      r.press(PIN_BTN_MODE);                                   // item 6: about
      {
        const UiCheck chk(r);
        r.press(PIN_BTN_SCAN);
        CHECK(r.state() == App::St::kAbout);
        snprintf(name, sizeof(name), "%s about (variant %d)", lt, variant);
        UiCheck::gfxClean(r, name, chk);
        std::string w;
        CHECK(!dangerousClaim(r.screen(), &w));
        if (lang == 0 && variant == 0) dumpScreen("about_en");
      }
      r.press(PIN_BTN_MODE);
      r.press(PIN_BTN_MENU);                                   // close the menu
      CHECK(r.state() == App::St::kIdle);
    }
    CHECK(replyOk(r.say("src sim")));
    CHECK(replyOk(r.say("sound on")));

    // ---- worst cases of the self-test detail column (the real format is "%s %.0fHz %ubins") and of messages
    {
      SelfTestView sv;
      sv.n = 5;
      for (int i = 0; i < 5; i++) {
        snprintf(sv.name[i], sizeof(sv.name[i]), "%s", i == 0 ? txt::t(txt::T_ST_IMU) : (i == 1 ? txt::t(txt::T_ST_STORAGE) : (i == 3 ? txt::t(txt::T_ST_RADAR) : txt::t(txt::T_ST_MEM))));
        snprintf(sv.detail[i], sizeof(sv.detail[i]), "SERIAL 10000Hz 24bins");
        sv.ok[i] = (i % 2) == 0;
      }
      const UiCheck chk(r);
      ui.drawSelfTest(sv);
      UiCheck::gfxClean(r, "selftest worst case", chk);
    }
    {
      const UiCheck chk(r);
      ui.drawMessage("DSP TIMEOUT", "The analysis did not finish. Restart the device.", col::kDkRed, "SIM");
      UiCheck::gfxClean(r, "message", chk);
      if (lang == 0) dumpScreen("message_en");
    }
    {
      const UiCheck chk(r);
      ui.drawMessage("RADAR CHANGED", "The front-end changed its frame rate or range settings during the scan. No result. Rescan.", col::kDkRed, "SERIAL");
      UiCheck::gfxClean(r, "message radar changed", chk);
    }
    {
      const UiCheck chk(r);
      ui.drawMessage("A VERY LONG TITLE THAT NEEDS TWO LINES", "x", col::kDkRed, "SIM");
      UiCheck::gfxClean(r, "message long title", chk);
    }
  }
  txt::setLang(Lang::EN);
  CHECK(ui.truncations() == 0);
}

// ============================================================================ buttons and menu
void fw_buttons_menu() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);
  txt::setLang(Lang::EN);

  // preset button: cycles through the trial presets and wraps
  const uint8_t p0 = r.app->settings().get().preset;
  CHECK(p0 == 1);
  r.press(PIN_BTN_MODE);
  CHECK(r.app->settings().get().preset == 2);
  CHECK(r.shows(txt::presetName(2)));
  for (int i = 0; i < kNumPresets - 1; i++) r.press(PIN_BTN_MODE);
  CHECK(r.app->settings().get().preset == 1);          // full circle
  CHECK(r.state() == App::St::kIdle);

  // a long press in idle does nothing
  r.longPress(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kIdle);
  CHECK(r.app->scanCount() == 0);

  // menu
  r.press(PIN_BTN_MENU);
  CHECK(r.state() == App::St::kMenu);
  CHECK(r.shows(txt::t(txt::T_MENU_TITLE)));
  CHECK(r.shows(txt::t(txt::T_MENU_LANG)));
  // item 3: language
  for (int i = 0; i < 3; i++) r.press(PIN_BTN_MODE);
  r.press(PIN_BTN_SCAN);
  CHECK(r.app->settings().get().lang == 1);
  CHECK(txt::lang() == Lang::ID);
  CHECK(r.shows("Bahasa"));
  // item 4: sound
  r.press(PIN_BTN_MODE);
  CHECK(r.app->alerts().sound());
  r.press(PIN_BTN_SCAN);
  CHECK(!r.app->alerts().sound());
  CHECK(!r.app->settings().get().sound);
  r.press(PIN_BTN_SCAN);
  CHECK(r.app->alerts().sound());
  // item 5: self-test, everything healthy on the healthy stub
  r.press(PIN_BTN_MODE);
  r.press(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kSelfTest);
  CHECK(countOf(r.screen(), txt::t(txt::T_ST_FAIL)) == 0);
  CHECK(r.shows("|a|=9.8"));                          // the IMU model delivers 1 g
  r.press(PIN_BTN_MODE);                                // any button leaves
  CHECK(r.state() == App::St::kMenu);
  // item 6: about / safety text
  r.press(PIN_BTN_MODE);
  r.press(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kAbout);
  CHECK(r.shows(txt::t(txt::T_ABOUT_2)));
  r.press(PIN_BTN_SCAN);
  CHECK(r.state() == App::St::kMenu);
  // back to Indonesian -> English through the menu entry, then leave with the menu button
  r.press(PIN_BTN_MODE);                                // wraps to item 0
  for (int i = 0; i < 3; i++) r.press(PIN_BTN_MODE);
  r.press(PIN_BTN_SCAN);
  CHECK(txt::lang() == Lang::EN);
  r.press(PIN_BTN_MENU);
  CHECK(r.state() == App::St::kIdle);
  CHECK(r.app->ui().truncations() == 0);
  CHECK(stub::gfx().clippedGlyphs() == 0);
  CHECK(stub::gfx().textOverText() == 0);

  // button mashing must not wedge anything (queue of 8, debounce 30 ms)
  for (int i = 0; i < 200; i++) {
    stub::setInput(PIN_BTN_MODE, (i & 1) ? 1 : 0);
    stub::setInput(PIN_BTN_MENU, (i % 3) ? 1 : 0);
    r.run_ms(7);
  }
  stub::setInput(PIN_BTN_MODE, 1);
  stub::setInput(PIN_BTN_MENU, 1);
  r.run_ms(300);
  CHECK(r.state() == App::St::kIdle || r.state() == App::St::kMenu);
  CHECK(r.app->ui().truncations() == 0);
}

// ============================================================================ console
void fw_console() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);

  std::string o = r.say("help");
  CHECK(replyOk(o));
  CHECK(has(o, "commands"));
  CHECK(has(o, "negative scan never means"));
  o = r.say("?");
  CHECK(replyOk(o));

  o = r.say("status");
  CHECK(replyOk(o));
  CHECK(has(o, "state=idle"));
  CHECK(has(o, "source=SIM"));
  CHECK(has(o, "imu=hw"));
  CHECK(has(o, "sim default"));                        // no calibrated baseline yet
  CHECK(has(o, "heap free"));

  // parameters
  o = r.say("get");
  CHECK(replyOk(o));
  CHECK(countOf(o, "\n# ") + (o[0] == '#' ? 1 : 0) >= 8);
  o = r.say("get pfa");
  CHECK(replyOk(o) && has(o, "pfa = 0.01"));
  o = r.say("set pfa 0.02");
  CHECK(replyOk(o) && has(o, "pfa = 0.02"));
  o = r.say("set pfa 5");                              // out of range: rejected, value kept
  CHECK(replyErr(o) && has(o, "rejected"));
  o = r.say("get pfa");
  CHECK(has(o, "pfa = 0.02"));
  o = r.say("set pfa abc");
  CHECK(replyErr(o) && has(o, "not a number"));
  o = r.say("set lo 0.9");                             // lo > hi
  CHECK(replyErr(o));
  o = r.say("set hi 0.7");                             // narrower band is fine
  CHECK(replyOk(o));
  o = r.say("set hi 0.9");                             // > 0.45 fs? 0.45*20 = 9, fine; <= 1.5 -> accepted? band must fit CFAR window
  const bool hi_ok = replyOk(o);
  (void)hi_ok;
  o = r.say("set bogus 1");
  CHECK(replyErr(o) && has(o, "unknown parameter"));
  o = r.say("set pfa");
  CHECK(replyErr(o));
  o = r.say("set minbin 99");                          // "minbin > 8"
  CHECK(replyErr(o));
  // the safety gates are NOT reachable
  o = r.say("set require_imu 0");
  CHECK(replyErr(o));
  o = r.say("get require_imu");
  CHECK(replyErr(o));
  o = r.say("set cpe off");
  CHECK(replyOk(o));
  o = r.say("set cpe 1");
  CHECK(replyOk(o));

  // selection
  o = r.say("preset c");
  CHECK(replyOk(o) && r.app->settings().get().preset == 3);
  o = r.say("preset 6");
  CHECK(replyOk(o) && r.app->settings().get().preset == 6);
  o = r.say("preset manual");
  CHECK(replyOk(o) && r.app->settings().get().preset == 0);
  o = r.say("preset 9");
  CHECK(replyErr(o));
  o = r.say("preset zz");
  CHECK(replyErr(o));
  o = r.say("preset a");
  CHECK(replyOk(o) && r.app->settings().get().preset == 1);
  o = r.say("lang id");
  CHECK(replyOk(o) && txt::lang() == Lang::ID && r.app->settings().get().lang == 1);
  o = r.say("lang en");
  CHECK(replyOk(o) && txt::lang() == Lang::EN);
  o = r.say("lang fr");
  CHECK(replyErr(o));
  o = r.say("sound off");
  CHECK(replyOk(o) && !r.app->alerts().sound());
  o = r.say("sound on");
  CHECK(replyOk(o) && r.app->alerts().sound());
  o = r.say("sound maybe");
  CHECK(replyErr(o));

  // selftest
  o = r.say("selftest");
  CHECK(replyOk(o) && countOf(o, "OK ") >= 4 && !has(o, "FAIL"));

  // unknown / malformed input
  o = r.say("frobnicate");
  CHECK(replyErr(o) && has(o, "unknown command"));
  o = r.say("   ");
  CHECK(o.empty());
  o = r.say("STATUS");                                 // case-insensitive
  CHECK(replyOk(o));
  o = r.say("factory");
  CHECK(replyErr(o));                                  // needs "confirm"
  o = r.say("src");
  CHECK(replyErr(o));
  o = r.say("calib bogus");
  CHECK(replyErr(o));
  o = r.say("log bogus");
  CHECK(replyErr(o));

  // lines the console must NOT execute: binary content, or longer than the line buffer
  const long scans0 = static_cast<long>(r.app->scanCount());
  {
    const uint8_t bin[] = {0x01, 's', 'c', 'a', 'n', '\n'};
    Serial.clearTx();
    Serial.injectRx(bin, sizeof(bin));
    r.run_ms(80);
    CHECK(!r.scanning());
    CHECK(r.takeSerial().empty());                      // not even an error line
    std::string longline = "scan";
    longline.append(120, ' ');
    longline += "x\n";
    Serial.injectRx(reinterpret_cast<const uint8_t*>(longline.data()), longline.size());
    r.run_ms(80);
    CHECK(!r.scanning());
    const uint8_t hi[] = {'s', 'c', 'a', 'n', 0xC3, 0xA9, '\n'};
    Serial.injectRx(hi, sizeof(hi));
    r.run_ms(80);
    CHECK(!r.scanning());
    CHECK(static_cast<long>(r.app->scanCount()) == scans0);
  }

  // log commands on an empty log
  o = r.say("log");
  CHECK(replyOk(o) && has(o, "scan log begin") && has(o, "scan log end"));
  o = r.say("log clear");
  CHECK(replyOk(o));

  // reboot request
  o = r.say("reboot");
  CHECK(has(o, "OK"));
  CHECK(r.app->rebootRequested());
  CHECK(r.app->ui().truncations() == 0);
}

// ============================================================================ persistence across a power cycle
void fw_persistence() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(300);
  r.say("preset d");
  r.say("lang id");
  r.say("sound off");
  r.say("set pfa 0.005");
  r.say("set lo 0.15");
  r.say("set cpe off");
  r.say("set guard 2");
  r.say("set lowsens 4");
  r.run_ms(100);

  r.powerCycle();
  r.run_ms(300);
  const Settings& s = r.app->settings().get();
  CHECK(s.preset == 4);
  CHECK(s.lang == 1 && txt::lang() == Lang::ID);
  CHECK(!s.sound && !r.app->alerts().sound());
  CHECK_NEAR(s.params.pfa_segment, 0.005, 1e-7);
  CHECK_NEAR(s.params.resp_lo_hz, 0.15, 1e-6);
  CHECK(!s.params.cpe_enable);
  CHECK(s.params.cpe_guard_bins == 2);
  CHECK_NEAR(s.params.low_sens_db, 4.0, 1e-6);
  std::string o = r.say("status");
  CHECK(has(o, "preset=D") && has(o, "lang=id") && has(o, "sound=off"));

  // a corrupt store never reaches the DSP: NVS values are overwritten with garbage
  {
    Preferences p;
    p.begin("rubblesense", false);
    p.putFloat("p_pfa", 7.0f);                         // > 0.2: invalid
    p.putFloat("p_lo", 0.9f);
    p.end();
  }
  r.powerCycle();
  r.run_ms(200);
  CHECK(r.app->settings().get().params.pfa_segment == rs::DspParams().pfa_segment);   // back to defaults as a set
  CHECK(r.app->settings().get().params.resp_lo_hz == rs::DspParams().resp_lo_hz);

  // factory reset wipes the settings and the log
  r.say("lang en");
  r.say("factory confirm");
  r.run_ms(100);
  CHECK(r.app->settings().get().preset == 1);
  CHECK(r.app->settings().get().lang == 0 && txt::lang() == Lang::EN);
  CHECK(r.app->alerts().sound());
  r.powerCycle();
  CHECK(r.app->settings().get().preset == 1);
  CHECK(r.app->settings().get().sound);
}

// ============================================================================ scan log (LittleFS)
namespace {
std::string readLog() {
  std::string all;
  for (const char* n : {"/scans.old.jsonl", "/scans.jsonl"}) {
    std::map<std::string, std::string>::const_iterator it = stub::fs().files.find(n);
    if (it != stub::fs().files.end()) all += it->second;
  }
  return all;
}
}  // namespace

void fw_storage() {
  stub::reset();
  Rig r;
  r.boot();
  r.run_ms(200);
  Storage& st = r.app->storage();
  CHECK(st.ok());
  CHECK(st.lines() == 0);

  // rotation keeps the newest data, never lets the file system fill up, and the line count stays truthful
  std::string line = "{\"v\":1,\"pad\":\"";
  line.append(1500, 'x');
  line += "\"}";
  const size_t max = RS_LOG_MAX_BYTES;
  const int total = static_cast<int>(max / (line.size() + 1)) * 3 + 17;
  for (int i = 0; i < total; i++) CHECK(st.append(line.c_str()));
  CHECK(stub::fs().files.count("/scans.old.jsonl") == 1);
  CHECK(stub::fs().files["/scans.jsonl"].size() <= max);
  CHECK(stub::fs().files["/scans.old.jsonl"].size() <= max);
  const std::string all = readLog();
  const uint32_t real_lines = static_cast<uint32_t>(countOf(all, "\n"));
  if (st.lines() != real_lines) std::printf("  Storage::lines()=%u but the files hold %u lines\n", st.lines(), real_lines);
  CHECK(st.lines() == real_lines);
  CHECK(st.size() == all.size());
  CHECK(st.size() <= 2 * max);
  // the newest line is the last one
  CHECK(!all.empty() && all.back() == '\n');

  // a restart recounts what is on flash
  r.powerCycle();
  CHECK(r.app->storage().lines() == real_lines);

  // dump through the console: old file first, then the current one
  std::string d = r.say("log", 400);
  CHECK(has(d, "scan log begin") && has(d, "scan log end"));
  CHECK(countOf(d, "{\"v\":1") == static_cast<int>(real_lines));

  CHECK(replyOk(r.say("log clear")));
  CHECK(r.app->storage().lines() == 0 && stub::fs().files.empty());

  // a file system that cannot be mounted: the device still scans and still prints its results
  stub::fs().mount_ok = false;
  r.powerCycle();
  r.run_ms(200);
  CHECK(!r.app->storage().ok());
  CHECK(has(r.takeSerial(), "LittleFS unavailable"));
  CHECK(r.shows(txt::t(txt::T_LOG_OFF)));
  CHECK(replyErr(r.say("log clear")));
}

// ---------------------------------------------------------------------------- JSON record
void fw_json() {
  // worst-case record: every segment slot filled, long strings, large numbers, all flags set
  ScanRecord rec;
  rec.scan_id = 4294967295u;
  rec.uptime_s = 4294967295u;
  rec.duration_s = 99999.9f;
  rec.source = "SERIAL";
  rec.preset = "MANUAL";
  rec.imu = ImuSource::kHost;
  rec.info.name = "SERIAL";
  rec.info.frame_rate_hz = 200.0f;
  rec.info.range_start_m = -123.456f;
  rec.info.range_step_m = 12.345f;
  rec.info.wavelength_mm = 125.0f;
  rec.info.n_bins = 24;
  rec.out = sampleOutcome(rs::Verdict::kInvalid, true, true, rs::InvalidReason::kNoSignal);
  rec.out.floor_db = 123456.7f;
  rec.out.theta_rms_rad = 1234.5f;
  rec.n_seg = rs::ScanSession::kMaxLog;
  for (int i = 0; i < rs::ScanSession::kMaxLog; i++) {
    rs::SegmentSummary& s = rec.seg[i];
    memset(&s, 0, sizeof(s));
    s.valid = false;
    s.reason = rs::InvalidReason::kNoSignal;
    s.n_det = 4; s.n_platform = 4; s.n_untested = 4;
    s.best_hz = 0.123456f; s.best_margin_db = -123.4f; s.floor_db = 999999.0f; s.bad_frac = 1.0f; s.theta_rms_rad = 99.0f;
    s.best_bin = 23;
  }
  rec.baseline_src = "sim_default";
  rec.lost_frames = 4294967295u;
  rec.crc_errors = 4294967295u;
  char buf[kScanJsonMax];
  const size_t n = buildScanJson(buf, sizeof(buf), rec);
  CHECK(n > 0 && n < kScanJsonMax);
  std::printf("  worst-case scan record: %u of %u bytes\n", static_cast<unsigned>(n), static_cast<unsigned>(kScanJsonMax));
  CHECK(n > 0 && buf[0] == '{' && buf[n - 1] == '}');
  CHECK(strlen(buf) == n);

  // too small a buffer: refuse (return 0), never overflow
  char small[200];
  memset(small, 0x55, sizeof(small));
  CHECK(buildScanJson(small, 100, rec) == 0);
  CHECK(small[150] == 0x55);

  // NaN / inf are written as 0, never as invalid JSON
  rec.out = sampleOutcome(rs::Verdict::kCandidate, true, false, rs::InvalidReason::kNone);
  rec.out.snr_db = NAN;
  rec.out.margin_db = INFINITY;
  rec.duration_s = -INFINITY;
  const size_t n2 = buildScanJson(buf, sizeof(buf), rec);
  CHECK(n2 > 0);
  // (the word "infer" is in the safety notice: look for a number position, not for a substring)
  bool bad_number = false;
  for (const char* p = buf; *p; p++) {
    if (*p == ':' || *p == ',' || *p == '[') {
      const char* q = p + 1;
      if (*q == '-') q++;
      if (strncasecmp(q, "nan", 3) == 0 || strncasecmp(q, "inf", 3) == 0) bad_number = true;
    }
  }
  if (bad_number) std::printf("  record with NaN/inf inputs: %s\n", buf);
  CHECK(!bad_number);
  CHECK(!has(buf, "\"snr_db\":nan") && has(buf, "\"snr_db\":0.0"));

  // negative verdict: the record carries the mandatory sentence and the safety notice, and nothing like "clear"
  rec.out = sampleOutcome(rs::Verdict::kNoPattern, true, false, rs::InvalidReason::kNone);
  rec.n_seg = 2;
  const size_t n3 = buildScanJson(buf, sizeof(buf), rec);
  CHECK(n3 > 0);
  CHECK(has(buf, "No reliable life-sign pattern detected in this scan. DO NOT clear area."));
  CHECK(has(buf, "Never infer absence of survivor"));
  CHECK(has(buf, "\"verdict\":\"NO_RELIABLE_PATTERN\""));
  CHECK(!has(lower(buf), "all clear") && !has(lower(buf), "area is clear"));
}
