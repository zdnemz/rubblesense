#include "ui.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rs_build.h"
#include "rs_text.h"

using namespace txt;

namespace {
constexpr int kW = 320;
constexpr int kH = 240;
constexpr int kMargin = 10;
constexpr int kCols1 = 48;            // text size 1 (6 px per glyph) inside the margins: 300 px / 6 = 50
constexpr int kCols2 = 25;            // text size 2 (12 px per glyph)
// The verdict banner is a little wider than the body text so that a full line of 25 size-2 characters
// ("JANGAN NYATAKAN AREA AMAN") fits between its outline and the screen edge.
constexpr int kBannerX = 4;
constexpr int kBannerW = kW - 2 * kBannerX;
constexpr int kBannerTextX = kBannerX + 6;

const char* imuText(ImuState s) {
  switch (s) {
    case ImuState::kOk:        return t(T_IMU_OK);
    case ImuState::kMoving:    return t(T_IMU_MOVING);
    case ImuState::kMissing:   return t(T_IMU_MISSING);
    case ImuState::kSimulated: return t(T_IMU_SIM);
    case ImuState::kHost:      return t(T_IMU_HOST);
  }
  return "";
}

uint16_t imuColor(ImuState s) {
  switch (s) {
    case ImuState::kOk:        return col::kGreen;
    case ImuState::kMoving:    return col::kRed;
    case ImuState::kMissing:   return col::kRed;
    case ImuState::kSimulated: return col::kYellow;
    case ImuState::kHost:      return col::kCyan;
  }
  return col::kWhite;
}

// A difference that would print as "-0.0" at one decimal is shown as +0.0.
double diff1(float x) {
  const double v = static_cast<double>(x);
  return (v > -0.05 && v < 0.05) ? 0.0 : v;
}
}  // namespace

// ------------------------------------------------------------------------ helpers

void Ui::text(int x, int y, uint8_t size, uint16_t color, const char* s) {
  tft_.setTextSize(size);
  tft_.setTextColor(color);
  tft_.setCursor(x, y);
  tft_.print(s);
}

// Word-wrap `s` into at most `max_lines` lines of `cols` characters. Returns the lines used.
int Ui::wrap(int x, int y, int cols, uint8_t size, uint16_t color, const char* s, int max_lines) {
  const int step = 8 * size + (size == 1 ? 4 : 0);        // 12 px at size 1, 16 px at size 2
  const int n = static_cast<int>(strlen(s));
  int pos = 0, line = 0;
  char buf[64];
  while (pos < n && line < max_lines) {
    while (pos < n && s[pos] == ' ') pos++;
    if (pos >= n) break;
    int len = n - pos;
    if (len > cols) {
      len = cols;
      int cut = len;
      while (cut > 0 && s[pos + cut] != ' ') cut--;       // last space at or before the limit
      if (cut > 0) len = cut;                            // else: one long word, hard break
    }
    if (len > static_cast<int>(sizeof(buf)) - 1) len = static_cast<int>(sizeof(buf)) - 1;
    memcpy(buf, s + pos, static_cast<size_t>(len));
    buf[len] = '\0';
    text(x, y + line * step, size, color, buf);
    pos += len;
    line++;
  }
  while (pos < n && s[pos] == ' ') pos++;
  if (pos < n) truncated_++;                              // text did not fit: the harness fails on this
  return line;
}

void Ui::header(const char* source_tag) {
  tft_.fillRect(0, 0, kW, 26, col::kNavy);
  char buf[48];
  snprintf(buf, sizeof(buf), "RUBBLESENSE SAR AID  v%s", RS_FW_VERSION);
  text(8, 5, 1, col::kWhite, buf);
  text(8, 15, 1, col::kCyan, t(T_SUBTITLE));
  if (source_tag && source_tag[0]) {
    const int w = static_cast<int>(strlen(source_tag)) * 6;
    text(kW - 8 - w, 5, 1, col::kGreen, source_tag);
  }
}

void Ui::clearBody() { tft_.fillRect(0, 26, kW, kH - 26, col::kBlack); }

void Ui::hint(const char* s) {
  tft_.fillRect(0, 222, kW, 18, col::kBlack);
  text(kMargin, 226, 1, col::kCyan, s);
}

void Ui::imuLine(int y, ImuState s) {
  tft_.fillRect(kMargin, y, kW - 2 * kMargin, 10, col::kBlack);
  text(kMargin, y, 1, col::kLtGrey, t(T_IMU));
  text(kMargin + 30, y, 1, imuColor(s), imuText(s));
}

uint16_t Ui::verdictColor(rs::Verdict v) {
  switch (v) {
    case rs::Verdict::kCandidate:      return col::kDkGreen;
    case rs::Verdict::kPossible:       return col::kAmber;
    case rs::Verdict::kConfounded:     return col::kPurple;
    case rs::Verdict::kLowSensitivity: return col::kBrown;
    case rs::Verdict::kInvalid:        return col::kDkRed;
    case rs::Verdict::kNoPattern:      return col::kSlate;
    case rs::Verdict::kNone:           return col::kSlate;
  }
  return col::kSlate;
}

void Ui::begin() {
  tft_.begin();
  tft_.setRotation(1);
  tft_.setTextWrap(false);
  tft_.fillScreen(col::kBlack);
}

// ------------------------------------------------------------------------ idle

void Ui::drawIdle(const IdleView& v) {
  header(v.source_tag);
  clearBody();

  src_ready_shown_ = -1;
  if (v.sim) {
    text(kMargin, 32, 1, col::kYellow, t(T_LBL_TRIAL));
    text(kMargin + 42, 32, 1, col::kWhite, v.trial_name);
  } else {
    text(kMargin, 32, 1, col::kYellow, t(T_LBL_SOURCE));
    sourceRow(v);
  }

  tft_.drawRect(kMargin, 46, kW - 2 * kMargin, 62, col::kSlate);
  text(18, 52, 1, col::kLtGrey, t(T_IDLE_1));
  text(18, 66, 1, col::kLtGrey, t(T_IDLE_2));
  text(18, 80, 1, col::kLtGrey, t(T_IDLE_3));
  if (v.sim && v.manual) text(18, 94, 1, col::kCyan, t(T_IDLE_POTS));

  tft_.fillRect(kMargin, 158, kW - 2 * kMargin, 34, col::kPanel);
  text(16, 163, 1, col::kOrange, t(T_SAFETY_1));
  text(16, 176, 1, col::kWhite, t(T_SAFETY_2));

  hint(t(T_HINT_IDLE));
  updateIdleStatus(v);
}

// The front-end can connect or die while the idle screen is up: the row must follow.
void Ui::sourceRow(const IdleView& v) {
  tft_.fillRect(kMargin + 48, 30, kW - kMargin - (kMargin + 48), 12, col::kBlack);
  text(kMargin + 48, 32, 1, v.serial_ready ? col::kGreen : col::kOrange,
       v.serial_ready ? t(T_SRC_CONNECTED) : t(T_SRC_WAITING));
  src_ready_shown_ = v.serial_ready ? 1 : 0;
}

void Ui::updateIdleStatus(const IdleView& v) {
  if (!v.sim && src_ready_shown_ != (v.serial_ready ? 1 : 0)) sourceRow(v);
  imuLine(116, v.imu);

  // baseline
  tft_.fillRect(kMargin, 128, kW - 2 * kMargin, 10, col::kBlack);
  text(kMargin, 128, 1, col::kLtGrey, t(T_BASELINE));
  if (v.baseline_known) {
    char b[32];
    snprintf(b, sizeof(b), t(T_F_BASELINE), static_cast<double>(v.baseline_db));
    text(kMargin + 60, 128, 1, col::kGreen, b);
    if (v.baseline_simdef) text(kMargin + 60 + static_cast<int>(strlen(b)) * 6 + 6, 128, 1, col::kYellow, t(T_BASELINE_SIMDEF));
  } else {
    text(kMargin + 60, 128, 1, col::kOrange, t(T_BASELINE_NONE));
  }

  // log + sound
  tft_.fillRect(kMargin, 140, kW - 2 * kMargin, 10, col::kBlack);
  text(kMargin, 140, 1, col::kLtGrey, t(T_LOG));
  if (v.log_ok) {
    char b[24];
    snprintf(b, sizeof(b), t(T_F_LOG), static_cast<unsigned long>(v.log_lines));
    text(kMargin + 30, 140, 1, col::kWhite, b);
  } else {
    text(kMargin + 30, 140, 1, col::kOrange, t(T_LOG_OFF));
  }
  text(200, 140, 1, v.sound ? col::kLtGrey : col::kYellow, v.sound ? t(T_SOUND_ON) : t(T_SOUND_OFF));

  // transient message
  tft_.fillRect(0, 200, kW, 16, col::kBlack);
  if (v.message && v.message[0]) text(kMargin, 204, 1, col::kYellow, v.message);
}

// ------------------------------------------------------------------------ scanning

void Ui::drawScanFrame(const ScanView& v) {
  header(v.source_tag);
  clearBody();
  tft_.drawRect(kMargin, 42, kW - 2 * kMargin, 60, col::kSlate);
  tft_.drawFastHLine(kMargin + 1, 72, kW - 2 * kMargin - 2, col::kPanel);
  tft_.drawRect(kMargin, 106, kW - 2 * kMargin, 48, col::kSlate);
  text(kMargin + 4, 62, 1, col::kGrey, t(T_COLLECTING));
  text(kMargin, 224, 1, col::kCyan, t(T_HINT_SCAN));
  updateScan(v);
}

void Ui::updateScan(const ScanView& v) {
  char b[64];
  tft_.fillRect(0, 28, kW, 12, col::kBlack);
  const unsigned mm = static_cast<unsigned>(v.elapsed_s / 60), ss = static_cast<unsigned>(v.elapsed_s % 60);
  if (v.calibration) snprintf(b, sizeof(b), t(T_F_CALIB_HDR), mm, ss);
  else snprintf(b, sizeof(b), t(T_F_SCAN_HDR), static_cast<unsigned long>(v.scan_id), static_cast<unsigned>(v.segment), mm, ss);
  text(kMargin, 30, 1, col::kCyan, b);

  // segment progress bar
  const int bar_w = kW - 2 * kMargin;
  const float f = static_cast<float>(v.frames) / static_cast<float>(rs::kSegFrames);
  int fill = static_cast<int>(f * static_cast<float>(bar_w));
  if (fill < 0) fill = 0;
  if (fill > bar_w) fill = bar_w;
  tft_.drawRect(kMargin, 168, bar_w, 8, col::kSlate);
  tft_.fillRect(kMargin + 1, 169, bar_w - 2, 6, col::kBlack);
  if (fill > 2) tft_.fillRect(kMargin + 1, 169, fill - 2, 6, col::kCyan);

  // IMU + frame counters
  tft_.fillRect(0, 182, kW, 10, col::kBlack);
  text(kMargin, 182, 1, col::kLtGrey, t(T_IMU));
  text(kMargin + 30, 182, 1, imuColor(v.imu), v.imu == ImuState::kMissing ? t(T_IMU_MISSING) : imuText(v.imu));
  snprintf(b, sizeof(b), t(T_F_FRAMES), static_cast<unsigned>(v.frames), static_cast<unsigned>(rs::kSegFrames),
           static_cast<unsigned>(v.drops));
  text(150, 182, 1, col::kLtGrey, b);

  // status line
  tft_.fillRect(0, 194, kW, 28, col::kBlack);
  if (v.stalled) {
    text(kMargin, 196, 1, col::kRed, t(T_STALLED));
  } else if (v.settling) {
    text(kMargin, 196, 1, col::kYellow, t(T_SETTLING));
  } else if (v.analyzing) {
    text(kMargin, 196, 1, col::kYellow, t(T_ANALYZING));
  } else {
    text(kMargin, 196, 1, col::kYellow, t(T_LIVE_NOT_RESULT));
  }
  text(kMargin, 208, 1, col::kWhite, t(T_HOLD_STEADY));
}

void Ui::drawPreview(const rs::PreviewData& pv, const rs::SegmentResult& r) {
  (void)r;
  if (!pv.have) return;

  // ---- waveform (dominant-axis projection of the best range bin, normalised to +-1)
  const int wx = kMargin + 1, wy = 43, ww = kW - 2 * kMargin - 2, wh = 58;
  tft_.fillRect(wx, wy, ww, wh, col::kBlack);
  tft_.drawFastHLine(wx, wy + wh / 2, ww, col::kPanel);
  if (pv.wave_n >= 2) {
    const int n = pv.wave_n;
    const int step = (n > 150) ? 2 : 1;
    int px = wx, py = wy + wh / 2 - static_cast<int>(pv.wave[0] * 26.0f);
    for (int i = step; i < n; i += step) {
      const int x = wx + (i * (ww - 1)) / (n - 1);
      float a = pv.wave[i];
      if (a > 1.0f) a = 1.0f;
      if (a < -1.0f) a = -1.0f;
      const int y = wy + wh / 2 - static_cast<int>(a * 26.0f);
      tft_.drawLine(px, py, x, y, col::kGreen);
      px = x; py = y;
    }
  }
  char b[32];
  snprintf(b, sizeof(b), "bin %u", static_cast<unsigned>(pv.best_bin));
  text(wx + 3, wy + 2, 1, col::kGrey, b);

  // ---- folded Doppler spectrum with the CFAR threshold of that bin
  const int sx = kMargin + 1, sy = 107, sw = kW - 2 * kMargin - 2, sh = 46;
  tft_.fillRect(sx, sy, sw, sh, col::kBlack);
  if (pv.n_cells >= 2) {
    float top = -1000.0f;
    for (int c = 0; c < pv.n_cells; c++) {
      if (pv.spec_db[c] > top) top = pv.spec_db[c];
      if (pv.thr_db[c] > -100.0f && pv.thr_db[c] > top) top = pv.thr_db[c];
    }
    top += 3.0f;
    const float range_db = 30.0f;
    int peak_c = 0;
    for (int c = 0; c < pv.n_cells; c++) if (pv.spec_db[c] > pv.spec_db[peak_c]) peak_c = c;
    for (int c = 0; c < pv.n_cells; c++) {
      const int x0 = sx + (c * sw) / pv.n_cells;
      const int x1 = sx + ((c + 1) * sw) / pv.n_cells;
      float fh = (pv.spec_db[c] - (top - range_db)) / range_db;
      if (fh < 0.0f) fh = 0.0f;
      if (fh > 1.0f) fh = 1.0f;
      const int h = static_cast<int>(fh * static_cast<float>(sh - 2));
      const bool over = pv.thr_db[c] > -100.0f && pv.spec_db[c] > pv.thr_db[c];
      if (h > 0) tft_.fillRect(x0, sy + sh - 1 - h, (x1 - x0 > 1) ? (x1 - x0 - 1) : 1, h, over ? col::kGreen : col::kGrey);
      if (pv.thr_db[c] > -100.0f) {
        float ft = (pv.thr_db[c] - (top - range_db)) / range_db;
        if (ft < 0.0f) ft = 0.0f;
        if (ft > 1.0f) ft = 1.0f;
        tft_.drawFastHLine(x0, sy + sh - 1 - static_cast<int>(ft * static_cast<float>(sh - 2)), (x1 - x0), col::kRed);
      }
    }
    // axis labels: breaths per minute at both ends and at the strongest cell
    tft_.fillRect(kMargin, 156, kW - 2 * kMargin, 9, col::kBlack);
    const float bpm0 = 60.0f * static_cast<float>(pv.k0) * pv.df_hz;
    const float bpm1 = 60.0f * static_cast<float>(pv.k0 + pv.n_cells - 1) * pv.df_hz;
    const float bpmp = 60.0f * static_cast<float>(pv.k0 + peak_c) * pv.df_hz;
    snprintf(b, sizeof(b), "%.0f/min", static_cast<double>(bpm0));
    text(kMargin, 157, 1, col::kGrey, b);
    snprintf(b, sizeof(b), "%.0f/min", static_cast<double>(bpm1));
    text(kW - kMargin - static_cast<int>(strlen(b)) * 6, 157, 1, col::kGrey, b);
    snprintf(b, sizeof(b), "peak %.1f/min", static_cast<double>(bpmp));
    text((kW - static_cast<int>(strlen(b)) * 6) / 2, 157, 1, col::kLtGrey, b);
  }
}

// ------------------------------------------------------------------------ result

void Ui::drawResult(const ResultView& v) {
  const rs::ScanOutcome& o = v.out;
  header(v.source_tag);
  clearBody();

  // ---- banner: verdict headline (size 2). The negative verdict adds the mandatory second line.
  const uint16_t bc = verdictColor(o.verdict);
  tft_.fillRect(kBannerX, 30, kBannerW, 72, bc);
  tft_.drawRect(kBannerX, 30, kBannerW, 72, col::kWhite);
  const bool negative = (o.verdict == rs::Verdict::kNoPattern);
  const int lines = wrap(kBannerTextX, 35, kCols2, 2, col::kWhite, verdictHeadline(o.verdict), 3);
  if (negative) text(kBannerTextX, 35 + lines * 16, 2, col::kYellow, negativeBannerLine());

  // ---- evidence block (size 1)
  int y = 108;
  char b[64];
  const bool has_line = (o.verdict == rs::Verdict::kCandidate || o.verdict == rs::Verdict::kPossible ||
                         (o.verdict == rs::Verdict::kConfounded && o.hz > 0.0f));
  if (has_line) {
    snprintf(b, sizeof(b), t(T_F_EVID_RANGE), static_cast<double>(v.range_m), static_cast<double>(o.bpm));
    text(kMargin, y, 1, col::kWhite, b);
    y += 12;
    snprintf(b, sizeof(b), t(T_F_EVID_SNR), static_cast<double>(o.snr_db), static_cast<double>(o.margin_db),
             static_cast<unsigned>(o.agree));
    text(kMargin, y, 1, col::kLtGrey, b);
    y += 12;
  } else {
    snprintf(b, sizeof(b), "Segments: %u valid / %u invalid", static_cast<unsigned>(o.n_valid), static_cast<unsigned>(o.n_invalid));
    text(kMargin, y, 1, col::kLtGrey, b);
    y += 12;
  }
  if (o.n_valid > 0) {
    if (o.sens_known) {
      snprintf(b, sizeof(b), t(T_F_FLOOR_BASE), static_cast<double>(o.floor_db), static_cast<double>(o.floor_db - o.floor_delta_db),
               diff1(o.floor_delta_db));
      text(kMargin, y, 1, o.low_sens ? col::kOrange : col::kLtGrey, b);
    } else {
      snprintf(b, sizeof(b), t(T_F_FLOOR_ONLY), static_cast<double>(o.floor_db));
      text(kMargin, y, 1, col::kLtGrey, b);
    }
    y += 12;
    if (!o.sens_known) {
      text(kMargin, y, 1, col::kOrange, t(T_SENS_UNKNOWN));
      y += 12;
    }
  }
  if (o.vibration) {
    snprintf(b, sizeof(b), t(T_F_VIB), static_cast<double>(o.vibration_hz), static_cast<double>(v.platform_mm));
    text(kMargin, y, 1, col::kYellow, b);
    y += 12;
  }

  // ---- what to do
  const int act_y = 168;
  wrap(kMargin, act_y, kCols1, 1, col::kYellow, verdictAction(o.verdict, o.last_invalid), 3);

  // ---- permanent reminder + keys
  tft_.fillRect(0, 210, kW, 12, col::kBlack);
  text(kMargin, 211, 1, col::kOrange, t(T_NEGATIVE_NOTE));
  hint(t(T_HINT_RESULT));
}

void Ui::drawCalibResult(const CalibView& v) {
  header(v.source_tag);
  clearBody();
  const uint16_t bc = v.ok ? col::kDkGreen : col::kDkRed;
  tft_.fillRect(kMargin, 30, kW - 2 * kMargin, 40, bc);
  tft_.drawRect(kMargin, 30, kW - 2 * kMargin, 40, col::kWhite);
  text(16, 36, 2, col::kWhite, v.ok ? t(T_CALIB_OK) : t(T_CALIB_FAIL));
  char b[64];
  if (v.ok) {
    snprintf(b, sizeof(b), t(T_F_CALIB_VAL), static_cast<double>(v.floor_db));
    text(kMargin, 80, 1, col::kWhite, b);
    if (v.lines_seen) wrap(kMargin, 96, kCols1, 1, col::kOrange,
                           "Periodic content was present: the baseline may be pessimistic. Repeat in a quieter spot.", 3);
  } else {
    wrap(kMargin, 80, kCols1, 1, col::kYellow, verdictAction(rs::Verdict::kInvalid, v.why), 3);
  }
  wrap(kMargin, 140, kCols1, 1, col::kLtGrey, t(T_CALIB_HINT), 2);
  hint(t(T_HINT_ANY));
}

// ------------------------------------------------------------------------ menu, self-test, about, message

void Ui::drawMenu(const MenuView& v) {
  header(v.source_tag);
  clearBody();
  text(kMargin, 32, 2, col::kWhite, t(T_MENU_TITLE));
  for (uint8_t i = 0; i < v.n && i < MenuView::kMax; i++) {
    const int y = 56 + 20 * i;
    const bool sel = (i == v.sel);
    if (sel) tft_.fillRect(kMargin, y - 2, kW - 2 * kMargin, 18, col::kNavy);
    text(kMargin + 4, y, 2, sel ? col::kYellow : col::kLtGrey, v.item[i]);
  }
  hint(t(T_HINT_MENU));
}

void Ui::drawSelfTest(const SelfTestView& v) {
  header(v.source_tag);
  clearBody();
  text(kMargin, 32, 2, col::kWhite, t(T_ST_TITLE));
  for (uint8_t i = 0; i < v.n && i < SelfTestView::kMax; i++) {
    const int y = 58 + 16 * i;
    text(kMargin, y, 1, col::kLtGrey, v.name[i]);
    text(150, y, 1, v.ok[i] ? col::kGreen : col::kRed, v.ok[i] ? t(T_ST_OK) : t(T_ST_FAIL));
    text(190, y, 1, col::kGrey, v.detail[i]);
  }
  hint(t(T_HINT_ANY));
}

void Ui::drawAbout(const char* source_tag) {
  header(source_tag);
  clearBody();
  text(kMargin, 36, 1, col::kWhite, t(T_ABOUT_1));
  text(kMargin, 56, 1, col::kOrange, t(T_ABOUT_2));
  text(kMargin, 68, 1, col::kOrange, t(T_ABOUT_3));
  text(kMargin, 88, 1, col::kYellow, t(T_ABOUT_4));
  char b[48];
  snprintf(b, sizeof(b), "firmware %s%s", RS_FW_VERSION, RS_WOKWI ? " (Wokwi build)" : "");
  text(kMargin, 116, 1, col::kLtGrey, b);
  hint(t(T_HINT_ANY));
}

void Ui::drawMessage(const char* title, const char* msg, uint16_t color, const char* source_tag) {
  header(source_tag);
  clearBody();
  tft_.fillRect(kMargin, 40, kW - 2 * kMargin, 60, color);
  tft_.drawRect(kMargin, 40, kW - 2 * kMargin, 60, col::kWhite);
  wrap(16, 46, kCols2 - 2, 2, col::kWhite, title, 2);
  wrap(kMargin, 112, kCols1, 1, col::kLtGrey, msg, 5);
  hint(t(T_HINT_ANY));
}
