// RubbleSense firmware - operator display (ILI9341, 320x240 landscape).
//
// All drawing lives here. The application hands over plain view structs, so every screen can be
// rendered in the host test-bench with canned data and inspected as an image.
#pragma once
#include <stdint.h>
#include "rs_decision.h"
#include "rs_dsp.h"
#include "rs_types.h"

class Adafruit_ILI9341;

// Palette (RGB565). Namespace-scope constants: no ODR / link issues on older C++ standards.
namespace col {
constexpr uint16_t kBlack = 0x0000, kWhite = 0xFFFF, kNavy = 0x000F, kCyan = 0x07FF, kYellow = 0xFFE0,
                   kGreen = 0x07E0, kDkGreen = 0x03E0, kRed = 0xF800, kOrange = 0xFD20, kGrey = 0x7BEF,
                   kSlate = 0x39E7, kPanel = 0x2104, kLtGrey = 0xC618, kAmber = 0xCC00, kPurple = 0x780F,
                   kBrown = 0xAA20, kDkRed = 0x9000;
}  // namespace col

enum class ImuState : uint8_t { kOk, kMoving, kMissing, kSimulated, kHost };

struct IdleView {
  const char* source_tag = "SIM";     // "SIM" / "SERIAL"
  bool        sim = true;
  const char* trial_name = "";
  bool        manual = false;
  bool        serial_ready = false;
  ImuState    imu = ImuState::kOk;
  bool        baseline_known = false;
  float       baseline_db = 0.0f;
  bool        baseline_simdef = false;
  bool        log_ok = true;
  uint32_t    log_lines = 0;
  bool        sound = true;
  const char* message = "";           // one transient line (feedback for the last action)
};

struct ScanView {
  bool        calibration = false;
  uint32_t    scan_id = 0;
  uint8_t     segment = 1;            // 1-based index of the segment being acquired
  uint32_t    elapsed_s = 0;
  uint16_t    frames = 0;             // stored in the current segment
  uint16_t    drops = 0;
  ImuState    imu = ImuState::kOk;
  bool        settling = false;
  bool        analyzing = false;
  bool        stalled = false;
  const char* source_tag = "SIM";
};

struct ResultView {
  uint32_t         scan_id = 0;
  rs::ScanOutcome  out{};
  float            range_m = 0.0f;
  float            platform_mm = 0.0f;
  const char*      evidence = "none";
  const char*      source_tag = "SIM";
};

struct CalibView {
  bool  ok = false;
  float floor_db = 0.0f;
  bool  lines_seen = false;           // periodic content was present: the baseline may be pessimistic
  rs::InvalidReason why = rs::InvalidReason::kNone;
  const char* source_tag = "SIM";
};

struct MenuView {
  static constexpr uint8_t kMax = 8;
  char    item[kMax][40] = {};
  uint8_t n = 0;
  uint8_t sel = 0;
  const char* source_tag = "SIM";
};

struct SelfTestView {
  static constexpr uint8_t kMax = 8;
  char    name[kMax][24] = {};
  char    detail[kMax][28] = {};
  bool    ok[kMax] = {};
  uint8_t n = 0;
  const char* source_tag = "SIM";
};

class Ui {
 public:
  explicit Ui(Adafruit_ILI9341& tft) : tft_(tft) {}

  void begin();

  void drawIdle(const IdleView& v);
  void updateIdleStatus(const IdleView& v);       // partial refresh of the live rows

  void drawScanFrame(const ScanView& v);
  void updateScan(const ScanView& v);
  void drawPreview(const rs::PreviewData& pv, const rs::SegmentResult& r);

  void drawResult(const ResultView& v);
  void drawCalibResult(const CalibView& v);
  void drawMenu(const MenuView& v);
  void drawSelfTest(const SelfTestView& v);
  void drawAbout(const char* source_tag);
  void drawMessage(const char* title, const char* text, uint16_t color, const char* source_tag);

  // Number of wrapped texts that did not fit their box so far (the test-bench requires 0).
  uint32_t truncations() const { return truncated_; }

 private:
  void header(const char* source_tag);
  void clearBody();
  void text(int x, int y, uint8_t size, uint16_t color, const char* s);
  int  wrap(int x, int y, int cols, uint8_t size, uint16_t color, const char* s, int max_lines);
  void imuLine(int y, ImuState s);
  void sourceRow(const IdleView& v);                // "serial I/Q: waiting / connected" (SERIAL source only)
  void hint(const char* s);
  static uint16_t verdictColor(rs::Verdict v);

  Adafruit_ILI9341& tft_;
  uint32_t truncated_ = 0;
  int8_t   src_ready_shown_ = -1;                   // what the source row currently shows (-1 = nothing)
};
