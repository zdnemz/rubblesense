// Test-bench control surface of the host stubs. NOT part of the firmware.
//
// The stubs replace the Arduino-ESP32 core, the I2C bus (with a small MPU-6050 register model),
// NVS (Preferences), LittleFS and the ILI9341 display (a recorded display list plus a pixel
// ownership map) so that the unmodified firmware sources can run on a PC under a virtual
// clock. They model just enough hardware behaviour to catch logic and layout errors; they do
// NOT prove that the firmware works on a real ESP32 (timing, stack sizes, cache, drivers).
#pragma once
#include <stdint.h>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace stub {

// Restores every stub to its power-on state (clock 0, pins released, empty NVS/FS, healthy IMU).
void reset();
// A reboot: RAM-side state is cleared (clock, pins, ports, display, sensor registers) while NVS,
// the file system and the physical sensor conditions (stub::mpu()) survive.
void softReset();

// ---- virtual clock ---------------------------------------------------------------------------
uint64_t nowUs();
void     setNowUs(uint64_t us);                // jump the clock (no background calls), e.g. to test counter wrap-around
// Advances time. The background hook (the "acquisition thread") runs once per step_us.
void advanceUs(uint64_t us);
void setBackground(void (*fn)(void*), void* ctx, uint32_t step_us = 1000);
void clearBackground();

// ---- GPIO / LEDC -----------------------------------------------------------------------------
void     setInput(uint8_t pin, int level);     // what digitalRead() returns (default HIGH: pulled up)
int      output(uint8_t pin);                  // last digitalWrite() value
void     setAnalog(uint8_t pin, int value);    // 0..4095
uint32_t buzzerHz();                           // last ledcWriteTone() frequency
uint32_t toneCount();                          // number of ledcWriteTone() calls

bool restartRequested();                       // ESP.restart() was called
void clearRestart();

// ---- MPU-6050 on the I2C bus -----------------------------------------------------------------
struct Mpu {
  bool     present = true;
  uint8_t  addr = 0x68;
  uint8_t  who = 0x68;
  bool     nack_reads = false;                 // bus error on the 14-byte sample burst
  bool     frozen = false;                     // returns the previous block again (stuck sensor)
  bool     config_sticks = true;               // false: writes to the range registers are ignored
  float    accel[3] = {0.0f, 0.0f, 9.80665f};  // m/s^2, sensor frame
  float    gyro[3] = {0.0f, 0.0f, 0.0f};       // rad/s
  float    noise_a = 0.02f;                    // m/s^2 rms
  float    noise_g = 0.002f;                   // rad/s rms
  uint32_t reads = 0;                          // sample bursts served
  // Optional scripted motion: called for every sample burst with the time in seconds; may
  // modify a[] and g[] (added to the base values above).
  std::function<void(double t_s, float a[3], float g[3])> script;
};
Mpu& mpu();

// ---- NVS / LittleFS --------------------------------------------------------------------------
void clearNvs();

struct Fs {
  std::map<std::string, std::string> files;
  size_t capacity = 1024u * 1024u;             // bytes in total; writes beyond it are short
  bool   mount_ok = true;
  bool   fail_open = false;
  uint32_t mounts = 0;
  size_t used() const {
    size_t n = 0;
    for (std::map<std::string, std::string>::const_iterator i = files.begin(); i != files.end(); ++i) n += i->second.size();
    return n;
  }
};
Fs& fs();

// ---- display ---------------------------------------------------------------------------------
struct GfxOp {
  char        kind;      // S screen fill, F fill rect, R rect outline, H hline, V vline, L line, T text, P pixel
  int16_t     x, y, w, h;   // L: (x,y)-(w,h) is the end point; H: w = length; V: h = length
  uint16_t    color;
  uint8_t     size;      // text only
  std::string text;      // text only
};

class Gfx {
 public:
  static const int kW = 320;
  static const int kH = 240;

  void reset();

  // recording (called by the Adafruit stubs)
  void screen(uint16_t color);
  void fillRect(int x, int y, int w, int h, uint16_t color);
  void rect(int x, int y, int w, int h, uint16_t color);
  void hline(int x, int y, int w, uint16_t color);
  void vline(int x, int y, int h, uint16_t color);
  void line(int x0, int y0, int x1, int y1, uint16_t color);
  void glyph(int x, int y, uint8_t size, uint16_t color, char c);
  void flushText();

  // queries
  const std::vector<GfxOp>& opList();
  uint32_t clippedGlyphs() const { return clipped_; }
  uint32_t partialOverdraws();                     // text runs only partly covered by later drawing
  uint32_t textOverText() const { return text_over_text_; }   // text printed on top of still-visible text
  const std::string& lastOverlap() const { return last_overlap_; }   // what the latest such overlap was
  uint32_t textOverLine() const { return text_over_line_; }   // text printed over an outline / line that is still visible
  uint32_t lineOverText() const { return line_over_text_; }   // an outline / line drawn across visible text
  const std::string& lastLineOverlap() const { return last_line_overlap_; }
  const std::string& lastClip() const { return last_clip_; }         // the latest glyph that fell off the screen
  std::string visibleText();                       // visible text runs, reading order, '\n' per run
  std::string flatText();                          // same, joined with single spaces
  bool        contains(const char* needle);        // needle found in flatText()
  uint32_t    opCount() const { return static_cast<uint32_t>(ops_.size()); }
  bool        dump(const std::string& path);       // display list for tools/render_screens.py
  uint16_t    colorAt(int x, int y);               // colour of a pixel (text cells are not resolved)
  uint32_t    beginCalls = 0;
  uint8_t     rotation = 0;
  bool        enabled = true;                      // false: ignore all drawing (speed)

 private:
  void  own(int x, int y, int w, int h, int32_t id);
  bool  textAt(int x, int y) const;                // a text glyph cell owns this pixel
  void  noteLineHitsText(int x, int y, int w, int h);   // run of pixels about to be owned by a line
  void  maybeCompact();
  float visibleFraction(size_t i) const;

  std::vector<GfxOp>   ops_;
  std::vector<int32_t> owner_;
  std::vector<uint16_t> pix_;
  uint32_t clipped_ = 0;
  uint32_t text_over_text_ = 0;
  uint32_t text_over_line_ = 0;
  uint32_t line_over_text_ = 0;
  std::string last_overlap_;
  std::string last_line_overlap_;
  std::string last_clip_;
  // pending text run
  bool        run_open_ = false;
  int         run_x_ = 0, run_y_ = 0, run_next_x_ = 0;
  uint8_t     run_size_ = 1;
  uint16_t    run_color_ = 0;
  std::string run_text_;
};
Gfx& gfx();

}  // namespace stub
