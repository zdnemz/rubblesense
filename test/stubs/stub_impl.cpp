// Implementation of the host stubs (see stub_hw.h). C++17 is fine here: only the firmware
// sources must stay C++11.
#include "stub_hw.h"

#include <Adafruit_ILI9341.h>
#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <SPI.h>
#include <Wire.h>
#include <algorithm>
#include <math.h>
#include <random>
#include <stdio.h>
#include <string.h>

// ============================================================================ file-scope state
namespace {

uint64_t g_now_us = 0;
void (*g_bg)(void*) = nullptr;
void*    g_bg_ctx = nullptr;
uint32_t g_bg_step = 1000;
uint64_t g_bg_acc = 0;
bool     g_in_bg = false;

const int kPins = 48;
int       g_in[kPins];
int       g_out[kPins];
int       g_analog[kPins];
uint32_t  g_tone_hz = 0;
uint32_t  g_tone_count = 0;
bool      g_restart = false;

typedef std::map<std::string, std::map<std::string, std::string> > NvsMap;
NvsMap& nvs() {
  static NvsMap m;
  return m;
}

// MPU-6050 register file + last sample block
uint8_t g_mpu_reg[128];
uint8_t g_mpu_last[14];
bool    g_mpu_have_last = false;
std::mt19937 g_rng(12345);
std::normal_distribution<float> g_norm(0.0f, 1.0f);

void mpuPowerOn() {
  memset(g_mpu_reg, 0, sizeof(g_mpu_reg));
  g_mpu_reg[0x6B] = 0x40;                // powers up asleep, like the real part
  g_mpu_have_last = false;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  if (reg >= 128) return;
  if (reg == 0x75) return;               // WHO_AM_I is read-only
  if (reg == 0x6B && (val & 0x80)) {     // DEVICE_RESET: registers back to power-on values, asleep
    mpuPowerOn();
    return;
  }
  if ((reg == 0x1B || reg == 0x1C) && !stub::mpu().config_sticks) return;
  g_mpu_reg[reg] = val;
}

int16_t rawOf(float v, float lsb_per_unit) {
  const float x = v * lsb_per_unit;
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return static_cast<int16_t>(lrintf(x));
}

void putBe(uint8_t* p, int16_t v) {
  p[0] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);
}

void mpuSample(uint8_t out[14]) {
  stub::Mpu& m = stub::mpu();
  m.reads++;
  if (m.frozen && g_mpu_have_last) {
    memcpy(out, g_mpu_last, 14);
    return;
  }
  if (g_mpu_reg[0x6B] & 0x40) {          // asleep: data registers read as zero
    memset(out, 0, 14);
  } else {
    float a[3], g[3];
    for (int k = 0; k < 3; k++) {
      a[k] = m.accel[k];
      g[k] = m.gyro[k];
    }
    if (m.script) m.script(static_cast<double>(g_now_us) * 1e-6, a, g);
    for (int k = 0; k < 3; k++) {
      a[k] += m.noise_a * g_norm(g_rng);
      g[k] += m.noise_g * g_norm(g_rng);
    }
    static const float kAccelLsbPerG[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    static const float kGyroLsbPerDps[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    const float as = kAccelLsbPerG[(g_mpu_reg[0x1C] >> 3) & 3] / 9.80665f;                       // LSB per m/s^2
    const float gs = kGyroLsbPerDps[(g_mpu_reg[0x1B] >> 3) & 3] * (180.0f / 3.14159265f);        // LSB per rad/s
    putBe(out + 0, rawOf(a[0], as));
    putBe(out + 2, rawOf(a[1], as));
    putBe(out + 4, rawOf(a[2], as));
    putBe(out + 6, rawOf((25.0f - 36.53f) * 340.0f / 340.0f, 340.0f));   // ~25 C
    putBe(out + 8, rawOf(g[0], gs));
    putBe(out + 10, rawOf(g[1], gs));
    putBe(out + 12, rawOf(g[2], gs));
  }
  memcpy(g_mpu_last, out, 14);
  g_mpu_have_last = true;
}

uint8_t mpuRead(uint8_t reg, const uint8_t block[14]) {
  if (reg >= 0x3B && reg <= 0x48) return block[reg - 0x3B];
  if (reg == 0x75) return stub::mpu().who;
  return reg < 128 ? g_mpu_reg[reg] : 0;
}

}  // namespace

// ============================================================================ Arduino core
uint32_t millis() { return static_cast<uint32_t>(g_now_us / 1000u); }
uint32_t micros() { return static_cast<uint32_t>(g_now_us); }
void delay(uint32_t ms) { stub::advanceUs(static_cast<uint64_t>(ms) * 1000u); }
void delayMicroseconds(uint32_t us) { stub::advanceUs(us); }

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t val) {
  if (pin < kPins) g_out[pin] = val ? 1 : 0;
}
int digitalRead(uint8_t pin) { return pin < kPins ? g_in[pin] : 0; }
int analogRead(uint8_t pin) { return pin < kPins ? g_analog[pin] : 0; }

#if defined(STUB_ARDUINO_V3)
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
uint32_t ledcWriteTone(uint8_t, uint32_t freq) {
  g_tone_hz = freq;
  g_tone_count++;
  return freq;
}
#else
double ledcSetup(uint8_t, double freq, uint8_t) { return freq; }
void   ledcAttachPin(uint8_t, uint8_t) {}
double ledcWriteTone(uint8_t, double freq) {
  g_tone_hz = static_cast<uint32_t>(freq);
  g_tone_count++;
  return freq;
}
#endif

HardwareSerial Serial;
HardwareSerial Serial2;
EspClass       ESP;
void EspClass::restart() { g_restart = true; }

int HardwareSerial::available() { return static_cast<int>(rx_.size() - rx_pos_); }
int HardwareSerial::read() {
  if (rx_pos_ >= rx_.size()) return -1;
  const int c = static_cast<uint8_t>(rx_[rx_pos_++]);
  if (rx_pos_ == rx_.size()) {
    rx_.clear();
    rx_pos_ = 0;
  }
  return c;
}
int HardwareSerial::peek() { return rx_pos_ < rx_.size() ? static_cast<uint8_t>(rx_[rx_pos_]) : -1; }
size_t HardwareSerial::write(uint8_t c) {
  tx_.push_back(static_cast<char>(c));
  return 1;
}
size_t HardwareSerial::write(const uint8_t* b, size_t n) {
  tx_.append(reinterpret_cast<const char*>(b), n);
  return n;
}
// Like the real driver: when its ring is full the new bytes are lost (and counted).
void HardwareSerial::injectRx(const uint8_t* b, size_t n) {
  const size_t used = rx_.size() - rx_pos_;
  const size_t room = used < rx_cap_ ? rx_cap_ - used : 0;
  const size_t take = n < room ? n : room;
  rx_.append(reinterpret_cast<const char*>(b), take);
  rx_dropped_ += static_cast<uint32_t>(n - take);
}

SPIClass SPI;

// ============================================================================ I2C
TwoWire Wire;

bool TwoWire::begin(int sda, int scl, uint32_t) {
  began_ = true;
  sda_ = sda;
  scl_ = scl;
  return true;
}
void TwoWire::beginTransmission(uint8_t address) {
  addr_ = address;
  tx_.clear();
}
uint8_t TwoWire::endTransmission(void) { return endTransmission(true); }
uint8_t TwoWire::endTransmission(bool) {
  stub::Mpu& m = stub::mpu();
  const std::vector<uint8_t> tx = tx_;
  tx_.clear();
  if (!began_) return 4;
  if (!m.present || addr_ != m.addr) return 2;      // NACK on address
  if (!tx.empty()) {
    reg_ptr_ = tx[0];
    for (size_t i = 1; i < tx.size(); i++) mpuWrite(static_cast<uint8_t>(reg_ptr_ + i - 1), tx[i]);
  }
  return 0;
}
size_t TwoWire::doRequest(uint8_t address, size_t len) {
  stub::Mpu& m = stub::mpu();
  rx_.clear();
  rx_pos_ = 0;
  if (!began_ || !m.present || address != m.addr) return 0;
  uint8_t block[14];
  memset(block, 0, sizeof(block));
  if (reg_ptr_ >= 0x3B && reg_ptr_ <= 0x48) {
    if (m.nack_reads) return 0;
    mpuSample(block);
  }
  for (size_t i = 0; i < len; i++) rx_.push_back(mpuRead(static_cast<uint8_t>(reg_ptr_ + i), block));
  reg_ptr_ = static_cast<uint8_t>(reg_ptr_ + len);
  return rx_.size();
}
size_t  TwoWire::requestFrom(uint16_t a, size_t n, bool) { return doRequest(static_cast<uint8_t>(a), n); }
uint8_t TwoWire::requestFrom(uint16_t a, uint8_t n, bool) { return static_cast<uint8_t>(doRequest(static_cast<uint8_t>(a), n)); }
uint8_t TwoWire::requestFrom(uint16_t a, uint8_t n, uint8_t) { return static_cast<uint8_t>(doRequest(static_cast<uint8_t>(a), n)); }
size_t  TwoWire::requestFrom(uint8_t a, size_t n, bool) { return doRequest(a, n); }
uint8_t TwoWire::requestFrom(uint16_t a, uint8_t n) { return static_cast<uint8_t>(doRequest(static_cast<uint8_t>(a), n)); }
uint8_t TwoWire::requestFrom(uint8_t a, uint8_t n, uint8_t) { return static_cast<uint8_t>(doRequest(a, n)); }
uint8_t TwoWire::requestFrom(uint8_t a, uint8_t n) { return static_cast<uint8_t>(doRequest(a, n)); }
uint8_t TwoWire::requestFrom(int a, int n, int) { return static_cast<uint8_t>(doRequest(static_cast<uint8_t>(a), static_cast<size_t>(n))); }
uint8_t TwoWire::requestFrom(int a, int n) { return static_cast<uint8_t>(doRequest(static_cast<uint8_t>(a), static_cast<size_t>(n))); }
size_t  TwoWire::write(uint8_t b) {
  tx_.push_back(b);
  return 1;
}
size_t TwoWire::write(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) tx_.push_back(b[i]);
  return n;
}
int TwoWire::available(void) { return static_cast<int>(rx_.size() - rx_pos_); }
int TwoWire::read(void) { return rx_pos_ < rx_.size() ? rx_[rx_pos_++] : -1; }
int TwoWire::peek(void) { return rx_pos_ < rx_.size() ? rx_[rx_pos_] : -1; }

// ============================================================================ NVS
bool Preferences::begin(const char* name, bool readOnly, const char*) {
  ns_ = name ? name : "";
  ro_ = readOnly;
  open_ = true;
  nvs()[ns_];
  return true;
}
void Preferences::end() { open_ = false; }
bool Preferences::clear() {
  if (!open_ || ro_) return false;
  nvs()[ns_].clear();
  return true;
}

namespace {
template <typename T>
T prefGet(const std::string& ns, bool open, const char* key, T dflt) {
  if (!open) return dflt;
  std::map<std::string, std::string>& m = nvs()[ns];
  std::map<std::string, std::string>::const_iterator it = m.find(key);
  if (it == m.end() || it->second.size() != sizeof(T)) return dflt;
  T v;
  memcpy(&v, it->second.data(), sizeof(T));
  return v;
}
template <typename T>
size_t prefPut(const std::string& ns, bool open, bool ro, const char* key, T v) {
  if (!open || ro) return 0;
  nvs()[ns][key] = std::string(reinterpret_cast<const char*>(&v), sizeof(T));
  return sizeof(T);
}
}  // namespace

uint8_t  Preferences::getUChar(const char* k, uint8_t d) { return prefGet<uint8_t>(ns_, open_, k, d); }
uint32_t Preferences::getUInt(const char* k, uint32_t d) { return prefGet<uint32_t>(ns_, open_, k, d); }
float    Preferences::getFloat(const char* k, float d) { return prefGet<float>(ns_, open_, k, d); }
size_t   Preferences::putUChar(const char* k, uint8_t v) { return prefPut<uint8_t>(ns_, open_, ro_, k, v); }
size_t   Preferences::putUInt(const char* k, uint32_t v) { return prefPut<uint32_t>(ns_, open_, ro_, k, v); }
size_t   Preferences::putFloat(const char* k, float v) { return prefPut<float>(ns_, open_, ro_, k, v); }

// ============================================================================ LittleFS
fs::LittleFSFS LittleFS;

namespace fs {

bool LittleFSFS::begin(bool, const char*, uint8_t, const char*) {
  stub::fs().mounts++;
  return stub::fs().mount_ok;
}

File FS::open(const char* path, const char* mode, const bool) {
  stub::Fs& f = stub::fs();
  if (!path || !mode || f.fail_open) return File();
  const std::string p(path);
  switch (mode[0]) {
    case 'r': {
      if (f.files.find(p) == f.files.end()) return File();
      return File(p, true, false);
    }
    case 'w':
      f.files[p].clear();
      return File(p, false, true);
    case 'a':
      f.files[p];                      // creates the file when missing
      return File(p, false, true);
    default: return File();
  }
}
bool FS::exists(const char* path) { return path && stub::fs().files.count(path) != 0; }
bool FS::remove(const char* path) { return path && stub::fs().files.erase(path) != 0; }
bool FS::rename(const char* from, const char* to) {
  stub::Fs& f = stub::fs();
  if (!from || !to) return false;
  std::map<std::string, std::string>::iterator it = f.files.find(from);
  if (it == f.files.end()) return false;
  f.files[to] = it->second;
  if (std::string(from) != std::string(to)) f.files.erase(from);
  return true;
}

size_t File::size() const {
  if (!open_) return 0;
  std::map<std::string, std::string>::const_iterator it = stub::fs().files.find(path_);
  return it == stub::fs().files.end() ? 0 : it->second.size();
}
size_t File::read(uint8_t* buf, size_t n) {
  if (!open_ || !rd_) return 0;
  std::map<std::string, std::string>::const_iterator it = stub::fs().files.find(path_);
  if (it == stub::fs().files.end() || pos_ >= it->second.size()) return 0;
  const size_t k = std::min(n, it->second.size() - pos_);
  memcpy(buf, it->second.data() + pos_, k);
  pos_ += k;
  return k;
}
int File::read() {
  uint8_t c;
  return read(&c, 1) == 1 ? c : -1;
}
int File::peek() {
  const size_t p = pos_;
  const int c = read();
  pos_ = p;
  return c;
}
int File::available() { return static_cast<int>(size() > pos_ ? size() - pos_ : 0); }
size_t File::write(uint8_t c) { return write(&c, 1); }
size_t File::write(const uint8_t* buf, size_t n) {
  if (!open_ || !wr_) return 0;
  stub::Fs& f = stub::fs();
  const size_t used = f.used();
  const size_t room = used < f.capacity ? f.capacity - used : 0;
  const size_t k = std::min(n, room);
  f.files[path_].append(reinterpret_cast<const char*>(buf), k);
  return k;
}

}  // namespace fs

// ============================================================================ display
void Adafruit_GFX::setRotation(uint8_t r) {
  r &= 3;
  if (r & 1) {
    width_ = raw_h_;
    height_ = raw_w_;
  } else {
    width_ = raw_w_;
    height_ = raw_h_;
  }
  stub::gfx().rotation = r;
}
void Adafruit_ILI9341::begin(uint32_t) { stub::gfx().beginCalls++; }

void Adafruit_GFX::fillScreen(uint16_t c) { stub::gfx().screen(c); }
void Adafruit_GFX::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) { stub::gfx().fillRect(x, y, w, h, c); }
void Adafruit_GFX::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) { stub::gfx().hline(x, y, w, c); }
void Adafruit_GFX::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) { stub::gfx().vline(x, y, h, c); }
void Adafruit_GFX::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t c) { stub::gfx().line(x0, y0, x1, y1, c); }
void Adafruit_GFX::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) { stub::gfx().rect(x, y, w, h, c); }
size_t Adafruit_GFX::write(uint8_t c) {
  if (c == '\n') {
    stub::gfx().flushText();
    cx_ = 0;
    cy_ = static_cast<int16_t>(cy_ + ts_ * 8);
  } else if (c != '\r') {
    if (wrap_ && (cx_ + ts_ * 6) > width_) {
      cx_ = 0;
      cy_ = static_cast<int16_t>(cy_ + ts_ * 8);
    }
    stub::gfx().glyph(cx_, cy_, ts_, tc_, static_cast<char>(c));
    cx_ = static_cast<int16_t>(cx_ + ts_ * 6);
  }
  return 1;
}

namespace stub {

void Gfx::reset() {
  ops_.clear();
  owner_.assign(static_cast<size_t>(kW) * kH, -1);
  pix_.assign(static_cast<size_t>(kW) * kH, 0);
  clipped_ = 0;
  text_over_text_ = 0;
  text_over_line_ = 0;
  line_over_text_ = 0;
  last_line_overlap_.clear();
  last_overlap_.clear();
  last_clip_.clear();
  run_open_ = false;
  run_text_.clear();
  beginCalls = 0;
  rotation = 0;
  enabled = true;
}

void Gfx::own(int x, int y, int w, int h, int32_t id) {
  const int x0 = std::max(x, 0), y0 = std::max(y, 0);
  const int x1 = std::min(x + w, static_cast<int>(kW)), y1 = std::min(y + h, static_cast<int>(kH));
  for (int yy = y0; yy < y1; yy++)
    for (int xx = x0; xx < x1; xx++) owner_[static_cast<size_t>(yy) * kW + xx] = id;
}

bool Gfx::textAt(int x, int y) const {
  if (x < 0 || y < 0 || x >= kW || y >= kH) return false;
  const int32_t id = owner_[static_cast<size_t>(y) * kW + x];
  return id >= 0 && ops_[static_cast<size_t>(id)].kind == 'T';
}

// A line / outline is about to be drawn over this run of pixels: does it cross visible text?
void Gfx::noteLineHitsText(int x, int y, int w, int h) {
  const int x0 = std::max(x, 0), y0 = std::max(y, 0);
  const int x1 = std::min(x + w, static_cast<int>(kW)), y1 = std::min(y + h, static_cast<int>(kH));
  for (int yy = y0; yy < y1; yy++)
    for (int xx = x0; xx < x1; xx++)
      if (textAt(xx, yy)) {
        line_over_text_++;
        char b[160];
        const GfxOp& t = ops_[static_cast<size_t>(owner_[static_cast<size_t>(yy) * kW + xx])];
        snprintf(b, sizeof(b), "line at (%d,%d) crosses text '%s' at (%d,%d)", xx, yy, t.text.c_str(), t.x, t.y);
        last_line_overlap_ = b;
        return;
      }
}

// Drops recorded operations that no longer own a single pixel (completely painted over). The
// visible picture is unchanged; this only bounds memory during long runs.
void Gfx::maybeCompact() {
  if (ops_.size() < 30000) return;
  std::vector<char> used(ops_.size(), 0);
  for (size_t i = 0; i < owner_.size(); i++)
    if (owner_[i] >= 0) used[static_cast<size_t>(owner_[i])] = 1;
  std::vector<int32_t> remap(ops_.size(), -1);
  std::vector<GfxOp> keep;
  keep.reserve(ops_.size() / 4 + 16);
  for (size_t i = 0; i < ops_.size(); i++) {
    if (!used[i]) continue;
    remap[i] = static_cast<int32_t>(keep.size());
    keep.push_back(ops_[i]);
  }
  for (size_t i = 0; i < owner_.size(); i++)
    if (owner_[i] >= 0) owner_[i] = remap[static_cast<size_t>(owner_[i])];
  ops_.swap(keep);
}

void Gfx::screen(uint16_t color) {
  if (!enabled) return;
  flushText();
  ops_.clear();                                    // a full-screen fill hides everything before it
  GfxOp o;
  o.kind = 'S'; o.x = 0; o.y = 0; o.w = kW; o.h = kH; o.color = color; o.size = 0;
  ops_.push_back(o);
  owner_.assign(static_cast<size_t>(kW) * kH, 0);
  pix_.assign(static_cast<size_t>(kW) * kH, color);
}

void Gfx::fillRect(int x, int y, int w, int h, uint16_t color) {
  if (!enabled) return;
  flushText();
  maybeCompact();
  if (w <= 0 || h <= 0) return;
  GfxOp o;
  o.kind = 'F'; o.x = static_cast<int16_t>(x); o.y = static_cast<int16_t>(y); o.w = static_cast<int16_t>(w);
  o.h = static_cast<int16_t>(h); o.color = color; o.size = 0;
  ops_.push_back(o);
  const int id = static_cast<int>(ops_.size()) - 1;
  own(x, y, w, h, id);
  const int x0 = std::max(x, 0), y0 = std::max(y, 0);
  const int x1 = std::min(x + w, static_cast<int>(kW)), y1 = std::min(y + h, static_cast<int>(kH));
  for (int yy = y0; yy < y1; yy++)
    for (int xx = x0; xx < x1; xx++) pix_[static_cast<size_t>(yy) * kW + xx] = color;
}

void Gfx::hline(int x, int y, int w, uint16_t color) {
  if (!enabled) return;
  flushText();
  maybeCompact();
  if (w <= 0) return;
  GfxOp o;
  o.kind = 'H'; o.x = static_cast<int16_t>(x); o.y = static_cast<int16_t>(y); o.w = static_cast<int16_t>(w);
  o.h = 1; o.color = color; o.size = 0;
  noteLineHitsText(x, y, w, 1);
  ops_.push_back(o);
  const int id = static_cast<int>(ops_.size()) - 1;
  own(x, y, w, 1, id);
  if (y >= 0 && y < kH)
    for (int xx = std::max(x, 0); xx < std::min(x + w, static_cast<int>(kW)); xx++) pix_[static_cast<size_t>(y) * kW + xx] = color;
}

void Gfx::vline(int x, int y, int h, uint16_t color) {
  if (!enabled) return;
  flushText();
  maybeCompact();
  if (h <= 0) return;
  GfxOp o;
  o.kind = 'V'; o.x = static_cast<int16_t>(x); o.y = static_cast<int16_t>(y); o.w = 1;
  o.h = static_cast<int16_t>(h); o.color = color; o.size = 0;
  noteLineHitsText(x, y, 1, h);
  ops_.push_back(o);
  const int id = static_cast<int>(ops_.size()) - 1;
  own(x, y, 1, h, id);
  if (x >= 0 && x < kW)
    for (int yy = std::max(y, 0); yy < std::min(y + h, static_cast<int>(kH)); yy++) pix_[static_cast<size_t>(yy) * kW + x] = color;
}

void Gfx::rect(int x, int y, int w, int h, uint16_t color) {
  if (!enabled) return;
  flushText();
  maybeCompact();
  if (w <= 0 || h <= 0) return;
  GfxOp o;
  o.kind = 'R'; o.x = static_cast<int16_t>(x); o.y = static_cast<int16_t>(y); o.w = static_cast<int16_t>(w);
  o.h = static_cast<int16_t>(h); o.color = color; o.size = 0;
  noteLineHitsText(x, y, w, 1);
  noteLineHitsText(x, y + h - 1, w, 1);
  noteLineHitsText(x, y, 1, h);
  noteLineHitsText(x + w - 1, y, 1, h);
  ops_.push_back(o);
  const int id = static_cast<int>(ops_.size()) - 1;
  own(x, y, w, 1, id);
  own(x, y + h - 1, w, 1, id);
  own(x, y, 1, h, id);
  own(x + w - 1, y, 1, h, id);
  auto put = [this, color](int xx, int yy) {
    if (xx >= 0 && xx < kW && yy >= 0 && yy < kH) pix_[static_cast<size_t>(yy) * kW + xx] = color;
  };
  for (int i = 0; i < w; i++) { put(x + i, y); put(x + i, y + h - 1); }
  for (int j = 0; j < h; j++) { put(x, y + j); put(x + w - 1, y + j); }
}

void Gfx::line(int x0, int y0, int x1, int y1, uint16_t color) {
  if (!enabled) return;
  flushText();
  maybeCompact();
  GfxOp o;
  o.kind = 'L'; o.x = static_cast<int16_t>(x0); o.y = static_cast<int16_t>(y0); o.w = static_cast<int16_t>(x1);
  o.h = static_cast<int16_t>(y1); o.color = color; o.size = 0;
  {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
    for (int guard = 0; guard < 2000; guard++) {
      noteLineHitsText(x, y, 1, 1);
      if (x == x1 && y == y1) break;
      const int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }
  ops_.push_back(o);
  const int id = static_cast<int>(ops_.size()) - 1;
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  for (int guard = 0; guard < 2000; guard++) {
    if (x >= 0 && x < kW && y >= 0 && y < kH) {
      owner_[static_cast<size_t>(y) * kW + x] = id;
      pix_[static_cast<size_t>(y) * kW + x] = color;
    }
    if (x == x1 && y == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
}

void Gfx::glyph(int x, int y, uint8_t size, uint16_t color, char c) {
  if (!enabled) return;
  if (x < 0 || y < 0 || x + 5 * size > kW || y + 7 * size > kH) {
    clipped_++;
    char b[96];
    snprintf(b, sizeof(b), "glyph '%c' at (%d,%d) size %d after '%s'", c, x, y, static_cast<int>(size), run_text_.c_str());
    last_clip_ = b;
  }
  if (run_open_ && run_size_ == size && run_color_ == color && run_y_ == y && run_next_x_ == x) {
    run_text_.push_back(c);
    run_next_x_ = x + 6 * size;
    return;
  }
  flushText();
  run_open_ = true;
  run_x_ = x; run_y_ = y; run_size_ = size; run_color_ = color;
  run_next_x_ = x + 6 * size;
  run_text_.assign(1, c);
}

void Gfx::flushText() {
  if (!run_open_) return;
  run_open_ = false;
  if (run_text_.empty()) return;
  GfxOp o;
  o.kind = 'T'; o.x = static_cast<int16_t>(run_x_); o.y = static_cast<int16_t>(run_y_); o.w = 0; o.h = 0;
  o.color = run_color_; o.size = run_size_; o.text = run_text_;
  ops_.push_back(o);
  const int id = static_cast<int>(ops_.size()) - 1;
  bool over = false, over_line = false;
  for (size_t i = 0; i < run_text_.size(); i++) {
    if (run_text_[i] == ' ') continue;
    const int cx = run_x_ + static_cast<int>(i) * 6 * run_size_;
    const int x0 = std::max(cx, 0), y0 = std::max(run_y_, 0);
    const int x1 = std::min(cx + 5 * run_size_, static_cast<int>(kW)), y1 = std::min(run_y_ + 7 * run_size_, static_cast<int>(kH));
    for (int yy = y0; yy < y1 && !over; yy++)
      for (int xx = x0; xx < x1; xx++) {
        const int32_t ow = owner_[static_cast<size_t>(yy) * kW + xx];
        if (ow >= 0 && ow < id) {
          const char k = ops_[static_cast<size_t>(ow)].kind;
          if ((k == 'R' || k == 'H' || k == 'V' || k == 'L') && !over_line) {
            over_line = true;
            char b[200];
            snprintf(b, sizeof(b), "'%s' at (%d,%d) is printed over a line / outline at (%d,%d)", run_text_.c_str(), run_x_, run_y_, xx, yy);
            last_line_overlap_ = b;
          }
        }
        if (ow >= 0 && ow < id && ops_[static_cast<size_t>(ow)].kind == 'T') {
          over = true;
          char b[200];
          snprintf(b, sizeof(b), "'%s' at (%d,%d) over '%s' at (%d,%d)", run_text_.c_str(), run_x_, run_y_,
                   ops_[static_cast<size_t>(ow)].text.c_str(), ops_[static_cast<size_t>(ow)].x, ops_[static_cast<size_t>(ow)].y);
          last_overlap_ = b;
          break;
        }
      }
    own(cx, run_y_, 5 * run_size_, 7 * run_size_, id);
  }
  if (over) text_over_text_++;
  if (over_line) text_over_line_++;
  run_text_.clear();
}

const std::vector<GfxOp>& Gfx::opList() {
  flushText();
  return ops_;
}

float Gfx::visibleFraction(size_t i) const {
  const GfxOp& o = ops_[i];
  if (o.kind != 'T') return 0.0f;
  long total = 0, mine = 0;
  for (size_t k = 0; k < o.text.size(); k++) {
    if (o.text[k] == ' ') continue;
    const int cx = o.x + static_cast<int>(k) * 6 * o.size;
    const int x0 = std::max(cx, 0), y0 = std::max(static_cast<int>(o.y), 0);
    const int x1 = std::min(cx + 5 * o.size, static_cast<int>(kW)), y1 = std::min(o.y + 7 * o.size, static_cast<int>(kH));
    for (int yy = y0; yy < y1; yy++)
      for (int xx = x0; xx < x1; xx++) {
        total++;
        if (owner_[static_cast<size_t>(yy) * kW + xx] == static_cast<int32_t>(i)) mine++;
      }
  }
  return total > 0 ? static_cast<float>(mine) / static_cast<float>(total) : 0.0f;
}

uint32_t Gfx::partialOverdraws() {
  flushText();
  uint32_t n = 0;
  for (size_t i = 0; i < ops_.size(); i++) {
    if (ops_[i].kind != 'T') continue;
    const float f = visibleFraction(i);
    if (f > 0.0f && f < 0.999f) n++;
  }
  return n;
}

std::string Gfx::visibleText() {
  flushText();
  std::vector<size_t> idx;
  for (size_t i = 0; i < ops_.size(); i++)
    if (ops_[i].kind == 'T' && visibleFraction(i) >= 0.5f) idx.push_back(i);
  std::stable_sort(idx.begin(), idx.end(), [this](size_t a, size_t b) {
    if (ops_[a].y != ops_[b].y) return ops_[a].y < ops_[b].y;
    return ops_[a].x < ops_[b].x;
  });
  std::string out;
  for (size_t k = 0; k < idx.size(); k++) {
    out += ops_[idx[k]].text;
    out += '\n';
  }
  return out;
}

std::string Gfx::flatText() {
  const std::string v = visibleText();
  std::string out;
  bool sp = true;
  for (size_t i = 0; i < v.size(); i++) {
    const char c = v[i];
    if (c == '\n' || c == ' ') {
      if (!sp) out += ' ';
      sp = true;
    } else {
      out += c;
      sp = false;
    }
  }
  while (!out.empty() && out.back() == ' ') out.pop_back();
  return out;
}

bool Gfx::contains(const char* needle) { return flatText().find(needle) != std::string::npos; }

uint16_t Gfx::colorAt(int x, int y) {
  if (x < 0 || y < 0 || x >= kW || y >= kH) return 0;
  return pix_[static_cast<size_t>(y) * kW + x];
}

bool Gfx::dump(const std::string& path) {
  flushText();
  FILE* f = fopen(path.c_str(), "w");
  if (!f) return false;
  for (size_t i = 0; i < ops_.size(); i++) {
    const GfxOp& o = ops_[i];
    switch (o.kind) {
      case 'T': {
        std::string esc;
        for (size_t k = 0; k < o.text.size(); k++) {
          const char c = o.text[k];
          if (c == '\\') esc += "\\\\";
          else if (c == '\n') esc += "\\n";
          else esc += c;
        }
        fprintf(f, "T %d %d %u %u %s\n", o.x, o.y, static_cast<unsigned>(o.size), static_cast<unsigned>(o.color), esc.c_str());
        break;
      }
      default:
        fprintf(f, "%c %d %d %d %d %u\n", o.kind, o.x, o.y, o.w, o.h, static_cast<unsigned>(o.color));
    }
  }
  fclose(f);
  return true;
}

Gfx& gfx() {
  static Gfx g;
  static bool init = false;
  if (!init) {
    g.reset();
    init = true;
  }
  return g;
}

// ============================================================================ test-bench control
Mpu& mpu() {
  static Mpu m;
  return m;
}
Fs& fs() {
  static Fs f;
  return f;
}

uint64_t nowUs() { return g_now_us; }
void setNowUs(uint64_t us) {
  g_now_us = us;
  g_bg_acc = 0;
}

void advanceUs(uint64_t us) {
  while (us > 0) {
    uint64_t chunk = us;
    if (g_bg) {
      const uint64_t to_next = g_bg_step - g_bg_acc;
      if (chunk > to_next) chunk = to_next;
    }
    g_now_us += chunk;
    us -= chunk;
    if (g_bg) {
      g_bg_acc += chunk;
      if (g_bg_acc >= g_bg_step) {
        g_bg_acc = 0;
        if (!g_in_bg) {                            // the background "thread" is not re-entrant
          g_in_bg = true;
          g_bg(g_bg_ctx);
          g_in_bg = false;
        }
      }
    }
  }
}

void setBackground(void (*fn)(void*), void* ctx, uint32_t step_us) {
  g_bg = fn;
  g_bg_ctx = ctx;
  g_bg_step = step_us ? step_us : 1000;
  g_bg_acc = 0;
}
void clearBackground() { g_bg = nullptr; }

void setInput(uint8_t pin, int level) {
  if (pin < kPins) g_in[pin] = level ? 1 : 0;
}
int  output(uint8_t pin) { return pin < kPins ? g_out[pin] : 0; }
void setAnalog(uint8_t pin, int value) {
  if (pin < kPins) g_analog[pin] = value;
}
uint32_t buzzerHz() { return g_tone_hz; }
uint32_t toneCount() { return g_tone_count; }
bool     restartRequested() { return g_restart; }
void     clearRestart() { g_restart = false; }
void     clearNvs() { nvs().clear(); }

void softReset() {
  g_now_us = 0;
  g_bg = nullptr;
  g_bg_acc = 0;
  g_in_bg = false;
  for (int i = 0; i < kPins; i++) {
    g_in[i] = 1;
    g_out[i] = 0;
    g_analog[i] = 0;
  }
  g_tone_hz = 0;
  g_tone_count = 0;
  g_restart = false;
  mpuPowerOn();
  g_rng.seed(12345);
  Serial = HardwareSerial();
  Serial2 = HardwareSerial();
  Wire = TwoWire();
  gfx().reset();
}

void reset() {
  g_now_us = 0;
  g_bg = nullptr;
  g_bg_acc = 0;
  g_in_bg = false;
  for (int i = 0; i < kPins; i++) {
    g_in[i] = 1;                                   // pulled up, button released
    g_out[i] = 0;
    g_analog[i] = 0;
  }
  g_tone_hz = 0;
  g_tone_count = 0;
  g_restart = false;
  nvs().clear();
  mpu() = Mpu();
  mpuPowerOn();
  g_rng.seed(12345);
  fs() = Fs();
  Serial = HardwareSerial();
  Serial2 = HardwareSerial();
  Wire = TwoWire();
  gfx().reset();
}

}  // namespace stub

namespace {
// Power-on state without requiring the test-bench to call stub::reset() first.
struct InitOnce {
  InitOnce() { stub::reset(); }
} g_init_once;
}  // namespace
