#include "alerts.h"
#include <Arduino.h>
#include "rs_pins.h"
#include "rs_sync.h"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

namespace {
constexpr uint8_t  kLedcChannel = 4;      // Arduino-ESP32 2.x channel (unused by the pin-based 3.x API)
constexpr uint32_t kLedcBaseHz = 2000;
constexpr uint8_t  kLedcBits = 10;

typedef Alerts::Step Step;

const Step kKey[]  = {{1200, 40}};
const Step kStart[] = {{1800, 80}};
const Step kCand[] = {{1500, 120}, {0, 50}, {1900, 120}, {0, 50}, {2400, 220}};
const Step kPoss[] = {{1000, 150}, {0, 100}, {1000, 150}};
const Step kConf[] = {{700, 150}, {0, 80}, {700, 150}, {0, 80}, {700, 150}};
const Step kLow[]  = {{500, 400}, {0, 100}, {400, 400}};
const Step kInv[]  = {{600, 300}};
const Step kNeg[]  = {{900, 100}};
const Step kDone[] = {{1500, 60}, {0, 40}, {1500, 60}};
const Step kErr[]  = {{300, 500}};

template <size_t N>
constexpr uint8_t count(const Step (&)[N]) { return static_cast<uint8_t>(N); }
}  // namespace

void Alerts::tone(uint16_t hz) {
  cur_hz_ = sound_ ? hz : 0;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(PIN_BUZZER, cur_hz_);
#else
  ledcWriteTone(kLedcChannel, cur_hz_);
#endif
}

void Alerts::begin() {
  pinMode(PIN_LED_CAND, OUTPUT);
  pinMode(PIN_LED_UNCERT, OUTPUT);
  pinMode(PIN_LED_SCAN, OUTPUT);
  digitalWrite(PIN_LED_CAND, LOW);
  digitalWrite(PIN_LED_UNCERT, LOW);
  digitalWrite(PIN_LED_SCAN, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_BUZZER, kLedcBaseHz, kLedcBits);
#else
  ledcSetup(kLedcChannel, kLedcBaseHz, kLedcBits);
  ledcAttachPin(PIN_BUZZER, kLedcChannel);
#endif
  tone(0);
}

void Alerts::setSound(bool on) {
  sound_ = on;
  if (!on) tone(0);
}

void Alerts::leds(bool candidate, bool uncertain) {
  digitalWrite(PIN_LED_CAND, candidate ? HIGH : LOW);
  digitalWrite(PIN_LED_UNCERT, uncertain ? HIGH : LOW);
}

void Alerts::startSeq(const Alerts::Step* s, uint8_t n) {
  seq_ = s;
  n_ = n;
  i_ = 0;
  active_ = true;
  t_step_ = 0;                           // set on the first update()
  cur_hz_ = 0;
}

void Alerts::play(Beep b) {
  switch (b) {
    case Beep::kNone:       active_ = false; tone(0); return;
    case Beep::kKey:        startSeq(kKey, count(kKey)); return;
    case Beep::kStart:      startSeq(kStart, count(kStart)); return;
    case Beep::kCandidate:  startSeq(kCand, count(kCand)); return;
    case Beep::kPossible:   startSeq(kPoss, count(kPoss)); return;
    case Beep::kConfounded: startSeq(kConf, count(kConf)); return;
    case Beep::kLowSens:    startSeq(kLow, count(kLow)); return;
    case Beep::kInvalid:    startSeq(kInv, count(kInv)); return;
    case Beep::kNegative:   startSeq(kNeg, count(kNeg)); return;
    case Beep::kDone:       startSeq(kDone, count(kDone)); return;
    case Beep::kError:      startSeq(kErr, count(kErr)); return;
  }
}

void Alerts::update(uint32_t now_ms) {
  // blue LED: steady while scanning, 1 Hz blink when ready
  bool want = false;
  switch (scan_mode_) {
    case ScanLed::kOff:   want = false; break;
    case ScanLed::kOn:    want = true; break;
    case ScanLed::kBlink:
      if (static_cast<uint32_t>(now_ms - scan_t_) >= 500) { scan_t_ = now_ms; scan_state_ = !scan_state_; }
      want = scan_state_;
      break;
  }
  digitalWrite(PIN_LED_SCAN, want ? HIGH : LOW);

  if (!active_) return;
  if (t_step_ == 0) {                    // first call after startSeq
    t_step_ = now_ms ? now_ms : 1;
    tone(seq_[0].hz);
    return;
  }
  if (static_cast<uint32_t>(now_ms - t_step_) < seq_[i_].ms) return;
  i_++;
  if (i_ >= n_) {
    active_ = false;
    tone(0);
    return;
  }
  t_step_ = now_ms ? now_ms : 1;
  tone(seq_[i_].hz);
}
