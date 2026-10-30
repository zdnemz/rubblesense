// RubbleSense firmware - LEDs and buzzer (non-blocking tone sequencer).
//
// The buzzer can be muted (rescue teams often need silence on site to listen); the LEDs always
// work. Green is ONLY ever lit for a candidate pattern - a negative scan never lights it.
#pragma once
#include <stdint.h>

enum class Beep : uint8_t {
  kNone = 0, kKey, kStart, kCandidate, kPossible, kConfounded, kLowSens, kInvalid, kNegative, kDone, kError
};

enum class ScanLed : uint8_t { kOff = 0, kOn, kBlink };

class Alerts {
 public:
  void begin();
  void update(uint32_t now_ms);
  void setSound(bool on);
  bool sound() const { return sound_; }

  void leds(bool candidate, bool uncertain);
  void scanLed(ScanLed mode) { scan_mode_ = mode; }
  void play(Beep b);

  // for the test-bench: the tone currently sounding (Hz, 0 = silent)
  uint16_t currentTone() const { return cur_hz_; }

  struct Step { uint16_t hz; uint16_t ms; };   // one note of a pattern (hz 0 = rest)

 private:
  void tone(uint16_t hz);
  void startSeq(const Step* s, uint8_t n);

  bool     sound_ = true;
  ScanLed  scan_mode_ = ScanLed::kOff;
  bool     scan_state_ = false;
  uint32_t scan_t_ = 0;
  const Step* seq_ = nullptr;
  uint8_t  n_ = 0, i_ = 0;
  uint32_t t_step_ = 0;
  bool     active_ = false;
  uint16_t cur_hz_ = 0;
};
