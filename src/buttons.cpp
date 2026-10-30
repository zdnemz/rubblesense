#include "buttons.h"
#include <Arduino.h>
#include "rs_pins.h"
#include "rs_sync.h"

constexpr uint8_t  Buttons::kCount;
constexpr uint16_t Buttons::kDebounceMs;
constexpr uint16_t Buttons::kLongMs;

namespace {
const uint8_t kPins[Buttons::kCount] = {PIN_BTN_SCAN, PIN_BTN_MODE, PIN_BTN_MENU};
}

void Buttons::begin() {
  for (uint8_t i = 0; i < kCount; i++) pinMode(kPins[i], INPUT_PULLUP);
  qh_ = qt_ = 0;
}

void Buttons::push(BtnId id, BtnEvt e) {
  const uint8_t next = static_cast<uint8_t>((qh_ + 1) % 8);
  if (next == qt_) return;               // queue full: drop (the operator is mashing buttons)
  q_[qh_].id = id;
  q_[qh_].evt = e;
  qh_ = next;
}

bool Buttons::pop(BtnEvent& e) {
  if (qt_ == qh_) return false;
  e = q_[qt_];
  qt_ = static_cast<uint8_t>((qt_ + 1) % 8);
  return true;
}

void Buttons::update(uint32_t now_ms) {
  for (uint8_t i = 0; i < kCount; i++) {
    One& b = st_[i];
    const bool raw = digitalRead(kPins[i]) == LOW;
    if (raw != b.raw) {
      b.raw = raw;
      b.t_raw = now_ms;
    }
    if (raw != b.down && static_cast<uint32_t>(now_ms - b.t_raw) >= kDebounceMs) {   // stable for the debounce time
      b.down = raw;
      if (raw) {
        b.t_down = now_ms;
        b.long_sent = false;
      } else if (!b.long_sent) {
        push(static_cast<BtnId>(i), BtnEvt::kShort);          // released before the long threshold
      }
    }
    if (b.down && !b.long_sent && static_cast<uint32_t>(now_ms - b.t_down) >= kLongMs) {
      b.long_sent = true;
      push(static_cast<BtnId>(i), BtnEvt::kLong);              // fires while still held
    }
  }
}
