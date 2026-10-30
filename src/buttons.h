// RubbleSense firmware - debounced push buttons (active low, internal pull-ups).
#pragma once
#include <stdint.h>

enum class BtnId : uint8_t { kScan = 0, kMode = 1, kMenu = 2 };
enum class BtnEvt : uint8_t { kShort = 0, kLong = 1 };

struct BtnEvent {
  BtnId  id;
  BtnEvt evt;
};

class Buttons {
 public:
  static constexpr uint8_t  kCount = 3;
  static constexpr uint16_t kDebounceMs = 30;
  static constexpr uint16_t kLongMs = 900;

  void begin();
  void update(uint32_t now_ms);          // poll at >= 100 Hz
  bool pop(BtnEvent& e);                 // oldest pending event
  bool held(BtnId id) const { return st_[static_cast<uint8_t>(id)].down; }

 private:
  struct One {
    bool     raw = false;                // last raw reading (true = pressed)
    bool     down = false;               // debounced state
    uint32_t t_raw = 0;                  // when the raw reading last changed
    uint32_t t_down = 0;                 // when the debounced press began
    bool     long_sent = false;
  };
  void push(BtnId id, BtnEvt e);

  One      st_[kCount];
  BtnEvent q_[8];
  uint8_t  qh_ = 0, qt_ = 0;
};
