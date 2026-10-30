// Host stub of Adafruit_ILI9341 (240x320 panel, hardware-SPI constructor).
#pragma once
#include "Adafruit_GFX.h"

#define ILI9341_TFTWIDTH 240
#define ILI9341_TFTHEIGHT 320

class Adafruit_ILI9341 : public Adafruit_GFX {
 public:
  Adafruit_ILI9341(int8_t cs, int8_t dc, int8_t rst = -1) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT), cs_(cs), dc_(dc), rst_(rst) {}
  void begin(uint32_t freq = 0);

 private:
  int8_t cs_, dc_, rst_;
};
