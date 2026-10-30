// Host stub: SPI is only needed so that the display library's header chain resolves.
#pragma once
#include <Arduino.h>

class SPIClass {
 public:
  void begin(int8_t = -1, int8_t = -1, int8_t = -1, int8_t = -1) {}
  void end() {}
};
extern SPIClass SPI;
