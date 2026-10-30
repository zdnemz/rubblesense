// Host stub of Adafruit_GFX: records drawing into stub::gfx() (a display list plus a pixel
// ownership map) instead of talking to a panel. Classic 5x7 font metrics: 6*size px advance,
// 8*size px line height.
#pragma once
#include <Arduino.h>
#include "stub_hw.h"

class Adafruit_GFX : public Print {
 public:
  Adafruit_GFX(int16_t w, int16_t h) : width_(w), height_(h), raw_w_(w), raw_h_(h) {}
  virtual ~Adafruit_GFX() {}

  virtual void fillScreen(uint16_t color);
  virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  virtual void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void         drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

  void setCursor(int16_t x, int16_t y) { cx_ = x; cy_ = y; }
  void setTextColor(uint16_t c) { tc_ = c; }
  void setTextColor(uint16_t c, uint16_t) { tc_ = c; }
  void setTextSize(uint8_t s) { ts_ = (s > 0) ? s : 1; }
  void setTextWrap(bool w) { wrap_ = w; }
  void setRotation(uint8_t r);

  int16_t width() const { return width_; }
  int16_t height() const { return height_; }
  int16_t getCursorX() const { return cx_; }
  int16_t getCursorY() const { return cy_; }

  size_t write(uint8_t c) override;
  using Print::write;

 protected:
  int16_t  width_, height_;
  int16_t  raw_w_, raw_h_;
  int16_t  cx_ = 0, cy_ = 0;
  uint16_t tc_ = 0xFFFF;
  uint8_t  ts_ = 1;
  bool     wrap_ = true;
};
