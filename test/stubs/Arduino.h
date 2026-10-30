// Host stub of the Arduino-ESP32 core: just enough of the API for the RubbleSense firmware to
// compile and run on a PC with a virtual clock. NOT a model of the real core's behaviour.
#pragma once
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define DEC 10
#define HEX 16
#define PI 3.14159265358979323846
#define SERIAL_8N1 0x800001c

typedef bool boolean;

// ---- time (virtual clock, see stub_hw.h)
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// ---- GPIO
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int  digitalRead(uint8_t pin);
int  analogRead(uint8_t pin);

// ---- LEDC: the two Arduino-ESP32 API generations (signatures as in the real headers).
// Default = 2.x (channel based; what espressif32@6 / PlatformIO ships). Compile with
// -DSTUB_ARDUINO_V3 -Itest/stubs_v3 to syntax-check the 3.x (pin based) branch.
#if defined(STUB_ARDUINO_V3)
bool     ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
uint32_t ledcWriteTone(uint8_t pin, uint32_t freq);
#else
double ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
void   ledcAttachPin(uint8_t pin, uint8_t channel);
double ledcWriteTone(uint8_t channel, double freq);
#endif

// ---- Print / Stream
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* buf, size_t n) {
    size_t k = 0;
    while (k < n) k += write(buf[k]);
    return k;
  }
  size_t write(const char* s) { return s ? write(reinterpret_cast<const uint8_t*>(s), strlen(s)) : 0; }
  size_t write(const char* s, size_t n) { return write(reinterpret_cast<const uint8_t*>(s), n); }

  size_t print(const char* s) { return write(s); }
  size_t print(char c) { return write(static_cast<uint8_t>(c)); }
  size_t print(unsigned char v, int base = DEC) { return printNumber(v, base); }
  size_t print(int v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned int v, int base = DEC) { return printNumber(v, base); }
  size_t print(long v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned long v, int base = DEC) { return printNumber(v, base); }
  size_t print(double v, int digits = 2) {
    char b[48];
    snprintf(b, sizeof(b), "%.*f", digits, v);
    return write(b);
  }
  size_t println() { return write(reinterpret_cast<const uint8_t*>("\r\n"), 2); }
  size_t println(const char* s) { return print(s) + println(); }
  size_t println(char c) { return print(c) + println(); }
  size_t println(unsigned char v, int base = DEC) { return print(v, base) + println(); }
  size_t println(int v, int base = DEC) { return print(v, base) + println(); }
  size_t println(unsigned int v, int base = DEC) { return print(v, base) + println(); }
  size_t println(long v, int base = DEC) { return print(v, base) + println(); }
  size_t println(unsigned long v, int base = DEC) { return print(v, base) + println(); }
  size_t println(double v, int digits = 2) { return print(v, digits) + println(); }

  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    return write(b);
  }

 private:
  size_t printNumber(unsigned long v, int base) {
    char b[40];
    snprintf(b, sizeof(b), base == HEX ? "%lX" : "%lu", v);
    return write(b);
  }
  size_t printSigned(long v, int base) {
    if (base == DEC) {
      char b[40];
      snprintf(b, sizeof(b), "%ld", v);
      return write(b);
    }
    return printNumber(static_cast<unsigned long>(v), base);
  }
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() {}
};

class HardwareSerial : public Stream {
 public:
  void   setRxBufferSize(size_t n) { rx_cap_ = n; }
  void   begin(unsigned long, uint32_t = SERIAL_8N1, int8_t = -1, int8_t = -1) { begun_ = true; }
  int    available() override;
  int    read() override;
  int    peek() override;
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* b, size_t n) override;
  using Print::write;

  // test-bench side
  void               injectRx(const uint8_t* b, size_t n);
  const std::string& tx() const { return tx_; }
  void               clearTx() { tx_.clear(); }
  bool               begun() const { return begun_; }
  uint32_t           rxDropped() const { return rx_dropped_; }   // bytes the driver ring had no room for

 private:
  std::string rx_;
  size_t      rx_pos_ = 0;
  size_t      rx_cap_ = 256;            // the Arduino-ESP32 default; the firmware enlarges it before begin()
  uint32_t    rx_dropped_ = 0;
  std::string tx_;
  bool        begun_ = false;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial2;

class EspClass {
 public:
  uint32_t getFreeHeap() const { return 180u * 1024u; }
  void     restart();
};
extern EspClass ESP;
