// Host stub of the Arduino-ESP32 I2C master (TwoWire) backed by an MPU-6050 register model.
// Overload set copied from the real core so that ambiguous calls fail here too.
#pragma once
#include <Arduino.h>
#include <vector>

class TwoWire : public Stream {
 public:
  bool begin(int sda = -1, int scl = -1, uint32_t frequency = 0);
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t address);
  uint8_t endTransmission(bool sendStop);
  uint8_t endTransmission(void);

  size_t  requestFrom(uint16_t address, size_t size, bool sendStop);
  uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop);
  uint8_t requestFrom(uint16_t address, uint8_t size, uint8_t sendStop);
  size_t  requestFrom(uint8_t address, size_t len, bool stopBit);
  uint8_t requestFrom(uint16_t address, uint8_t size);
  uint8_t requestFrom(uint8_t address, uint8_t size, uint8_t sendStop);
  uint8_t requestFrom(uint8_t address, uint8_t size);
  uint8_t requestFrom(int address, int size, int sendStop);
  uint8_t requestFrom(int address, int size);

  size_t write(uint8_t b) override;
  size_t write(const uint8_t* b, size_t n) override;
  using Print::write;
  int available(void) override;
  int read(void) override;
  int peek(void) override;

  bool began() const { return began_; }
  int  sda() const { return sda_; }
  int  scl() const { return scl_; }

 private:
  size_t doRequest(uint8_t address, size_t len);

  bool                 began_ = false;
  int                  sda_ = -1, scl_ = -1;
  uint8_t              addr_ = 0;
  std::vector<uint8_t> tx_;
  std::vector<uint8_t> rx_;
  size_t               rx_pos_ = 0;
  uint8_t              reg_ptr_ = 0;
};

extern TwoWire Wire;
