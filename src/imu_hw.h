// RubbleSense firmware - MPU6050 driver (raw I2C registers, no library dependency).
//
// The IMU does one job here: tell the DSP whether the sensor was held still. It is read at
// 100 Hz and reduced by rs::ImuProcessor. It cannot see sub-millimetre micro-vibration at
// breathing frequencies - that case is handled in the radar domain (common-phase estimate).
#pragma once
#include <stdint.h>

class TwoWire;

struct ImuSample {
  float ax, ay, az;   // m/s^2
  float gx, gy, gz;   // rad/s
};

class ImuHw {
 public:
  // Probe 0x68 then 0x69, reset, configure (+-4 g, +-500 deg/s, 21 Hz DLPF, 100 Hz output).
  // Returns false if no sensor answers or the configuration does not read back.
  bool begin(TwoWire& wire);

  bool     present() const { return present_; }
  uint8_t  whoAmI() const { return who_; }
  uint8_t  address() const { return addr_; }
  uint32_t errors() const { return errors_; }

  // One 14-byte burst read. False on a bus error, or when the sensor returns the very same raw
  // block for ~0.6 s (frozen / stuck output; only checked if RS_IMU_FROZEN_CHECK).
  bool read(ImuSample& s);

  // Self-test helper: |a| in m/s^2 of a fresh sample, or a negative number on failure.
  float gravityMagnitude();

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t& val);

  TwoWire* wire_ = nullptr;
  bool     present_ = false;
  uint8_t  addr_ = 0x68;
  uint8_t  who_ = 0;
  uint32_t errors_ = 0;
  uint8_t  last_[14] = {0};
  bool     have_last_ = false;
  uint16_t same_ = 0;
};
