#include "imu_hw.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "rs_build.h"

namespace {
constexpr uint8_t kRegSmplrtDiv  = 0x19;
constexpr uint8_t kRegConfig     = 0x1A;
constexpr uint8_t kRegGyroCfg    = 0x1B;
constexpr uint8_t kRegAccelCfg   = 0x1C;
constexpr uint8_t kRegAccelXout  = 0x3B;
constexpr uint8_t kRegPwrMgmt1   = 0x6B;
constexpr uint8_t kRegPwrMgmt2   = 0x6C;
constexpr uint8_t kRegWhoAmI     = 0x75;

constexpr uint8_t kCfgDlpf21Hz   = 0x04;   // 21 Hz accel / 20 Hz gyro bandwidth, 1 kHz internal rate
constexpr uint8_t kCfgGyro500    = 0x08;   // +-500 deg/s  -> 65.5 LSB per deg/s
constexpr uint8_t kCfgAccel4g    = 0x08;   // +-4 g        -> 8192 LSB per g
constexpr uint8_t kSmplrtDiv100  = 9;      // 1 kHz / (1 + 9) = 100 Hz

constexpr float kAccelScale = 9.80665f / 8192.0f;
constexpr float kGyroScale  = 0.017453293f / 65.5f;
constexpr uint16_t kFrozenLimit = 60;      // identical raw blocks in a row = 0.6 s at 100 Hz

inline int16_t be16(const uint8_t* p) { return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]); }
}  // namespace

bool ImuHw::writeReg(uint8_t reg, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(val);
  return wire_->endTransmission() == 0;
}

bool ImuHw::readReg(uint8_t reg, uint8_t& val) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, static_cast<uint8_t>(1)) != 1) return false;
  val = static_cast<uint8_t>(wire_->read());
  return true;
}

bool ImuHw::begin(TwoWire& wire) {
  wire_ = &wire;
  present_ = false;
  have_last_ = false;
  same_ = 0;
  static const uint8_t kAddrs[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    addr_ = kAddrs[i];
    wire_->beginTransmission(addr_);
    if (wire_->endTransmission() != 0) continue;
    uint8_t who = 0;
    if (!readReg(kRegWhoAmI, who) || who == 0x00 || who == 0xFF) continue;   // something else on the bus
    who_ = who;

    if (!writeReg(kRegPwrMgmt1, 0x80)) continue;            // device reset
    delay(100);
    if (!writeReg(kRegPwrMgmt1, 0x01)) continue;            // wake, clock = gyro X PLL
    delay(10);
    bool ok = writeReg(kRegPwrMgmt2, 0x00);
    ok = ok && writeReg(kRegSmplrtDiv, kSmplrtDiv100);
    ok = ok && writeReg(kRegConfig, kCfgDlpf21Hz);
    ok = ok && writeReg(kRegGyroCfg, kCfgGyro500);
    ok = ok && writeReg(kRegAccelCfg, kCfgAccel4g);
    uint8_t g = 0, a = 0;
    ok = ok && readReg(kRegGyroCfg, g) && readReg(kRegAccelCfg, a);
    if (!ok || g != kCfgGyro500 || a != kCfgAccel4g) continue;   // configuration did not stick
    present_ = true;
    return true;
  }
  return false;
}

bool ImuHw::read(ImuSample& s) {
  if (!present_) return false;
  wire_->beginTransmission(addr_);
  wire_->write(kRegAccelXout);
  if (wire_->endTransmission(false) != 0) { errors_++; return false; }
  if (wire_->requestFrom(addr_, static_cast<uint8_t>(14)) != 14) { errors_++; return false; }
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = static_cast<uint8_t>(wire_->read());

#if RS_IMU_FROZEN_CHECK
  if (have_last_ && memcmp(b, last_, sizeof(b)) == 0) {
    if (same_ < 0xFFFF) same_++;
  } else {
    same_ = 0;
  }
  memcpy(last_, b, sizeof(b));
  have_last_ = true;
  if (same_ >= kFrozenLimit) { errors_++; return false; }
#endif

  s.ax = static_cast<float>(be16(b + 0)) * kAccelScale;
  s.ay = static_cast<float>(be16(b + 2)) * kAccelScale;
  s.az = static_cast<float>(be16(b + 4)) * kAccelScale;
  // b[6..7] = temperature, unused
  s.gx = static_cast<float>(be16(b + 8)) * kGyroScale;
  s.gy = static_cast<float>(be16(b + 10)) * kGyroScale;
  s.gz = static_cast<float>(be16(b + 12)) * kGyroScale;
  return true;
}

float ImuHw::gravityMagnitude() {
  ImuSample s;
  if (!read(s)) return -1.0f;
  return sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
}
