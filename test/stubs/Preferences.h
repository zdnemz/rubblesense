// Host stub of the ESP32 NVS wrapper. The data lives in a process-wide map, so it survives
// destroying and re-creating the firmware's App object (= a reboot in the test-bench).
#pragma once
#include <Arduino.h>

class Preferences {
 public:
  bool begin(const char* name, bool readOnly = false, const char* partition_label = nullptr);
  void end();
  bool clear();

  uint8_t  getUChar(const char* key, uint8_t defaultValue = 0);
  uint32_t getUInt(const char* key, uint32_t defaultValue = 0);
  float    getFloat(const char* key, float defaultValue = NAN);

  size_t putUChar(const char* key, uint8_t value);
  size_t putUInt(const char* key, uint32_t value);
  size_t putFloat(const char* key, float value);

 private:
  std::string ns_;
  bool        open_ = false;
  bool        ro_ = false;
};
