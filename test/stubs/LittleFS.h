// Host stub of LittleFS.h
#pragma once
#include <FS.h>

namespace fs {
class LittleFSFS : public FS {
 public:
  bool begin(bool formatOnFail = false, const char* basePath = "/littlefs", uint8_t maxOpenFiles = 10,
             const char* partitionLabel = "spiffs");
  void end() {}
};
}  // namespace fs

extern fs::LittleFSFS LittleFS;
