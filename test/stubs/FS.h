// Host stub of the Arduino-ESP32 FS/File API, backed by the in-memory map in stub_hw.h.
#pragma once
#include <Arduino.h>
#include <string>

namespace fs {

class File : public Stream {
 public:
  File() {}
  File(const std::string& path, bool readable, bool writable) : path_(path), open_(true), rd_(readable), wr_(writable) {}

  operator bool() const { return open_; }
  size_t size() const;
  size_t position() const { return pos_; }
  void   close() { open_ = false; }
  const char* path() const { return path_.c_str(); }

  size_t read(uint8_t* buf, size_t n);
  int    read() override;
  int    peek() override;
  int    available() override;
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t n) override;
  using Print::write;

 private:
  std::string path_;
  bool        open_ = false, rd_ = false, wr_ = false;
  size_t      pos_ = 0;
};

class FS {
 public:
  File open(const char* path, const char* mode = "r", const bool create = false);
  bool exists(const char* path);
  bool remove(const char* path);
  bool rename(const char* from, const char* to);
};

}  // namespace fs

using fs::File;
using fs::FS;
