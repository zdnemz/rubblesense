// RubbleSense firmware - scan log on LittleFS (one JSON object per line).
//
// Flash writes stall the whole chip for milliseconds: append() must only be called while no
// scan is running. The file rotates (scans.jsonl -> scans.old.jsonl) at RS_LOG_MAX_BYTES, so
// the newest data is always kept and the filesystem can never fill up.
#pragma once
#include <stddef.h>
#include <stdint.h>

class Print;

class Storage {
 public:
  bool begin();                         // mount, formatting on first use
  bool ok() const { return ok_; }

  bool append(const char* line);        // adds the newline; rotates when full
  uint32_t lines() const { return cur_lines_ + old_lines_; }
  size_t   size() const;
  void     dump(Print& out) const;      // old file first, then the current one
  bool     clear();

 private:
  bool ok_ = false;
  uint32_t cur_lines_ = 0;              // lines in scans.jsonl
  uint32_t old_lines_ = 0;              // lines in scans.old.jsonl (the previous generation)
};
