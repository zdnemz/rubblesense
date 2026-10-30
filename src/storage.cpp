#include "storage.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <string.h>
#include "rs_build.h"

namespace {
const char* kCur = "/scans.jsonl";
const char* kOld = "/scans.old.jsonl";

size_t fileSize(const char* path) {
  if (!LittleFS.exists(path)) return 0;
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  const size_t n = f.size();
  f.close();
  return n;
}

uint32_t countLines(const char* path) {
  if (!LittleFS.exists(path)) return 0;
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  uint32_t n = 0;
  uint8_t chunk[128];
  for (;;) {
    const size_t got = f.read(chunk, sizeof(chunk));
    if (got == 0) break;
    for (size_t i = 0; i < got; i++) if (chunk[i] == '\n') n++;
  }
  f.close();
  return n;
}

void dumpFile(const char* path, Print& out) {
  if (!LittleFS.exists(path)) return;
  File f = LittleFS.open(path, "r");
  if (!f) return;
  uint8_t chunk[128];
  for (;;) {
    const size_t got = f.read(chunk, sizeof(chunk));
    if (got == 0) break;
    out.write(chunk, got);
  }
  f.close();
}
}  // namespace

bool Storage::begin() {
  ok_ = LittleFS.begin(true);             // true: format an empty / corrupt partition once
  cur_lines_ = ok_ ? countLines(kCur) : 0;
  old_lines_ = ok_ ? countLines(kOld) : 0;
  return ok_;
}

size_t Storage::size() const { return ok_ ? fileSize(kCur) + fileSize(kOld) : 0; }

bool Storage::append(const char* line) {
  if (!ok_ || !line) return false;
  const size_t len = strlen(line);
  if (fileSize(kCur) + len + 1 > static_cast<size_t>(RS_LOG_MAX_BYTES)) {
    if (LittleFS.exists(kOld)) LittleFS.remove(kOld);
    if (LittleFS.rename(kCur, kOld)) {
      old_lines_ = cur_lines_;            // the current generation became the previous one
    } else {
      LittleFS.remove(kCur);
      old_lines_ = 0;
    }
    cur_lines_ = 0;
  }
  File f = LittleFS.open(kCur, "a");
  if (!f) return false;
  const size_t w1 = f.write(reinterpret_cast<const uint8_t*>(line), len);
  const size_t w2 = f.write(reinterpret_cast<const uint8_t*>("\n"), 1);
  f.close();
  if (w1 != len || w2 != 1) return false;
  cur_lines_++;
  return true;
}

void Storage::dump(Print& out) const {
  if (!ok_) return;
  dumpFile(kOld, out);
  dumpFile(kCur, out);
}

bool Storage::clear() {
  if (!ok_) return false;
  if (LittleFS.exists(kCur)) LittleFS.remove(kCur);
  if (LittleFS.exists(kOld)) LittleFS.remove(kOld);
  cur_lines_ = old_lines_ = 0;
  return true;
}
