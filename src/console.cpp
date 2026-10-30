#include "console.h"
#include <Arduino.h>
#include "radar_sim.h"
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace {
bool eq(const char* a, const char* b) { return a && b && strcasecmp(a, b) == 0; }

// "0".."6", "manual", "a".."f"  ->  preset index, or -1.
int parsePreset(const char* s) {
  if (!s || !s[0]) return -1;
  if (eq(s, "manual")) return 0;
  if (s[1] == '\0') {
    if (s[0] >= '0' && s[0] <= '9') return (s[0] - '0' < static_cast<int>(kNumPresets)) ? s[0] - '0' : -1;
    const char c = static_cast<char>(s[0] | 0x20);
    if (c >= 'a' && c <= 'f') return 1 + (c - 'a');
  }
  return -1;
}
}  // namespace

void Console::feed(uint8_t b) {
  if (b == '\n' || b == '\r') {
    if (len_ > 0 && !discard_) {
      line_[len_] = '\0';
      exec(line_);
    }
    len_ = 0;
    discard_ = false;
    return;
  }
  if (b == '\t') b = ' ';
  if (b < 32 || b > 126) { discard_ = true; return; }        // binary: never part of a command
  if (len_ >= sizeof(line_) - 1) { discard_ = true; return; }
  line_[len_++] = static_cast<char>(b);
}

void Console::help() {
  out_.println("# RubbleSense console - commands:");
  out_.println("#   status                 state, source, IMU, baseline, memory");
  out_.println("#   scan | abort           start / abort a scan");
  out_.println("#   calib | calib clear    measure / clear the noise-floor baseline of the active source");
  out_.println("#   src sim|serial         select the radar source");
  out_.println("#   preset 0..6|manual|a..f  simulated trial preset");
  out_.println("#   lang en|id             operator language");
  out_.println("#   sound on|off           buzzer");
  out_.println("#   get [name] | set <name> <value>   tunable DSP parameters");
  out_.println("#   log | log clear        dump / clear the scan log (JSON lines)");
  out_.println("#   selftest | reboot | factory confirm");
  out_.println("# SAFETY: a negative scan never means 'no survivor'.");
}

void Console::exec(const char* line) {
  char buf[96];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* tok[4] = {nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  char* p = buf;
  while (*p && n < 4) {
    while (*p == ' ') p++;
    if (!*p) break;
    tok[n++] = p;
    while (*p && *p != ' ') p++;
    if (*p) *p++ = '\0';
  }
  if (n == 0) return;
  const char* cmd = tok[0];
  const char* a1 = tok[1];
  const char* a2 = tok[2];

  bool ok = true;
  const char* why = "bad arguments (try: help)";

  if (eq(cmd, "help") || eq(cmd, "?")) {
    help();
  } else if (eq(cmd, "status")) {
    host_.conStatus(out_);
  } else if (eq(cmd, "scan")) {
    ok = host_.conScan(out_);
    why = "cannot start a scan now";
  } else if (eq(cmd, "abort")) {
    host_.conAbort(out_);
  } else if (eq(cmd, "calib")) {
    if (!a1) { ok = host_.conCalibrate(out_); why = "cannot calibrate now"; }
    else if (eq(a1, "clear")) host_.conClearBaseline(out_);
    else ok = false;
  } else if (eq(cmd, "src")) {
    if (eq(a1, "sim")) { ok = host_.conSource(out_, SourceKind::kSim); why = "cannot change the source now"; }
    else if (eq(a1, "serial")) { ok = host_.conSource(out_, SourceKind::kSerial); why = "cannot change the source now"; }
    else ok = false;
  } else if (eq(cmd, "preset")) {
    const int idx = parsePreset(a1);
    if (idx < 0) ok = false;
    else { ok = host_.conPreset(out_, idx); why = "cannot change the preset now"; }
  } else if (eq(cmd, "lang")) {
    if (eq(a1, "en")) host_.conLang(out_, 0);
    else if (eq(a1, "id")) host_.conLang(out_, 1);
    else ok = false;
  } else if (eq(cmd, "sound")) {
    if (eq(a1, "on")) host_.conSound(out_, true);
    else if (eq(a1, "off")) host_.conSound(out_, false);
    else ok = false;
  } else if (eq(cmd, "get")) {
    if (!a1) host_.conParams(out_);
    else { ok = host_.conGet(out_, a1); why = "unknown parameter"; }
  } else if (eq(cmd, "set")) {
    if (!a1 || !a2) ok = false;
    else { ok = host_.conSet(out_, a1, a2); why = "rejected (see message above)"; }
  } else if (eq(cmd, "log")) {
    if (!a1) host_.conLogDump(out_);
    else if (eq(a1, "clear")) { ok = host_.conLogClear(out_); why = "cannot clear the log"; }
    else ok = false;
  } else if (eq(cmd, "selftest")) {
    host_.conSelfTest(out_);
  } else if (eq(cmd, "factory")) {
    if (eq(a1, "confirm")) host_.conFactoryReset(out_);
    else { ok = false; why = "type: factory confirm"; }
  } else if (eq(cmd, "reboot")) {
    out_.println("OK");
    host_.conReboot();
    return;
  } else {
    if (quiet_unknown_) return;
    ok = false;
    why = "unknown command (try: help)";
  }

  if (ok) {
    out_.println("OK");
  } else {
    out_.print("ERR ");
    out_.println(why);
  }
}
