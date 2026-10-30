// RubbleSense firmware - serial console (USB, 115200 baud).
//
// Line oriented text: one command per line, answered with "# ..." comment lines followed by a
// final "OK" or "ERR <reason>" line, so a script can drive it. Scan results are printed as
// one-line JSON objects (they start with '{'). The same UART can carry serial I/Q frames
// (RS_IQ_ON_UART0); a line that contains any binary byte is discarded, never executed.
#pragma once
#include <stdint.h>
#include "settings.h"

class Print;

class ConsoleHost {
 public:
  virtual ~ConsoleHost() {}
  virtual void conStatus(Print& out) = 0;
  virtual bool conScan(Print& out) = 0;
  virtual void conAbort(Print& out) = 0;
  virtual bool conCalibrate(Print& out) = 0;
  virtual void conClearBaseline(Print& out) = 0;
  virtual bool conSource(Print& out, SourceKind k) = 0;
  virtual bool conPreset(Print& out, int idx) = 0;
  virtual void conLang(Print& out, uint8_t lang) = 0;
  virtual void conSound(Print& out, bool on) = 0;
  virtual void conLogDump(Print& out) = 0;
  virtual bool conLogClear(Print& out) = 0;
  virtual void conParams(Print& out) = 0;
  virtual bool conGet(Print& out, const char* name) = 0;
  virtual bool conSet(Print& out, const char* name, const char* value) = 0;
  virtual void conSelfTest(Print& out) = 0;
  virtual void conFactoryReset(Print& out) = 0;
  virtual void conReboot() = 0;
};

class Console {
 public:
  Console(ConsoleHost& host, Print& out) : host_(host), out_(out) {}

  void feed(uint8_t b);              // one received byte
  void exec(const char* line);       // one complete line (also used by the tests)

  // While serial I/Q frames share the console UART, a fragment of the binary stream can by chance
  // consist of printable bytes only. Such a line must not be answered with "ERR unknown command"
  // (a script reading the console would see noise): unknown commands are dropped silently then.
  // Known commands with bad arguments are still answered.
  void setQuietUnknown(bool on) { quiet_unknown_ = on; }

 private:
  void help();

  ConsoleHost& host_;
  Print&       out_;
  char     line_[96];
  uint8_t  len_ = 0;
  bool     discard_ = false;         // overflow or binary content on this line
  bool     quiet_unknown_ = false;
};
