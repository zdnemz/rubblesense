// RubbleSense firmware - the single reader of the physical UARTs.
//
// Runs in the acquisition thread so the serial I/Q stream is drained at >= 1 kHz regardless of
// what the UI is doing. Bytes are fanned out through SPSC rings:
//   * console ring  -> UI thread (console line editor)
//   * I/Q ring      -> SerialIqSource (only while that source is selected)
// With RS_IQ_ON_UART0 the console and the I/Q stream share UART0 (USB): every byte goes to both,
// and each parser ignores what is not meant for it (console commands are text lines, I/Q packets
// are CRC-protected binary frames).
#pragma once
#include <stdint.h>
#include "byte_ring.h"
#include "radar_serial.h"

typedef ByteRing<512> ConsoleRing;

class RxPump {
 public:
  void begin();                              // configure the dedicated I/Q UART if there is one
  void setIqEnabled(bool on) { iq_enabled_ = on; }
  void pump();                               // acquisition thread

  ConsoleRing& console() { return console_; }
  IqRing&      iq() { return iq_; }

 private:
  ConsoleRing console_;
  IqRing      iq_;
  volatile bool iq_enabled_ = false;
};
