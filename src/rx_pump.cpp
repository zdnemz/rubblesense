#include "rx_pump.h"
#include <Arduino.h>
#include "rs_build.h"
#include "rs_pins.h"

void RxPump::begin() {
#if !RS_IQ_ON_UART0
  Serial2.setRxBufferSize(RS_IQ_RX_RING);   // must precede begin()
  Serial2.begin(RS_IQ_BAUD, SERIAL_8N1, PIN_IQ_RX, PIN_IQ_TX);
#endif
}

void RxPump::pump() {
  int n = 0;
  while (n < 512 && Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    n++;
    const uint8_t b = static_cast<uint8_t>(c);
    console_.push(b);
#if RS_IQ_ON_UART0
    if (iq_enabled_) iq_.push(b);
#endif
  }
#if !RS_IQ_ON_UART0
  n = 0;
  while (n < 1024 && Serial2.available() > 0) {
    const int c = Serial2.read();
    if (c < 0) break;
    n++;
    if (iq_enabled_) iq_.push(static_cast<uint8_t>(c));   // not selected: drained and dropped
  }
#endif
}
