// RubbleSense firmware - entry point (device only; the host test-bench has its own main).
//
//   core 1  Arduino loop task : App::update()          UI, buttons, console, scan state machine
//   core 0  rs_acq  (prio 4)  : App::backgroundStep()  UART pump, IMU at 100 Hz, radar polling
//   core 0  rs_dsp  (prio 2)  : DspWorker               segment analysis (created in App::begin)
//
// Research prototype. A negative scan never proves that nobody is there.
#include <Arduino.h>
#include "rs_build.h"

#if !RS_HOST
#include "app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
App  g_app;
bool g_ready = false;

void acqTask(void*) {
  for (;;) {
    g_app.backgroundStep();
    vTaskDelay(1);                       // 1 ms: the Arduino core runs FreeRTOS at 1 kHz
  }
}
}  // namespace

void setup() {
  g_ready = g_app.begin();
  if (g_ready) xTaskCreatePinnedToCore(acqTask, "rs_acq", 6144, nullptr, 4, nullptr, 0);
}

void loop() {
  if (g_ready) g_app.update();
  delay(5);
}
#endif
