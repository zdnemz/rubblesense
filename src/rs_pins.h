// RubbleSense firmware - ESP32 DevKit pin map. diagram.json (project root) is generated from this file by
// tools/make_diagram.py and verified against it by tools/check_diagram.py.
//
// Chosen to avoid the ESP32 boot-strapping pins (0, 2, 5*, 12, 15) for inputs that can be held
// at boot, to keep the ADC pots on ADC1 (ADC2 is unusable when Wi-Fi is on and is shared with
// strapping pins) and to leave UART0 free for the console.
//   * GPIO5 is a strapping pin only for SDIO timing; it is fine as the TFT chip-select.
//   * GPIO34..39 are input only: pots and the I/Q receive line.
#pragma once

// TFT (ILI9341, hardware VSPI: SCK 18, MISO 19, MOSI 23)
#define PIN_TFT_CS    5
#define PIN_TFT_DC    16
#define PIN_TFT_RST   17

// I2C bus of the IMU (MPU6050)
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22

// Push buttons to GND (internal pull-ups)
#define PIN_BTN_SCAN  32   // green: start / abort scan, select in menus
#define PIN_BTN_MODE  33   // blue : next trial preset / next menu entry
#define PIN_BTN_MENU  4    // white: open / close menu

// Indicators
#define PIN_LED_CAND    25   // green : candidate pattern
#define PIN_LED_UNCERT  26   // yellow: uncertain / rescan / inconclusive
#define PIN_LED_SCAN    27   // blue  : scanning (steady) / ready (blink)
#define PIN_BUZZER      13   // passive buzzer (LEDC tone)

// Simulation controls (MANUAL preset of the simulated radar only)
#define PIN_POT_TARGET  34   // simulated target reflection amplitude
#define PIN_POT_NOISE   35   // simulated receiver noise

// Dedicated I/Q UART to a radar front-end MCU (RS_IQ_ON_UART0 == 0)
#define PIN_IQ_RX     36
#define PIN_IQ_TX     14
