#!/usr/bin/env python3
"""Write the Wokwi circuit (diagram.json) from the firmware's pin map (src/rs_pins.h).

    python3 tools/make_diagram.py            # writes ./diagram.json
    python3 tools/check_diagram.py           # verifies it against rs_pins.h (also run after hand edits)

The wiring is generated, not hand-drawn, so the circuit cannot drift away from the firmware's pin map.
Wire routing is left to the simulator; drag parts around in the Wokwi editor as you like, then run
check_diagram.py again.

Parts (Wokwi part types) and their pins, from docs.wokwi.com/parts/*:
  board-esp32-devkit-c-v4   GPIOs are called by number ("esp:21"), plus 3V3, 5V, GND.1/.2/.3, TX, RX
  wokwi-ili9341             VCC GND CS RST D/C MOSI SCK LED MISO
  wokwi-mpu6050             VCC GND SCL SDA XDA XCL AD0 INT          (AD0 open: address 0x68)
  wokwi-pushbutton          1.l 1.r 2.l 2.r                           (1.l = 1.r and 2.l = 2.r inside)
  wokwi-led                 A (anode) C (cathode)
  wokwi-resistor            1 2
  wokwi-buzzer              1 (negative) 2 (positive)
  wokwi-slide-potentiometer GND SIG VCC
"""
import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_pins(path):
    pins = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = re.match(r"\s*#define\s+PIN_(\w+)\s+(\d+)", line)
            if m:
                pins[m.group(1)] = int(m.group(2))
    return pins


def build(pins):
    need = ["TFT_CS", "TFT_DC", "TFT_RST", "I2C_SDA", "I2C_SCL", "BTN_SCAN", "BTN_MODE", "BTN_MENU",
            "LED_CAND", "LED_UNCERT", "LED_SCAN", "BUZZER", "POT_TARGET", "POT_NOISE"]
    missing = [n for n in need if n not in pins]
    if missing:
        raise SystemExit("rs_pins.h lacks: " + ", ".join(missing))
    g = lambda n: "esp:%d" % pins[n]

    # Hardware SPI (VSPI) pins of the ESP32 Arduino core; the firmware does not remap them.
    SCK, MISO, MOSI = 18, 19, 23

    parts = [
        {"type": "board-esp32-devkit-c-v4", "id": "esp", "top": 0, "left": 0, "attrs": {}},
        {"type": "wokwi-ili9341", "id": "tft", "top": -60, "left": 140, "attrs": {}},
        {"type": "wokwi-mpu6050", "id": "mpu", "top": 20, "left": 420, "attrs": {}},
    ]
    # Pushbuttons (to GND, internal pull-ups): scan = green, mode = blue, menu = white.
    for i, (pid, pin, color, label) in enumerate([
            ("btn_scan", "BTN_SCAN", "green", "SCAN"),
            ("btn_mode", "BTN_MODE", "blue", "TRIAL"),
            ("btn_menu", "BTN_MENU", "white", "MENU")]):
        parts.append({"type": "wokwi-pushbutton", "id": pid, "top": 300, "left": 140 + i * 80,
                      "attrs": {"color": color, "label": label}})
    # LEDs with 220 ohm series resistors: green = candidate, yellow = uncertain, blue = scanning.
    for i, (pid, color, label) in enumerate([("cand", "green", "CAND"), ("unc", "yellow", "RESCAN"), ("scan", "blue", "SCAN")]):
        parts.append({"type": "wokwi-led", "id": "led_" + pid, "top": 100, "left": 430 + i * 50,
                      "attrs": {"color": color, "label": label}})
        parts.append({"type": "wokwi-resistor", "id": "r_" + pid, "top": 170 + i * 16, "left": 400 + i * 40,
                      "rotate": 90, "attrs": {"value": "220"}})
    parts.append({"type": "wokwi-buzzer", "id": "bz", "top": 250, "left": 430, "attrs": {"volume": "0.3"}})
    # Slide potentiometers: MANUAL trial only. Top = simulated target (0.5 -> amplitude 30),
    # bottom = simulated noise (0.14 -> sigma 8).
    parts.append({"type": "wokwi-slide-potentiometer", "id": "pot_target", "top": 40, "left": -200,
                  "attrs": {"travelLength": "30", "value": "512"}})
    parts.append({"type": "wokwi-slide-potentiometer", "id": "pot_noise", "top": 110, "left": -200,
                  "attrs": {"travelLength": "30", "value": "146"}})

    C = []
    add = lambda a, b, color: C.append([a, b, color, []])

    # serial monitor (console on UART0)
    add("esp:TX", "$serialMonitor:RX", "")
    add("esp:RX", "$serialMonitor:TX", "")

    # display: SPI + control lines; backlight and supply from 3V3
    add("tft:VCC", "esp:3V3", "red")
    add("tft:GND", "esp:GND.1", "black")
    add("tft:LED", "esp:3V3", "red")
    add("tft:CS", g("TFT_CS"), "orange")
    add("tft:RST", g("TFT_RST"), "gray")
    add("tft:D/C", g("TFT_DC"), "yellow")
    add("tft:MOSI", "esp:%d" % MOSI, "blue")
    add("tft:SCK", "esp:%d" % SCK, "purple")
    add("tft:MISO", "esp:%d" % MISO, "green")

    # IMU on I2C (AD0 open -> 0x68)
    add("mpu:VCC", "esp:3V3", "red")
    add("mpu:GND", "esp:GND.3", "black")
    add("mpu:SDA", g("I2C_SDA"), "blue")
    add("mpu:SCL", g("I2C_SCL"), "yellow")

    # buttons: GPIO <-> button <-> GND
    for pid, pin in [("btn_scan", "BTN_SCAN"), ("btn_mode", "BTN_MODE"), ("btn_menu", "BTN_MENU")]:
        add(pid + ":1.l", g(pin), "green")
        add(pid + ":2.l", "esp:GND.2", "black")

    # LEDs: GPIO -> resistor -> anode, cathode -> GND
    for pid, pin in [("cand", "LED_CAND"), ("unc", "LED_UNCERT"), ("scan", "LED_SCAN")]:
        add(g(pin), "r_%s:1" % pid, "green")
        add("r_%s:2" % pid, "led_%s:A" % pid, "green")
        add("led_%s:C" % pid, "esp:GND.2", "black")

    # buzzer
    add("bz:2", g("BUZZER"), "orange")
    add("bz:1", "esp:GND.2", "black")

    # potentiometers (ADC1 pins)
    add("pot_target:VCC", "esp:3V3", "red")
    add("pot_target:GND", "esp:GND.2", "black")
    add("pot_target:SIG", g("POT_TARGET"), "green")
    add("pot_noise:VCC", "esp:3V3", "red")
    add("pot_noise:GND", "esp:GND.2", "black")
    add("pot_noise:SIG", g("POT_NOISE"), "green")

    return {
        "version": 1,
        "author": "RubbleSense Research Team",
        "editor": "wokwi",
        "serialMonitor": {"display": "terminal", "newline": "lf", "convertEol": True},
        "parts": parts,
        "connections": C,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pins", default=os.path.join(ROOT, "src", "rs_pins.h"))
    ap.add_argument("--out", default=os.path.join(ROOT, "diagram.json"))
    args = ap.parse_args()
    d = build(read_pins(args.pins))
    with open(args.out, "w", encoding="utf-8") as f:
        # one part / one wire per line: readable in a diff
        f.write("{\n")
        for k in ("version", "author", "editor", "serialMonitor"):
            f.write('  "%s": %s,\n' % (k, json.dumps(d[k])))
        f.write('  "parts": [\n')
        f.write(",\n".join("    " + json.dumps(p) for p in d["parts"]))
        f.write("\n  ],\n  \"connections\": [\n")
        f.write(",\n".join("    " + json.dumps(c) for c in d["connections"]))
        f.write("\n  ]\n}\n")
    print("wrote %s (%d parts, %d wires)" % (args.out, len(d["parts"]), len(d["connections"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
