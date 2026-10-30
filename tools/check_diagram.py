#!/usr/bin/env python3
"""Check the Wokwi circuit (diagram.json) against the firmware's pin map (src/rs_pins.h).

    python3 tools/check_diagram.py [--diagram diagram.json] [--pins src/rs_pins.h]

Builds the netlist of the circuit (wires, the ESP32's three GND pins, the two halves of a push
button) and verifies that every function in rs_pins.h is wired to the right part pin: display,
IMU, buttons, LEDs (through their resistors), buzzer, potentiometers, serial monitor, power.
It also flags electrical mistakes (3V3 shorted to GND, a GPIO used twice, an input-only pin
driving an output, a potentiometer on an ADC2 pin, an unknown pin name). Exit status 0 = clean.

This is a netlist check, not a simulation: it cannot tell whether a part is drawn where you like it.
"""
import argparse
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PART_PINS = {
    "wokwi-ili9341": {"VCC", "GND", "CS", "RST", "D/C", "MOSI", "SCK", "LED", "MISO"},
    "wokwi-mpu6050": {"VCC", "GND", "SCL", "SDA", "XDA", "XCL", "AD0", "INT"},
    "wokwi-pushbutton": {"1.l", "1.r", "2.l", "2.r"},
    "wokwi-led": {"A", "C"},
    "wokwi-resistor": {"1", "2"},
    "wokwi-buzzer": {"1", "2"},
    "wokwi-slide-potentiometer": {"GND", "SIG", "VCC"},
    "wokwi-potentiometer": {"GND", "SIG", "VCC"},
}
ESP_TYPE = "board-esp32-devkit-c-v4"
ESP_PINS = ({str(n) for n in (0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35)}
            | {"VP", "VN", "3V3", "5V", "EN", "TX", "RX", "GND.1", "GND.2", "GND.3"})
INPUT_ONLY = {34, 35, 36, 39}
ADC1 = {32, 33, 34, 35, 36, 39}
STRAPPING = {0, 2, 5, 12, 15}
FLASH = {6, 7, 8, 9, 10, 11}
SPI_PINS = {"SCK": 18, "MISO": 19, "MOSI": 23}       # VSPI defaults of the Arduino-ESP32 core


class Nets:
    def __init__(self):
        self.parent = {}

    def find(self, a):
        self.parent.setdefault(a, a)
        while self.parent[a] != a:
            self.parent[a] = self.parent[self.parent[a]]
            a = self.parent[a]
        return a

    def join(self, a, b):
        self.parent[self.find(a)] = self.find(b)

    def same(self, a, b):
        return self.find(a) == self.find(b)


def read_pins(path):
    pins = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = re.match(r"\s*#define\s+PIN_(\w+)\s+(\d+)", line)
            if m:
                pins[m.group(1)] = int(m.group(2))
    return pins


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--diagram", default=os.path.join(ROOT, "diagram.json"))
    ap.add_argument("--pins", default=os.path.join(ROOT, "src", "rs_pins.h"))
    args = ap.parse_args()

    errors, warns = [], []
    err = errors.append
    pins = read_pins(args.pins)
    try:
        with open(args.diagram, "r", encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError) as e:
        print("cannot read %s: %s" % (args.diagram, e))
        return 1

    # ---- structure
    if d.get("version") != 1:
        err("version must be 1")
    parts = {}
    for p in d.get("parts", []):
        pid, typ = p.get("id"), p.get("type")
        if not pid or not typ:
            err("part without id/type: %r" % (p,))
            continue
        if pid in parts:
            err("duplicate part id '%s'" % pid)
        parts[pid] = typ
    esp_ids = [i for i, t in parts.items() if t == ESP_TYPE]
    if len(esp_ids) != 1:
        err("need exactly one %s, found %d" % (ESP_TYPE, len(esp_ids)))
        return report(errors, warns)
    esp = esp_ids[0]
    for pid, typ in parts.items():
        if typ != ESP_TYPE and typ not in PART_PINS:
            warns.append("part '%s' has a type this checker does not know (%s)" % (pid, typ))

    nets = Nets()
    used_pin_count = {}

    def node(ref):
        return ref

    def valid_endpoint(ref):
        if ref.startswith("$serialMonitor:"):
            return ref.split(":", 1)[1] in ("TX", "RX")
        if ":" not in ref:
            return False
        pid, pin = ref.split(":", 1)
        if pid not in parts:
            return False
        typ = parts[pid]
        if typ == ESP_TYPE:
            return pin in ESP_PINS
        return pin in PART_PINS.get(typ, {pin})

    for c in d.get("connections", []):
        if len(c) < 3:
            err("malformed connection %r" % (c,))
            continue
        a, b = c[0], c[1]
        ok = True
        for ref in (a, b):
            if not valid_endpoint(ref):
                err("unknown part or pin in wire %s -> %s: '%s'" % (a, b, ref))
                ok = False
        if ok:
            nets.join(node(a), node(b))
            for ref in (a, b):
                used_pin_count[ref] = used_pin_count.get(ref, 0) + 1

    # internal connections
    for g in ("GND.1", "GND.2", "GND.3"):
        nets.join(esp + ":GND.1", esp + ":" + g)
    for pid, typ in parts.items():
        if typ == "wokwi-pushbutton":
            nets.join(pid + ":1.l", pid + ":1.r")
            nets.join(pid + ":2.l", pid + ":2.r")

    GND, V33 = esp + ":GND.1", esp + ":3V3"
    gpio = lambda n: "%s:%d" % (esp, n)

    if nets.same(GND, V33):
        err("3V3 is shorted to GND")

    def need(cond, msg):
        if not cond:
            err(msg)

    def must_pin(name):
        if name not in pins:
            err("rs_pins.h does not define PIN_%s" % name)
            return None
        return pins[name]

    # ---- one function per GPIO
    seen = {}
    for name, n in sorted(pins.items()):
        if n in seen:
            err("GPIO%d is used for both %s and %s" % (n, seen[n], name))
        seen[n] = name
        if n in FLASH:
            err("PIN_%s = GPIO%d is connected to the flash chip" % (name, n))

    # ---- display
    for sig, pname in (("CS", "TFT_CS"), ("D/C", "TFT_DC"), ("RST", "TFT_RST")):
        n = must_pin(pname)
        if n is not None:
            need(nets.same("tft:" + sig, gpio(n)), "tft:%s is not on GPIO%d (PIN_%s)" % (sig, n, pname))
            need(n not in INPUT_ONLY, "PIN_%s = GPIO%d is input-only but drives the display" % (pname, n))
    for sig, n in SPI_PINS.items():
        need(nets.same("tft:" + sig, gpio(n)), "tft:%s must be on GPIO%d (hardware SPI)" % (sig, n))
    need(nets.same("tft:VCC", V33), "tft:VCC is not on 3V3")
    need(nets.same("tft:LED", V33), "tft:LED (backlight) is not on 3V3")
    need(nets.same("tft:GND", GND), "tft:GND is not on GND")

    # ---- IMU
    for sig, pname in (("SDA", "I2C_SDA"), ("SCL", "I2C_SCL")):
        n = must_pin(pname)
        if n is not None:
            need(nets.same("mpu:" + sig, gpio(n)), "mpu:%s is not on GPIO%d (PIN_%s)" % (sig, n, pname))
    need(nets.same("mpu:VCC", V33), "mpu:VCC is not on 3V3")
    need(nets.same("mpu:GND", GND), "mpu:GND is not on GND")
    need(used_pin_count.get("mpu:AD0", 0) == 0 or nets.same("mpu:AD0", GND),
         "mpu:AD0 must be open or on GND (address 0x68 is what the firmware probes first)")

    # ---- buttons
    for pid, pname in (("btn_scan", "BTN_SCAN"), ("btn_mode", "BTN_MODE"), ("btn_menu", "BTN_MENU")):
        n = must_pin(pname)
        if pid not in parts:
            err("part '%s' is missing" % pid)
            continue
        if n is not None:
            need(nets.same(pid + ":1.l", gpio(n)), "%s is not on GPIO%d (PIN_%s)" % (pid, n, pname))
            need(nets.same(pid + ":2.l", GND), "%s: the other contact is not on GND" % pid)
            need(not nets.same(pid + ":1.l", pid + ":2.l"), "%s is shorted" % pid)
            if n in STRAPPING:
                warns.append("PIN_%s = GPIO%d is a boot-strapping pin; a button held at reset can change the boot mode" % (pname, n))

    # ---- LEDs through their resistors
    for tag, pname in (("cand", "LED_CAND"), ("unc", "LED_UNCERT"), ("scan", "LED_SCAN")):
        n = must_pin(pname)
        r, led = "r_" + tag, "led_" + tag
        if r not in parts or led not in parts:
            err("part '%s' or '%s' is missing" % (r, led))
            continue
        if n is not None:
            need(nets.same(r + ":1", gpio(n)), "%s:1 is not on GPIO%d (PIN_%s)" % (r, n, pname))
            need(nets.same(r + ":2", led + ":A"), "%s:2 does not feed the anode of %s" % (r, led))
            need(not nets.same(r + ":1", r + ":2"), "%s is bypassed (GPIO wired straight to the LED)" % r)
            need(nets.same(led + ":C", GND), "%s cathode is not on GND" % led)
            need(n not in INPUT_ONLY, "PIN_%s = GPIO%d is input-only but drives an LED" % (pname, n))

    # ---- buzzer
    n = must_pin("BUZZER")
    if n is not None:
        need("bz" in parts, "part 'bz' (buzzer) is missing")
        need(nets.same("bz:2", gpio(n)), "bz:2 (+) is not on GPIO%d (PIN_BUZZER)" % n)
        need(nets.same("bz:1", GND), "bz:1 (-) is not on GND")
        need(n not in INPUT_ONLY, "PIN_BUZZER = GPIO%d is input-only" % n)

    # ---- potentiometers
    for pid, pname in (("pot_target", "POT_TARGET"), ("pot_noise", "POT_NOISE")):
        n = must_pin(pname)
        if pid not in parts:
            err("part '%s' is missing" % pid)
            continue
        if n is not None:
            need(nets.same(pid + ":SIG", gpio(n)), "%s:SIG is not on GPIO%d (PIN_%s)" % (pid, n, pname))
            need(n in ADC1, "PIN_%s = GPIO%d is not an ADC1 pin (ADC2 is unusable with Wi-Fi and shares strapping pins)" % (pname, n))
        need(nets.same(pid + ":VCC", V33), "%s:VCC is not on 3V3" % pid)
        need(nets.same(pid + ":GND", GND), "%s:GND is not on GND" % pid)

    # ---- serial monitor (console on UART0)
    need(nets.same("esp:TX", "$serialMonitor:RX"), "esp:TX is not wired to the serial monitor's RX")
    need(nets.same("esp:RX", "$serialMonitor:TX"), "esp:RX is not wired to the serial monitor's TX")

    # ---- every part is used
    for pid, typ in parts.items():
        if typ == ESP_TYPE:
            continue
        if not any(ref.startswith(pid + ":") for ref in used_pin_count):
            warns.append("part '%s' has no wire" % pid)

    return report(errors, warns, len(parts), len(d.get("connections", [])))


def report(errors, warns, n_parts=0, n_wires=0):
    for w in warns:
        print("warning:", w)
    for e in errors:
        print("ERROR:", e)
    if errors:
        print("diagram check FAILED: %d error(s), %d warning(s)" % (len(errors), len(warns)))
        return 1
    print("diagram check OK: %d parts, %d wires, pin map matches src/rs_pins.h (%d warning(s))" % (n_parts, n_wires, len(warns)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
