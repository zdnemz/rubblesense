#!/usr/bin/env python3
"""Self-test of the Python tooling and of the recording-replay path.

    python3 tools/test_tools.py [--bin-dir build/bin] [--no-e2e] [--keep DIR]

  1  serial I/Q protocol (iqproto): golden packets shared with the C++ tests, CRC check value, decoder resync
  2  Wokwi circuit: the committed diagram.json is what make_diagram.py generates, passes the netlist check,
     and every one of a dozen deliberate wiring mistakes is caught
  3  scan-log validator: good records pass, every deliberate violation of the safety rules is caught
  4  send_iq_replay.py: contents of the byte stream, fault injection, embedded console commands, input errors
  5  end to end (needs `make tools`): dump_sim -> send_iq_replay --out -> fw_replay, i.e. the real firmware
     sources on the host: the six trial presets, calibration, a low-sensitivity case, link and time-base
     faults, and the same path through a TCP socket (the code path of --port / --tcp)

All of it runs on simulated data (white-Gaussian simulator). It checks the tooling and that the firmware logic
handles these streams; it says nothing about a real radar, real rubble or an ESP32's timing.
Exit status 0 = everything passed.
"""
import argparse
import contextlib
import copy
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
sys.path.insert(0, TOOLS)
import check_diagram  # noqa: E402
import iqproto  # noqa: E402
import make_diagram  # noqa: E402
import validate_scan_log  # noqa: E402

SEND = os.path.join(TOOLS, "send_iq_replay.py")
MANDATORY = "No reliable life-sign pattern detected in this scan. DO NOT clear area."

PASS = 0
FAIL = 0


def section(title):
    print("\n== %s" % title)


def check(cond, name, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
        print("  FAIL %s%s" % (name, (" - " + detail) if detail else ""))


def run(args, **kw):
    kw.setdefault("capture_output", True)
    kw.setdefault("text", True)
    kw.setdefault("timeout", 300)
    return subprocess.run(args, **kw)


# ------------------------------------------------------------------------------------------------ 1. protocol
def test_protocol():
    section("1. serial I/Q protocol")
    iq3 = [(1, -1), (256, -256), (32767, -32768)]
    check(iqproto.encode_frame(0x1234, 0x0A0B0C0D, iq3, iqproto.FLAG_MOTION | iqproto.FLAG_HOST_IMU).hex() == iqproto.GOLDEN["frame"],
          "golden frame packet")
    check(iqproto.encode_frame(0xFFFF, 0, iq3, iqproto.FLAG_SATURATED).hex() == iqproto.GOLDEN["frame_sat_wrap"], "golden frame, seq wrap")
    check(iqproto.encode_info(20.0, 0.5, 0.15, 5.0, 4).hex() == iqproto.GOLDEN["info"], "golden info packet")
    check(iqproto.crc16_ccitt(b"123456789") == 0x29B1, "CRC-16/CCITT-FALSE check value")
    for n in (0, iqproto.MAX_BINS + 1):
        try:
            iqproto.encode_frame(0, 0, [(0, 0)] * n)
            check(False, "frame with %d bins must be refused" % n)
        except ValueError:
            check(True, "")

    pk = [iqproto.encode_info(20.0, 0.5, 0.15, 125.0, 16)]
    for i in range(50):
        pk.append(iqproto.encode_frame(i, i * 50, [(i, -i)] * 16, iqproto.FLAG_HOST_IMU if i % 2 else 0))
    stream = b"".join(pk)
    d = iqproto.Decoder()
    out = d.feed(stream)
    check(len(out) == 51 and out[0]["type"] == "info" and out[1]["type"] == "frame", "decoder: all packets of a clean stream")
    check(out[0]["n_bins"] == 16 and abs(out[0]["range_step_m"] - 0.15) < 1e-6, "decoder: info fields")
    check(out[7]["seq"] == 6 and out[7]["t_ms"] == 300 and out[7]["iq"][3] == (6, -6) and out[2]["flags"] == iqproto.FLAG_HOST_IMU,
          "decoder: frame fields")
    d2 = iqproto.Decoder()
    out2 = []
    for b in stream:
        out2 += d2.feed(bytes([b]))
    check(out2 == out, "decoder: byte-wise feeding gives the same packets")

    bad = bytearray(pk[5])
    bad[20] ^= 0x04
    messy = b"\x00\xA5\x00\x5A" + pk[1] + bytes(bad) + pk[6] + pk[7][:30] + pk[8]
    d3 = iqproto.Decoder()
    got = [p["seq"] for p in d3.feed(messy) if p["type"] == "frame"]
    check(got == [0, 5, 7], "decoder resynchronises after garbage, a corrupt and a truncated packet", str(got))
    check(d3.crc_errors >= 2 and d3.skipped > 0, "decoder counts what it threw away")


# ------------------------------------------------------------------------------------------------ 2. diagram
def test_diagram(tmp):
    section("2. Wokwi circuit")
    pins_path = os.path.join(ROOT, "src", "rs_pins.h")
    committed = os.path.join(ROOT, "diagram.json")
    gen = os.path.join(tmp, "gen.json")
    r = run([sys.executable, os.path.join(TOOLS, "make_diagram.py"), "--out", gen])
    check(r.returncode == 0, "make_diagram runs", r.stderr)
    check(os.path.exists(gen) and open(gen, "rb").read() == open(committed, "rb").read(),
          "diagram.json is exactly what make_diagram.py generates from rs_pins.h")
    r = run([sys.executable, os.path.join(TOOLS, "check_diagram.py")])
    check(r.returncode == 0 and "diagram check OK" in r.stdout, "committed diagram passes the netlist check", r.stdout)

    base = make_diagram.build(make_diagram.read_pins(pins_path))

    def conn(d, a, b=None):
        for c in d["connections"]:
            if (b is None and (c[0] == a or c[1] == a)) or (b is not None and {c[0], c[1]} == {a, b}):
                return c
        raise KeyError((a, b))

    def drop(d, a, b=None):
        d["connections"].remove(conn(d, a, b))

    def retarget(d, a, old, new):
        c = conn(d, a, old)
        c[0 if c[0] == old else 1] = new

    def m_tft_cs(d): retarget(d, "tft:CS", "esp:5", "esp:12")
    def m_i2c_swap(d):
        retarget(d, "mpu:SDA", "esp:21", "esp:22")
        retarget(d, "mpu:SCL", "esp:22", "esp:21")
    def m_bypass_resistor(d):
        drop(d, "r_cand:2", "led_cand:A")
        d["connections"].append(["esp:25", "led_cand:A", "green", []])
    def m_short(d): d["connections"].append(["esp:3V3", "esp:GND.1", "red", []])
    def m_button_floating(d): drop(d, "btn_scan:2.l", "esp:GND.2")
    def m_pot_moved(d): retarget(d, "pot_noise:SIG", "esp:35", "esp:2")
    def m_unknown_pin(d): retarget(d, "tft:CS", "esp:5", "esp:99")
    def m_missing_part(d): d["parts"] = [p for p in d["parts"] if p["id"] != "mpu"]
    def m_buzzer_polarity(d):
        retarget(d, "bz:2", "esp:13", "esp:GND.2")
        retarget(d, "bz:1", "esp:GND.2", "esp:13")
    def m_serial_swap(d):
        retarget(d, "esp:TX", "$serialMonitor:RX", "$serialMonitor:TX")
        retarget(d, "esp:RX", "$serialMonitor:TX", "$serialMonitor:RX")
    def m_backlight(d): drop(d, "tft:LED", "esp:3V3")
    def m_spi_mosi(d): retarget(d, "tft:MOSI", "esp:23", "esp:19")

    mutations = [
        ("display CS moved to another GPIO", m_tft_cs, "tft:CS is not on GPIO5"),
        ("I2C SDA / SCL swapped", m_i2c_swap, "mpu:SDA is not on GPIO21"),
        ("LED wired without its resistor", m_bypass_resistor, "r_cand:2 does not feed"),
        ("3V3 shorted to GND", m_short, "3V3 is shorted to GND"),
        ("button contact not on GND", m_button_floating, "btn_scan: the other contact is not on GND"),
        ("potentiometer moved to a non-ADC1 pin", m_pot_moved, "pot_noise:SIG is not on GPIO35"),
        ("wire to a pin the board does not have", m_unknown_pin, "unknown part or pin"),
        ("IMU part missing", m_missing_part, "unknown part or pin"),
        ("buzzer polarity reversed", m_buzzer_polarity, "bz:2 (+) is not on GPIO13"),
        ("serial monitor RX/TX swapped", m_serial_swap, "serial monitor"),
        ("display backlight not powered", m_backlight, "tft:LED (backlight) is not on 3V3"),
        ("display MOSI on the wrong SPI pin", m_spi_mosi, "tft:MOSI must be on GPIO23"),
    ]
    for name, fn, expect in mutations:
        d = copy.deepcopy(base)
        fn(d)
        path = os.path.join(tmp, "mut.json")
        with open(path, "w") as f:
            json.dump(d, f)
        r = run([sys.executable, os.path.join(TOOLS, "check_diagram.py"), "--diagram", path])
        check(r.returncode == 1 and expect in r.stdout, "diagram mistake caught: " + name, r.stdout.strip()[-300:])

    # a duplicated GPIO in the pin map itself
    with open(pins_path) as f:
        pins_txt = f.read()
    dup = re.sub(r"(#define\s+PIN_BUZZER\s+)\d+", r"\g<1>25", pins_txt)
    pp = os.path.join(tmp, "pins_dup.h")
    with open(pp, "w") as f:
        f.write(dup)
    r = run([sys.executable, os.path.join(TOOLS, "check_diagram.py"), "--pins", pp])
    check(r.returncode == 1 and "GPIO25 is used for both" in r.stdout, "pin map with one GPIO used twice is caught", r.stdout.strip()[-300:])


# ------------------------------------------------------------------------------------------------ 3. validator
def sample_candidate():
    return {
        "v": 1, "fw": "1.0.0", "scan_id": "RS-0001", "uptime_s": 56, "duration_s": 52.7, "source": "SERIAL", "preset": "",
        "imu": "host", "verdict": "CANDIDATE_PATTERN", "evidence": "strong", "result": "CANDIDATE LIFE-SIGN PATTERN",
        "operator_action": "Confirm with a 2nd position and audio / canine / camera. This is NOT proof of life.",
        "invalid_reason": "none", "bin": 9, "range_m": 1.85, "hz": 0.281, "bpm": 16.8, "snr_db": 19.6, "margin_db": 11.6,
        "segments_agree": 2, "vetted": True, "segments_total": 2, "segments_valid": 2, "segments_invalid": 0,
        "floor_db": 21.6, "sensitivity": "unknown", "baseline_src": "none", "cpe_applied": True, "vibration": False,
        "vibration_hz": 0.0, "platform_motion_mm_rms": 0.014,
        "radar": {"name": "SERIAL", "fs_hz": 20.0, "range_start_m": 0.5, "range_step_m": 0.15, "wavelength_mm": 125.0, "bins": 16},
        "link": {"lost_frames": 0, "crc_errors": 0, "timing_flagged": 0, "timestamps": True},
        "segments": [
            {"valid": True, "reason": "none", "det": 2, "platform": 0, "untested": 0, "confirmed": False, "best_bin": 9,
             "best_hz": 0.282, "best_margin_db": 7.6, "floor_db": 21.4, "bad_frac": 0.0, "cpe": True, "vib": False},
            {"valid": True, "reason": "none", "det": 3, "platform": 0, "untested": 0, "confirmed": True, "best_bin": 9,
             "best_hz": 0.279, "best_margin_db": 9.3, "floor_db": 21.6, "bad_frac": 0.0, "cpe": True, "vib": False}],
        "params": {"pfa": 0.01, "pfc": 0.02, "band_hz": [0.1, 0.8], "min_bin": 2, "cpe": True, "guard": 1, "low_sens_db": 3.0,
                   "require_imu": True},
        "safety_notice": "Aid only. Never infer absence of survivor from a negative scan."}


def sample_negative():
    r = sample_candidate()
    for k in ("bin", "range_m", "hz", "bpm", "snr_db", "margin_db", "segments_agree", "vetted"):
        del r[k]
    r.update(verdict="NO_RELIABLE_PATTERN", evidence="none", result=MANDATORY,
             operator_action="Rescan from a 2nd position and use other methods. Do not clear area.")
    for s in r["segments"]:
        s.update(det=0, confirmed=False, best_bin=0, best_hz=0.0, best_margin_db=0.0)
    return r


def validate(records, extra=()):
    path = os.path.join(tempfile.mkdtemp(prefix="rs_val_"), "log.jsonl")
    with open(path, "w") as f:
        for r in records:
            f.write((r if isinstance(r, str) else json.dumps(r)) + "\n")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = validate_scan_log.main([path, "--quiet"] + list(extra))
    shutil.rmtree(os.path.dirname(path), ignore_errors=True)
    return rc, buf.getvalue()


def test_validator():
    section("3. scan-log validator")
    rc, out = validate([sample_candidate(), sample_negative()])
    check(rc == 0, "good candidate and good negative pass", out)

    low = sample_negative()
    low.update(verdict="LOW_SENSITIVITY", result="LOW SENSITIVITY - INCONCLUSIVE", sensitivity="low", baseline_src="calibrated",
               baseline_db=21.2, floor_delta_db=9.6, floor_db=30.8,
               operator_action="Noise is far above the calibrated level: a negative here means NOTHING. Do not clear area.")
    rc, out = validate([low])
    check(rc == 0, "good LOW_SENSITIVITY record passes", out)
    poss = sample_candidate()
    poss.update(verdict="POSSIBLE_PATTERN", evidence="weak", result="POSSIBLE PATTERN - NOT CONFIRMED", segments_agree=1, vetted=False,
                sensitivity="low", baseline_src="calibrated", baseline_db=21.1, floor_delta_db=9.5, floor_db=30.6,
                operator_action="Rescan from the same spot, then from a 2nd position. Do not clear area.")
    rc, out = validate([poss])
    check(rc == 0, "a POSSIBLE line at a raised floor is allowed to say sensitivity 'low'", out)
    inv = sample_negative()
    inv.update(verdict="INVALID_SCAN", result="INVALID SCAN", invalid_reason="sensor_motion", segments_valid=0, segments_invalid=2,
               operator_action="The sensor moved during the scan. Hold it still and rescan. Do not clear area.")
    for s in inv["segments"]:
        s.update(valid=False, reason="sensor_motion")
    rc, out = validate([inv])
    check(rc == 0, "good INVALID_SCAN record passes", out)

    def bad(name, base, fn, expect):
        r = copy.deepcopy(base)
        fn(r)
        rc, out = validate([r])
        check(rc == 1 and expect in out, "validator catches: " + name, out.strip()[-300:])

    C, N = sample_candidate(), sample_negative()
    bad("a verdict that says 'all clear'", C, lambda r: r.update(verdict="ALL_CLEAR"), "not one of the defined verdicts")
    bad("a negative without the mandatory sentence", N, lambda r: r.update(result="No survivors detected."), "mandatory sentence")
    bad("a missing safety notice", C, lambda r: r.pop("safety_notice"), "missing field(s)")
    bad("an altered safety notice", C, lambda r: r.update(safety_notice="ok"), "safety_notice is missing or altered")
    bad("a confidence figure", C, lambda r: r.update(confidence=0.93), "unknown field")
    bad("text that clears the area", C, lambda r: r.update(operator_action="Area is clear, safe to leave."), "claims the area is clear")
    bad("text that says nobody is there", N, lambda r: r.update(operator_action="Nobody is there."), "nobody")
    bad("a percentage in the text", C, lambda r: r.update(result="CANDIDATE - 93% sure"), "percentage")
    bad("a candidate seen in one segment only", C, lambda r: r.update(segments_agree=1), "segments_agree")
    bad("a negative at a raised noise floor", N, lambda r: r.update(sensitivity="low", baseline_src="calibrated", baseline_db=21.0,
                                                                   floor_delta_db=9.6), "negative at low sensitivity")
    bad("a sensitivity statement that contradicts the numbers", C,
        lambda r: r.update(sensitivity="ok", baseline_src="calibrated", baseline_db=21.0, floor_delta_db=9.6), "sensitivity 'ok'")
    bad("bpm that does not match hz", C, lambda r: r.update(bpm=40.0), "does not match hz")
    bad("a range that does not match the bin", C, lambda r: r.update(range_m=3.0), "does not match bin")
    bad("a candidate inside the leakage bins", C, lambda r: r.update(bin=1, range_m=0.65), "below min_bin")
    bad("an invalid scan without a reason", sample_negative(), lambda r: r.update(verdict="INVALID_SCAN", result="INVALID SCAN"), "without a reason")
    bad("candidate fields on a negative", N, lambda r: r.update(bin=9, range_m=1.85, hz=0.28, bpm=16.8, snr_db=15, margin_db=9,
                                                                   segments_agree=2, vetted=True), "must not carry candidate fields")
    bad("evidence on a negative", N, lambda r: r.update(evidence="strong"), "with evidence")
    bad("segment counts that do not add up", C, lambda r: r.update(segments_invalid=1), "!= segments_total")
    bad("timing flags without timestamps", C, lambda r: r["link"].update(timing_flagged=5, timestamps=False), "no timestamps")
    bad("a baseline without a sensitivity statement", C, lambda r: r.update(baseline_src="calibrated"), "sensitivity is 'unknown'")
    rc, out = validate(["{not json"])
    check(rc == 1 and "not valid JSON" in out, "validator catches a broken line", out)
    rc, out = validate([])
    check(rc == 1 and "expected at least 1" in out, "validator fails on an empty log", out)
    rc, out = validate(["# just chatter", "OK"])
    check(rc == 1, "validator fails on a log without scan records", out)


# ------------------------------------------------------------------------------------------------ 4. sender
CSV_TINY = """# rs_iq_csv v1
# fs_hz=20 range_start_m=0.5 range_step_m=0.15 wavelength_mm=125 n_bins=2
0,1,2,3,4
2,5,6,7,8
1,-1,-2,-3,-4
"""


def decode_file(path):
    d = iqproto.Decoder()
    with open(path, "rb") as f:
        pk = d.feed(f.read())
    return pk, d


def send(tmp, name, args):
    out = os.path.join(tmp, name)
    r = run([sys.executable, SEND] + args + ["--out", out, "--quiet"])
    return r, out


def test_sender(tmp):
    section("4. send_iq_replay.py")
    # a hand-written recording
    csv = os.path.join(tmp, "tiny.csv")
    with open(csv, "w") as f:
        f.write(CSV_TINY)
    r, out = send(tmp, "tiny.bin", ["--file", csv, "--info-period", "100"])
    check(r.returncode == 0, "tiny CSV is accepted", r.stderr)
    pk, d = decode_file(out)
    fr = [p for p in pk if p["type"] == "frame"]
    check(len(fr) == 3 and [p["seq"] for p in fr] == [0, 1, 2] and [p["t_ms"] for p in fr] == [0, 50, 100], "frames: seq and t_ms")
    check(fr[1]["iq"] == [(5, 6), (7, 8)] and fr[2]["iq"] == [(-1, -2), (-3, -4)], "frames: I/Q values")
    hi = iqproto.FLAG_HOST_IMU
    check([p["flags"] for p in fr] == [hi, hi | iqproto.FLAG_MOTION, hi | iqproto.FLAG_SATURATED],
          "frames: motion / saturation from the recording, host-IMU marker set because the recording has flags")
    check(len([p for p in pk if p["type"] == "info"]) == 1 and pk[0]["type"] == "info" and pk[0]["n_bins"] == 2 and pk[0]["frame_rate_hz"] == 20.0,
          "info packet first, with the recording's geometry")
    r, out = send(tmp, "tiny2.bin", ["--file", csv, "--no-host-imu", "--no-stamps", "--loops", "3", "--announce-fs", "25"])
    pk, d = decode_file(out)
    fr = [p for p in pk if p["type"] == "frame"]
    check(len(fr) == 9 and [p["seq"] for p in fr] == list(range(9)), "--loops 3 continues the sequence")
    check(all(p["t_ms"] == 0 for p in fr), "--no-stamps sends t_ms = 0")
    check(all(not (p["flags"] & hi) for p in fr) and fr[1]["flags"] == iqproto.FLAG_MOTION, "--no-host-imu clears the marker only")
    check(pk[0]["frame_rate_hz"] == 25.0, "--announce-fs changes the announced rate only")

    # a synthetic scene
    r, out = send(tmp, "synth.bin", ["--synth", "--seconds", "20", "--seed", "5"])
    check(r.returncode == 0, "--synth runs", r.stderr)
    pk, d = decode_file(out)
    fr = [p for p in pk if p["type"] == "frame"]
    inf = [p for p in pk if p["type"] == "info"]
    check(len(fr) == 400 and [p["seq"] for p in fr] == list(range(400)), "synth: 400 frames, continuous sequence")
    check(all(p["t_ms"] == 50 * i for i, p in enumerate(fr)), "synth: timestamps follow the frame rate")
    check(19 <= len(inf) <= 21 and inf[0]["n_bins"] == 16, "synth: about one info packet per second")
    check(all(not (p["flags"] & hi) for p in fr), "synth: no host-IMU marker (there is no IMU data)")
    check(d.crc_errors == 0 and d.bad_packets == 0, "synth: clean stream")

    # faults
    def faulty(name, extra):
        r, out = send(tmp, name + ".bin", ["--synth", "--seconds", "20", "--seed", "5"] + extra)
        pk, d = decode_file(out)
        return pk, d, [p for p in pk if p["type"] == "frame"]

    pk, d, fr = faulty("drop", ["--drop-every", "10"])
    check(len(fr) == 360 and d.crc_errors == 0, "--drop-every 10: every tenth frame missing", str(len(fr)))
    seqs = [p["seq"] for p in fr]
    check(all(b - a in (1, 2) for a, b in zip(seqs, seqs[1:])) and any(b - a == 2 for a, b in zip(seqs, seqs[1:])), "drops leave sequence gaps")
    pk, d, fr = faulty("corrupt", ["--corrupt-every", "10"])
    # a flipped bit in the version / type / length field is rejected by the header checks before the CRC is reached
    check(len(fr) == 360 and d.crc_errors + d.bad_packets >= 40 and d.crc_errors >= 30,
          "--corrupt-every 10: the CRC and header checks catch them", "%d frames, %d crc, %d header" % (len(fr), d.crc_errors, d.bad_packets))
    pk, d, fr = faulty("dup", ["--dup-every", "10"])
    check(len(fr) == 440, "--dup-every 10: every tenth frame twice", str(len(fr)))
    pk, d, fr = faulty("garbage", ["--garbage-every", "10"])
    check(len(fr) >= 395 and d.skipped > 40, "--garbage-every 10: noise between packets, decoder resynchronises", "%d frames, %d skipped" % (len(fr), d.skipped))
    pk, d, fr = faulty("pause", ["--pause-at", "5", "--pause-for", "1.5"])
    jumps = [b["t_ms"] - a["t_ms"] for a, b in zip(fr, fr[1:])]
    check(len(fr) == 400 and max(jumps) == 1550 and [p["seq"] for p in fr] == list(range(400)), "--pause-at: a 1.5 s gap in the stamps without a sequence gap")

    # embedded console commands
    r, out = send(tmp, "cmd.bin", ["--synth", "--seconds", "20", "--cmd", "src serial", "--cmd", "scan", "--cmd-delay", "1", "--cmd-gap", "3"])
    raw = open(out, "rb").read()
    pk, d = decode_file(out)
    fr = [p for p in pk if p["type"] == "frame"]
    check(b"\nsrc serial\n" in raw and b"\nscan\n" in raw and raw.index(b"\nsrc serial\n") < raw.index(b"\nscan\n"), "--cmd lines are embedded in order")
    check(len(fr) == 400 and d.crc_errors == 0, "embedded commands sit between packets (no frame is damaged)", "%d frames, %d crc" % (len(fr), d.crc_errors))

    # input errors
    r = run([sys.executable, SEND, "--synth", "--file", csv, "--out", os.path.join(tmp, "x.bin")])
    check(r.returncode != 0, "--synth together with --file is refused")
    r = run([sys.executable, SEND, "--synth", "--out", os.path.join(tmp, "x.bin"), "--port", "COM1"])
    check(r.returncode != 0, "two outputs are refused")
    bad_csv = os.path.join(tmp, "bad.csv")
    with open(bad_csv, "w") as f:
        f.write("0,1,2,x,4\n")
    r = run([sys.executable, SEND, "--file", bad_csv, "--out", os.path.join(tmp, "x.bin")])
    check(r.returncode != 0 and "not a number" in (r.stdout + r.stderr), "a damaged recording is refused with a message", r.stderr)
    wide = os.path.join(tmp, "wide.csv")
    with open(wide, "w") as f:
        f.write("0," + ",".join(["1,2"] * (iqproto.MAX_BINS + 1)) + "\n")
    r = run([sys.executable, SEND, "--file", wide, "--out", os.path.join(tmp, "x.bin")])
    check(r.returncode != 0 and "range bins" in (r.stdout + r.stderr), "more range bins than the firmware stores is refused", r.stderr)


# ------------------------------------------------------------------------------------------------ 5. end to end
def records_of(stdout):
    out = []
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("{") and '"verdict"' in line:
            out.append(json.loads(line))
    return out


class E2E:
    def __init__(self, bin_dir, tmp):
        self.dump = os.path.join(bin_dir, "dump_sim")
        self.replay = os.path.join(bin_dir, "fw_replay")
        self.tmp = tmp
        self.all_records = []

    def available(self):
        return os.path.exists(self.dump) and os.path.exists(self.replay)

    def recording(self, name, parts):
        """parts: [(preset, frames, seed), ...] concatenated into one CSV."""
        path = os.path.join(self.tmp, name + ".csv")
        with open(path, "w") as out:
            for preset, frames, seed in parts:
                r = run([self.dump, "--preset", preset, "--frames", str(frames), "--seed", str(seed)])
                if r.returncode != 0:
                    raise RuntimeError("dump_sim failed: " + r.stderr)
                out.write(r.stdout)
        return path

    def stream(self, name, csv, extra=()):
        out = os.path.join(self.tmp, name + ".bin")
        r = run([sys.executable, SEND, "--file", csv, "--out", out, "--quiet"] + list(extra))
        if r.returncode != 0:
            raise RuntimeError("send_iq_replay failed: " + r.stderr)
        return out

    def play(self, stream, cmds, presses=(), until=1, extra=()):
        args = [self.replay, "--stream", stream]
        for at, text in cmds:
            args += ["--cmd-at", str(at), text]
        for at, name in presses:
            args += ["--press-at", str(at), name]
        args += ["--until-results", str(until)] + list(extra)
        r = run(args)
        recs = records_of(r.stdout)
        self.all_records += recs
        return r, recs

    def one(self, name, csv, extra=(), cmds=None, presses=()):
        cmds = cmds if cmds is not None else [(500, "src serial"), (3500, "scan")]
        r, recs = self.play(self.stream(name, csv, extra), cmds, presses)
        return r, (recs[0] if recs else None)


def test_end_to_end(bin_dir, tmp):
    section("5. end to end: recording -> byte stream -> the real firmware (host build)")
    e = E2E(bin_dir, tmp)
    if not e.available():
        check(False, "binaries dump_sim and fw_replay exist in %s (run `make tools`, or pass --no-e2e)" % bin_dir)
        return e

    # ---- the six trial presets
    expect = {
        "A": ("CANDIDATE_PATTERN", "a person breathing, quiet"),
        "B": ("NO_RELIABLE_PATTERN", "empty"),
        "C": ("INVALID_SCAN", "the sensor is knocked"),
        "D": ("CANDIDATE_PATTERN", "a person with platform vibration"),
        "E": ("CONFOUNDED", "vibration at breathing rate"),
    }
    recs = {}
    for p, (verdict, what) in sorted(expect.items()):
        csv = e.recording("preset_" + p, [(p, 1800, 3)])
        r, rec = e.one("preset_" + p, csv)
        recs[p] = rec
        check(rec is not None and rec["verdict"] == verdict, "preset %s (%s) -> %s" % (p, what, verdict), str(rec and rec["verdict"]) + " " + r.stderr[-200:])
    a = recs["A"]
    if a:
        check(abs(a["bpm"] - 16.8) < 1.5 and abs(a["bin"] - 9) <= 1 and abs(a["range_m"] - 1.85) < 0.3, "A: rate, bin and range as simulated",
              "bpm %s bin %s range %s" % (a["bpm"], a["bin"], a["range_m"]))
        check(a["imu"] == "host" and a["link"]["lost_frames"] == 0 and a["link"]["timestamps"] is True and a["sensitivity"] == "unknown",
              "A: host IMU used, link clean, sensitivity honestly 'unknown' without calibration")
        check(a["evidence"] in ("strong", "moderate", "weak") and "confidence" not in a, "A: evidence is ordinal")
    if recs["B"]:
        check(recs["B"]["result"] == MANDATORY, "B: the negative carries the mandatory sentence")
    if recs["C"]:
        check(recs["C"]["invalid_reason"] == "sensor_motion", "C: rejected because the sensor moved")
    if recs["E"]:
        check(recs["E"]["vibration"] is True, "E: vibration reported")

    # ---- calibration and sensitivity (the message after `calib` is closed with the menu button)
    cal_cmds = [(500, "src serial"), (3000, "calib"), (40000, "scan")]
    cal_press = [(36000, "menu")]
    csv = e.recording("cal_f", [("B", 800, 5), ("F", 1800, 6)])
    r, rec = e.one("cal_f", csv, cmds=cal_cmds, presses=cal_press)
    check("baseline stored" in r.stdout, "calibration on the empty reference stored a baseline", r.stdout[-300:])
    check(rec is not None and rec["verdict"] == "LOW_SENSITIVITY" and rec["sensitivity"] == "low" and rec["baseline_src"] == "calibrated",
          "3x noise after calibration -> LOW_SENSITIVITY, never a plain negative", str(rec and rec["verdict"]))
    if rec:
        check(rec["floor_delta_db"] > 6.0, "the floor rise is reported", str(rec["floor_delta_db"]))
    csv = e.recording("cal_b", [("B", 800, 5), ("B", 1800, 7)])
    r, rec = e.one("cal_b", csv, cmds=cal_cmds, presses=cal_press)
    check(rec is not None and rec["verdict"] == "NO_RELIABLE_PATTERN" and rec["sensitivity"] == "ok" and abs(rec["floor_delta_db"]) < 3.0,
          "empty after calibration: NO_RELIABLE_PATTERN with sensitivity 'ok'", str(rec and (rec["verdict"], rec["sensitivity"])))
    csv = e.recording("cal_a", [("B", 800, 5), ("A", 1800, 8)])
    r, rec = e.one("cal_a", csv, cmds=cal_cmds, presses=cal_press)
    check(rec is not None and rec["verdict"] == "CANDIDATE_PATTERN" and rec["sensitivity"] == "ok", "person after calibration: candidate, sensitivity 'ok'",
          str(rec and rec["verdict"]))

    # ---- a damaged stream must never turn into a negative
    csv = os.path.join(tmp, "preset_A.csv")
    faults = [
        ("drop every 20th frame (5 %)", ["--drop-every", "20"], dict(verdict="INVALID_SCAN", reason="radar_dropout", lost=True)),
        ("drop every 200th frame (0.5 %)", ["--drop-every", "200"], dict(verdict="CANDIDATE_PATTERN", lost=True)),
        ("corrupt every 25th packet", ["--corrupt-every", "25"], dict(verdict="INVALID_SCAN", reason="radar_dropout", crc=True)),
        ("corrupt every 300th packet", ["--corrupt-every", "300"], dict(verdict="CANDIDATE_PATTERN", crc=True)),
        ("duplicate every 10th packet", ["--dup-every", "10"], dict(verdict="CANDIDATE_PATTERN")),
        ("noise between packets", ["--garbage-every", "7"], dict(verdict="CANDIDATE_PATTERN")),
        ("no timestamps", ["--no-stamps"], dict(verdict="CANDIDATE_PATTERN", stamps=False)),
        ("front-end stalls 1.5 s inside a segment", ["--pause-at", "10", "--pause-for", "1.5"], dict(verdict="CANDIDATE_PATTERN", reason="radar_timing", timing=True)),
        ("front-end announces 25 Hz, sends 20 Hz", ["--announce-fs", "25"], dict(verdict="INVALID_SCAN", reason="radar_timing", timing=True)),
    ]
    for name, extra, exp in faults:
        r, rec = e.one("fault", csv, extra)
        ok = rec is not None and rec["verdict"] == exp["verdict"]
        if rec and "reason" in exp:
            ok = ok and rec["invalid_reason"] == exp["reason"]
        if rec and exp.get("lost"):
            ok = ok and rec["link"]["lost_frames"] > 0
        if rec and exp.get("crc"):
            ok = ok and rec["link"]["crc_errors"] > 0
        if rec and exp.get("timing"):
            ok = ok and rec["link"]["timing_flagged"] > 0
        if rec and exp.get("stamps") is False:
            ok = ok and rec["link"]["timestamps"] is False
        check(ok, "fault: %s -> %s" % (name, exp["verdict"]), str(rec and (rec["verdict"], rec["invalid_reason"], rec["link"])))
        check(rec is not None and rec["verdict"] != "NO_RELIABLE_PATTERN", "fault: %s never ends as a negative" % name)

    # ---- the scan log of all of the above passes the validator
    log = os.path.join(tmp, "e2e_scans.jsonl")
    with open(log, "w") as f:
        for rec in e.all_records:
            f.write(json.dumps(rec) + "\n")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = validate_scan_log.main([log, "--quiet", "--min-records", "17", "--ignore-ids"])
    check(rc == 0, "every record of the runs above passes validate_scan_log.py (%d records)" % len(e.all_records), buf.getvalue()[-600:])
    return e


def test_online(bin_dir, tmp):
    """The path of --port / --tcp: a real socket, paced sending, command replies, retries, capture file."""
    section("5b. the live path through a TCP socket (fw_replay --listen)")
    replay = os.path.join(bin_dir, "fw_replay")
    csv = os.path.join(tmp, "preset_A.csv")
    if not (os.path.exists(replay) and os.path.exists(csv)):
        check(False, "fw_replay and a recording exist for the TCP test")
        return
    speed = "16"
    proc = subprocess.Popen([replay, "--listen", "0", "--speed", speed, "--quiet"], stderr=subprocess.PIPE, text=True)
    try:
        line = proc.stderr.readline()
        m = re.search(r":(\d+)", line)
        check(m is not None, "fw_replay announces its port", line)
        if not m:
            return
        cap = os.path.join(tmp, "live_capture.jsonl")
        if os.path.exists(cap):
            os.remove(cap)
        r = run([sys.executable, SEND, "--file", csv, "--tcp", "127.0.0.1:%s" % m.group(1), "--speed", speed, "--cmd", "src serial", "--cmd", "scan",
                 "--capture", cap, "--until-result", "1"], timeout=180)
        check(r.returncode == 0, "send_iq_replay --tcp ends normally after the first scan record", "rc=%d\n%s\n%s" % (r.returncode, r.stdout[-600:], r.stderr[-400:]))
        recs = []
        if os.path.exists(cap):
            with open(cap) as f:
                recs = [json.loads(l) for l in f if l.strip()]
        check(len(recs) == 1 and recs[0]["verdict"] == "CANDIDATE_PATTERN", "the captured record is the candidate", str(recs and recs[0]["verdict"]))
        if recs:
            check(recs[0]["imu"] == "host" and recs[0]["link"]["lost_frames"] == 0, "no frame was lost on the way", str(recs[0]["link"]))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = validate_scan_log.main([cap, "--quiet"])
        check(rc == 0, "the captured record passes the validator", buf.getvalue())
        check("scan RS-0001" in r.stdout, "the monitor printed what the device said", r.stdout[-300:])
    finally:
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    # a refused command is retried, then given up with a non-zero exit code
    proc = subprocess.Popen([replay, "--listen", "0", "--speed", speed, "--quiet"], stderr=subprocess.PIPE, text=True)
    try:
        line = proc.stderr.readline()
        m = re.search(r":(\d+)", line)
        if not m:
            check(False, "second fw_replay announces its port", line)
            return
        r = run([sys.executable, SEND, "--synth", "--seconds", "40", "--tcp", "127.0.0.1:%s" % m.group(1), "--speed", speed,
                 "--cmd", "preset zzz", "--cmd-retries", "2", "--quiet"], timeout=120)
        check(r.returncode == 4 and "refused" in r.stderr, "a command the device keeps refusing ends with exit status 4", "rc=%d %s" % (r.returncode, r.stderr[-300:]))
    finally:
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


# ------------------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin-dir", default=os.environ.get("RS_BIN_DIR", os.path.join(ROOT, "build", "bin")))
    ap.add_argument("--no-e2e", action="store_true", help="skip section 5 (needs the C++ tools)")
    ap.add_argument("--keep", help="copy the generated recordings, streams and logs into this directory")
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="rs_tools_")
    try:
        test_protocol()
        test_diagram(tmp)
        test_validator()
        test_sender(tmp)
        if not args.no_e2e:
            test_end_to_end(args.bin_dir, tmp)
            test_online(args.bin_dir, tmp)
        if args.keep:
            shutil.copytree(tmp, args.keep, dirs_exist_ok=True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("\n%d checks passed, %d failed" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
