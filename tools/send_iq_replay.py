#!/usr/bin/env python3
"""Feed the firmware with I/Q frames over the serial protocol: recording replay and hardware-in-the-loop.

Examples
  # simulated scene A (person, quiet) from the C++ simulator, into a Wokwi simulation (VS Code, wokwi.toml
  # opens an RFC2217 server on port 4000), select the serial source, start a scan, keep the JSON result:
  tools/dump_sim --preset A --frames 1800 --out a.csv
  python3 tools/send_iq_replay.py --file a.csv --port rfc2217://localhost:4000 \\
          --cmd "src serial" --cmd scan --capture scans.jsonl --until-result 1

  # a real ESP32 on USB (I/Q on UART0, the "esp32dev_usb_iq" build):
  python3 tools/send_iq_replay.py --file a.csv --port /dev/ttyUSB0 --baud 115200 --cmd "src serial" --cmd scan

  # no hardware: write the byte stream to a file (for test/host/fw_replay or for inspection)
  python3 tools/send_iq_replay.py --file a.csv --out a.bin

  # a purely synthetic breathing target (NOT the firmware's simulator: a plain model, see synth())
  python3 tools/send_iq_replay.py --synth --bpm 16 --amp 30 --sigma 8 --seconds 90 --out s.bin

Recordings: CSV (as written by dump_sim: '#' metadata lines, then flags,i0,q0,i1,q1,... per frame) or
NumPy .npz with a complex array `iq` (frames x bins) and optional `flags`, `fs_hz`, `range_start_m`,
`range_step_m`, `wavelength_mm`.

The frames carry the IMU verdict of a sensor mounted with the radar (protocol flag "host IMU"), which a
replay must do: the device's own IMU is not on the recorded radar. Use --no-host-imu to send frames
without it (the device then judges stability with its own IMU, which is only meaningful on a live radar).

Fault injection (--drop-every, --corrupt-every, --dup-every, --garbage-every, --pause-at, --announce-fs)
exercises the link-quality and time-base checks of the firmware.

Needs pyserial only for --port (pip install pyserial). Sending is paced to real time (--speed 1).
"""
import argparse
import json
import math
import os
import random
import re
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import iqproto  # noqa: E402


# ----------------------------------------------------------------------------- sources
class Recording:
    def __init__(self, frames, flags, fs_hz, range_start_m, range_step_m, wavelength_mm, has_flags, label):
        self.frames = frames                     # list of [(i, q), ...]
        self.flags = flags                       # list of int (protocol flag bits from the recording)
        self.fs_hz = fs_hz
        self.range_start_m = range_start_m
        self.range_step_m = range_step_m
        self.wavelength_mm = wavelength_mm
        self.has_flags = has_flags
        self.label = label

    @property
    def n_bins(self):
        return len(self.frames[0])


def load_csv(path):
    meta, frames, flags = {}, [], []
    with open(path, "r", encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                for m in re.finditer(r"(\w+)=(\S+)", line):
                    meta[m.group(1)] = m.group(2)
                continue
            try:
                v = [int(x) for x in line.split(",")]
            except ValueError:
                raise SystemExit("%s:%d: not a number in a frame line" % (path, ln))
            if len(v) < 3 or (len(v) - 1) % 2:
                raise SystemExit("%s:%d: a frame line is 'flags,i0,q0,i1,q1,...'" % (path, ln))
            flags.append(v[0])
            frames.append([(v[1 + 2 * k], v[2 + 2 * k]) for k in range((len(v) - 1) // 2)])
    if not frames:
        raise SystemExit("%s holds no frames" % path)
    if len({len(fr) for fr in frames}) != 1:
        raise SystemExit("%s: frames have different bin counts" % path)
    return Recording(frames, flags, float(meta.get("fs_hz", 20.0)), float(meta.get("range_start_m", 0.5)),
                     float(meta.get("range_step_m", 0.15)), float(meta.get("wavelength_mm", 125.0)), True,
                     "%s (preset %s)" % (os.path.basename(path), meta.get("preset", "?")))


def load_npz(path):
    try:
        import numpy as np
    except ImportError:
        raise SystemExit("reading .npz needs numpy (pip install numpy); or convert the recording to CSV")
    z = np.load(path)
    iq = z["iq"]
    if iq.ndim != 2:
        raise SystemExit("'iq' must be a (frames x bins) array")
    scale = 1.0
    peak = float(np.max(np.abs(iq))) if iq.size else 0.0
    if peak > 32767:
        scale = 32767.0 / peak
        print("note: samples were scaled by %.4g to fit int16" % scale, file=sys.stderr)
    frames = [[(int(round(c.real * scale)), int(round(c.imag * scale))) for c in row] for row in iq]
    has_flags = "flags" in z.files
    flags = [int(x) for x in z["flags"]] if has_flags else [0] * len(frames)
    g = lambda k, d: float(z[k]) if k in z.files else d
    return Recording(frames, flags, g("fs_hz", 20.0), g("range_start_m", 0.5), g("range_step_m", 0.15),
                     g("wavelength_mm", 125.0), has_flags, os.path.basename(path))


def synth(seconds, fs, bins, target_bin, amp, bpm, sigma, seed, wavelength_mm=125.0):
    """A plain model of one weak breathing reflector in static clutter and white noise (counts)."""
    rng = random.Random(seed)
    n = int(round(seconds * fs))
    leak, wall, wall_bin, clutter = 12000.0, 9000.0, 4, 1500.0
    stat = []
    for b in range(bins):
        a = leak if b < 2 else (wall if b == wall_bin else (clutter * math.exp(-0.15 * (b - wall_bin)) if b > wall_bin else 0.0))
        ph = rng.uniform(0, 2 * math.pi)
        stat.append(complex(a * math.cos(ph), a * math.sin(ph)))
    f_resp = bpm / 60.0
    k = 4.0 * math.pi / wavelength_mm
    frames = []
    for i in range(n):
        t = i / fs
        x_mm = 4.0 * math.sin(2 * math.pi * f_resp * t)                     # chest displacement, 4 mm peak
        tgt = amp * complex(math.cos(k * x_mm + 0.7), math.sin(k * x_mm + 0.7))
        row = []
        for b in range(bins):
            z = stat[b]
            if b == target_bin:
                z += tgt
            elif abs(b - target_bin) == 1:
                z += 0.35 * tgt
            z += complex(rng.gauss(0, sigma), rng.gauss(0, sigma))
            row.append((max(-32768, min(32767, int(round(z.real)))), max(-32768, min(32767, int(round(z.imag))))))
        frames.append(row)
    return Recording(frames, [0] * n, fs, 0.5, 0.15, wavelength_mm, False,
                     "synthetic: %.1f breaths/min, amplitude %.0f, noise %.0f" % (bpm, amp, sigma))


# ----------------------------------------------------------------------------- transports
class FileOut:
    bidirectional = False

    def __init__(self, path):
        self.f = sys.stdout.buffer if path == "-" else open(path, "wb")

    def write(self, b):
        self.f.write(b)

    def read(self):
        return b""

    def close(self):
        if self.f is not sys.stdout.buffer:
            self.f.close()


class SerialOut:
    bidirectional = True

    def __init__(self, url, baud):
        try:
            import serial
        except ImportError:
            raise SystemExit("--port needs pyserial: pip install pyserial")
        self.s = serial.serial_for_url(url, baudrate=baud, timeout=0.02, write_timeout=5)

    def write(self, b):
        self.s.write(b)

    def read(self):
        return self.s.read(max(1, self.s.in_waiting))

    def close(self):
        self.s.close()


class TcpOut:
    bidirectional = True

    def __init__(self, hostport):
        import socket
        host, port = hostport.rsplit(":", 1)
        self.s = socket.create_connection((host, int(port)), timeout=5)
        self.s.settimeout(0.02)

    def write(self, b):
        self.s.sendall(b)

    def read(self):
        import socket
        try:
            data = self.s.recv(4096)
        except socket.timeout:
            return b""
        if not data:
            raise EOFError("the device closed the connection")
        return data

    def close(self):
        self.s.close()


class Monitor(threading.Thread):
    """Reads what the device prints: shows it, keeps the JSON scan records and the replies to commands."""

    def __init__(self, link, capture_path, quiet):
        super().__init__(daemon=True)
        self.link, self.quiet = link, quiet
        self.stop_flag = False
        self.buf = b""
        self.results = []
        self.replies = []
        self.lock = threading.Lock()
        self.cap = open(capture_path, "a", encoding="utf-8") if capture_path else None

    def run(self):
        while not self.stop_flag:
            try:
                data = self.link.read()
            except Exception:
                break
            if not data:
                continue
            self.buf += data
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                self.on_line(line.decode("utf-8", "replace").strip())

    def on_line(self, line):
        if not line:
            return
        if line.startswith("{") and line.endswith("}"):
            try:
                rec = json.loads(line)
            except ValueError:
                rec = None
            if rec is not None and "verdict" in rec:
                with self.lock:
                    self.results.append(rec)
                if self.cap:
                    self.cap.write(line + "\n")
                    self.cap.flush()
                if not self.quiet:
                    print("device: scan %s -> %s (invalid reason: %s, evidence: %s)" % (
                        rec.get("scan_id"), rec.get("verdict"), rec.get("invalid_reason"), rec.get("evidence")))
                return
        if line == "OK" or line.startswith("ERR"):
            with self.lock:
                self.replies.append(line)
        if not self.quiet:
            print("device:", line)

    def count(self):
        with self.lock:
            return len(self.results)

    def pop_reply(self):
        with self.lock:
            return self.replies.pop(0) if self.replies else None


class Commander:
    """Console commands for the device, sent when the stream reaches their time.

    Live (serial / tcp): each command waits for the device's reply; a refused one ("ERR ...", typically a
    scan before the front-end is announced) is retried every second of stream time. Into a file there is
    no reply to wait for, so the lines are embedded blindly, `gap` seconds apart. Times are stream time,
    so --speed does not change the behaviour."""

    def __init__(self, cmds, first_at, gap, retries, monitor):
        self.queue = list(cmds)
        self.next_at = first_at
        self.gap, self.retries, self.mon = gap, retries, monitor
        self.cur = None
        self.failed = []

    @staticmethod
    def _line(text):
        # the leading newline flushes whatever the device's console had collected from binary noise
        return ("\n" + text + "\n").encode("ascii")

    def pending(self):
        return bool(self.queue) or self.cur is not None

    def poll(self, now):
        if self.cur is None:
            if not self.queue or now < self.next_at:
                return b""
            text = self.queue.pop(0)
            if self.mon is None:                                   # blind: into a file
                self.next_at = now + self.gap
                return self._line(text)
            self.cur = {"text": text, "sent": now, "tries": 1, "retry_at": None}
            return self._line(text)
        cur = self.cur
        rep = self.mon.pop_reply()
        if rep is not None:
            if rep == "OK":
                self.cur, self.next_at = None, now + 0.3
            elif cur["tries"] < self.retries:
                cur["retry_at"] = now + 1.0
            else:
                print("command '%s' was refused %d times: %s" % (cur["text"], cur["tries"], rep), file=sys.stderr)
                self.failed.append(cur["text"])
                self.cur, self.next_at = None, now + 0.3
            return b""
        if cur["retry_at"] is not None and now >= cur["retry_at"]:
            cur["tries"] += 1
            cur["retry_at"], cur["sent"] = None, now
            return self._line(cur["text"])
        if cur["retry_at"] is None and now - cur["sent"] > 5.0:
            print("no reply to '%s' within 5 s of stream time" % cur["text"], file=sys.stderr)
            self.failed.append(cur["text"])
            self.cur, self.next_at = None, now + 0.3
        return b""


# ----------------------------------------------------------------------------- the stream
def build_stream(rec, args, rng):
    """Yield (due_seconds, bytes, kind) for the whole stream; due times are stream time (before --speed)."""
    fs = rec.fs_hz
    announced = args.announce_fs if args.announce_fs else fs
    info = iqproto.encode_info(announced, rec.range_start_m, rec.range_step_m, rec.wavelength_mm, rec.n_bins)
    host_imu = rec.has_flags if args.host_imu is None else args.host_imu
    n_total = len(rec.frames) * (args.loops if args.loops else 1) if not args.forever else None
    seq, idx, next_info, pause_total, paused = 0, 0, 0.0, 0.0, False
    yield 0.0, info, "info"
    next_info = args.info_period
    while n_total is None or idx < n_total:
        t = idx / fs
        if args.pause_at is not None and not paused and t >= args.pause_at:
            paused = True
            pause_total += args.pause_for
        stream_t = t + pause_total
        if stream_t >= next_info:
            yield stream_t, info, "info"
            next_info += args.info_period
        k = idx % len(rec.frames)
        flags = rec.flags[k] & (iqproto.FLAG_SATURATED | iqproto.FLAG_MOTION)
        if host_imu:
            flags |= iqproto.FLAG_HOST_IMU
        t_ms = 0 if args.no_stamps else int(round(stream_t * 1000.0))
        pkt = iqproto.encode_frame(seq, t_ms, rec.frames[k], flags)
        n = idx + 1
        out = bytearray()
        if args.drop_every and n % args.drop_every == 0:
            pkt = b""
        if pkt:
            if args.garbage_every and n % args.garbage_every == 0:
                out += bytes(rng.randrange(256) for _ in range(rng.randint(1, 40)))
            body = bytearray(pkt)
            if args.corrupt_every and n % args.corrupt_every == 0:
                body[rng.randrange(2, len(body))] ^= 1 << rng.randrange(8)
            out += body
            if args.dup_every and n % args.dup_every == 0:
                out += bytes(body)
        if out:
            yield stream_t, bytes(out), "frame"
        seq = (seq + 1) & 0xFFFF
        idx += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_argument_group("source")
    src.add_argument("--file", help="recording: .csv (dump_sim format) or .npz")
    src.add_argument("--synth", action="store_true", help="generate a synthetic breathing scene instead")
    src.add_argument("--seconds", type=float, default=90.0, help="--synth: length (default 90 s)")
    src.add_argument("--bpm", type=float, default=16.8, help="--synth: breaths per minute")
    src.add_argument("--amp", type=float, default=30.0, help="--synth: target amplitude in counts")
    src.add_argument("--sigma", type=float, default=8.0, help="--synth: noise, counts rms per I and Q")
    src.add_argument("--bins", type=int, default=16, help="--synth: range bins")
    src.add_argument("--target-bin", type=int, default=9, help="--synth: target range bin")
    src.add_argument("--seed", type=int, default=1)
    src.add_argument("--fs", type=float, help="frame rate (overrides the recording's)")
    out = ap.add_argument_group("output")
    out.add_argument("--port", help="serial port or pyserial URL (e.g. /dev/ttyUSB0, COM5, rfc2217://localhost:4000)")
    out.add_argument("--baud", type=int, default=115200)
    out.add_argument("--tcp", help="HOST:PORT of a raw TCP serial bridge")
    out.add_argument("--out", help="write the byte stream to this file ('-' = stdout) instead of a device")
    out.add_argument("--speed", type=float, default=None, help="pacing: 1 = real time (default for a device), 0 = as fast as possible (default for --out)")
    st = ap.add_argument_group("stream")
    st.add_argument("--host-imu", dest="host_imu", action="store_true", default=None, help="frames vouch for the IMU verdict (default: on if the recording has flags)")
    st.add_argument("--no-host-imu", dest="host_imu", action="store_false")
    st.add_argument("--no-stamps", action="store_true", help="send t_ms = 0 (a front-end without timestamps)")
    st.add_argument("--info-period", type=float, default=1.0, help="seconds between info packets (the device needs one at least every 2 s)")
    st.add_argument("--loops", type=int, default=1, help="play the recording this many times")
    st.add_argument("--forever", action="store_true", help="loop until interrupted")
    flt = ap.add_argument_group("faults (all off by default)")
    flt.add_argument("--drop-every", type=int, default=0)
    flt.add_argument("--corrupt-every", type=int, default=0)
    flt.add_argument("--dup-every", type=int, default=0)
    flt.add_argument("--garbage-every", type=int, default=0)
    flt.add_argument("--pause-at", type=float, help="stream time (s) at which the front-end stops without a sequence gap")
    flt.add_argument("--pause-for", type=float, default=1.5)
    flt.add_argument("--announce-fs", type=float, help="announce this frame rate while sending at the recording's")
    dev = ap.add_argument_group("device interaction")
    dev.add_argument("--cmd", action="append", default=[], help="console command, e.g. 'src serial', 'calib', 'scan' (repeatable, sent in order)")
    dev.add_argument("--cmd-delay", type=float, default=3.0, help="stream seconds before the first command (default 3)")
    dev.add_argument("--cmd-gap", type=float, default=2.5, help="--out only: stream seconds between embedded commands (no replies to wait for)")
    dev.add_argument("--cmd-retries", type=int, default=6, help="live: how often a refused command is repeated (1 s apart)")
    dev.add_argument("--capture", help="append the JSON scan records the device prints to this file (serial / tcp)")
    dev.add_argument("--until-result", type=int, default=0, help="stop after this many scan records (serial / tcp)")
    dev.add_argument("--timeout", type=float, default=0.0, help="stop after this many seconds of stream time")
    dev.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if bool(args.file) == bool(args.synth):
        ap.error("give exactly one of --file / --synth")
    if sum(bool(x) for x in (args.port, args.tcp, args.out)) != 1:
        ap.error("give exactly one of --port / --tcp / --out")

    rec = load_npz(args.file) if args.file and args.file.endswith(".npz") else \
        (load_csv(args.file) if args.file else synth(args.seconds, args.fs or 20.0, args.bins, args.target_bin, args.amp, args.bpm, args.sigma, args.seed))
    if args.fs:
        rec.fs_hz = args.fs
    if not 1 <= rec.n_bins <= iqproto.MAX_BINS:
        raise SystemExit("the firmware handles 1..%d range bins, the recording has %d" % (iqproto.MAX_BINS, rec.n_bins))
    if rec.fs_hz <= 0:
        raise SystemExit("frame rate must be positive")

    link = FileOut(args.out) if args.out else (SerialOut(args.port, args.baud) if args.port else TcpOut(args.tcp))
    speed = args.speed if args.speed is not None else (0.0 if args.out else 1.0)
    rng = random.Random(args.seed)
    mon = None
    if link.bidirectional:
        mon = Monitor(link, args.capture, args.quiet)
        mon.start()
    if not args.quiet:
        print("replaying %s: %d frames x %d bins at %.2f Hz%s" % (rec.label, len(rec.frames), rec.n_bins, rec.fs_hz,
              ", paced x%.2f" % speed if speed else ", unpaced"), file=sys.stderr)

    cmdr = Commander(args.cmd, args.cmd_delay, args.cmd_gap, args.cmd_retries, mon)
    t_wall0 = time.monotonic()
    sent_frames = 0
    code = 0
    try:
        for due, data, kind in build_stream(rec, args, rng):
            if speed:
                delay = t_wall0 + due / speed - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            # a console line must land between two packets, never inside one
            line = cmdr.poll(due)
            if line:
                link.write(line)
            link.write(data)
            if kind == "frame":
                sent_frames += 1
            if mon and args.until_result and mon.count() >= args.until_result:
                break
            if args.timeout and due >= args.timeout:
                code = 3
                break
        if mon:
            if args.until_result and mon.count() < args.until_result:
                code = 3 if code == 0 else code
                print("stream ended before %d scan record(s) arrived (got %d)" % (args.until_result, mon.count()), file=sys.stderr)
            time.sleep(0.5)                   # let the last lines arrive
        if cmdr.failed and code == 0:
            code = 4
        elif cmdr.pending() and mon is None:
            print("note: %d command(s) did not fit into the stream (it ended first)" % (len(cmdr.queue) + (1 if cmdr.cur else 0)), file=sys.stderr)
    except KeyboardInterrupt:
        pass
    except OSError as e:                      # the device went away (closed socket, unplugged USB, ...)
        print("link error: %s" % e, file=sys.stderr)
        code = 5
    finally:
        if mon:
            mon.stop_flag = True
        try:
            link.close()
        except OSError:
            pass
    if not args.quiet:
        print("sent %d frame packets" % sent_frames, file=sys.stderr)
    return code


if __name__ == "__main__":
    sys.exit(main())
