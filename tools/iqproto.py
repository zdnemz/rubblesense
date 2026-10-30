"""RubbleSense serial I/Q protocol v1 (see lib/rs_core/rs_proto.h) - Python side.

Used by send_iq_replay.py to feed the firmware from a PC (recording replay, hardware-in-the-loop
with a Wokwi simulation or a real ESP32). Everything is little-endian. One packet:

    off size field
      0   1  0xA5   sync 0                       8   4  t_ms  front-end timestamp, ms
      1   1  0x5A   sync 1                      12   N  payload (frame: nbins x int16 I, int16 Q;
      2   1  version (1)                                 info: float32 frame_rate_hz, range_start_m,
      3   1  type 1 = frame, 2 = info                          range_step_m, wavelength_mm; uint8 n_bins)
      4   2  seq (frames), 0 for info          12+N 2  CRC-16/CCITT-FALSE over bytes [2, 12+N)
      6   1  nbins (frames), 0 for info
      7   1  flags: bit0 saturated, bit1 motion (the sender's IMU saw the sensor move),
                    bit2 dropped (hold of the previous frame), bit3 host IMU (the sender vouches that
                    bit1 is valid, i.e. its IMU is mounted with the radar)

The encoders are checked against packets produced by the C++ encoder (GOLDEN below; the same hex strings
are asserted in test/host/test_proto.cpp), so the two sides cannot drift apart silently.
"""
import struct

SYNC0, SYNC1 = 0xA5, 0x5A
VERSION = 1
TYPE_FRAME, TYPE_INFO = 1, 2
HEADER = 12
INFO_PAYLOAD = 17
MAX_BINS = 24                      # kMaxBins in lib/rs_core/rs_config.h (the firmware rejects more)

FLAG_SATURATED = 1 << 0
FLAG_MOTION = 1 << 1
FLAG_DROPPED = 1 << 2
FLAG_HOST_IMU = 1 << 3

# Packets produced by the C++ encoder (encodeFrame / encodeInfo) for these inputs. test_proto.cpp
# asserts the same strings, test_tools.py asserts that this module reproduces them.
GOLDEN = {
    # 3 bins (1,-1) (256,-256) (32767,-32768), seq 0x1234, t_ms 0x0A0B0C0D, flags motion|host_imu
    "frame": "a55a01013412030a0d0c0b0a0100ffff000100ffff7f0080073a",
    # same bins, saturated, seq 0xFFFF, t_ms 0, no extra flags
    "frame_sat_wrap": "a55a0101ffff0301000000000100ffff000100ffff7f008022ef",
    # 20 Hz, range start 0.5 m, step 0.15 m, wavelength 5 mm, 4 bins
    "info": "a55a010200000000000000000000a0410000003f9a99193e0000a04004809e",
}


def crc16_ccitt(data, crc=0xFFFF):
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _packet(ptype, seq, nbins, flags, t_ms, payload):
    head = struct.pack("<BBBBHBBI", SYNC0, SYNC1, VERSION, ptype, seq & 0xFFFF, nbins, flags & 0xFF, t_ms & 0xFFFFFFFF)
    body = head + payload
    return body + struct.pack("<H", crc16_ccitt(body[2:]))


def encode_frame(seq, t_ms, iq, flags=0):
    """iq: sequence of (i, q) int16 pairs, one per range bin."""
    n = len(iq)
    if n == 0 or n > MAX_BINS:
        raise ValueError("a frame needs 1..%d range bins, got %d" % (MAX_BINS, n))
    payload = b"".join(struct.pack("<hh", int(i), int(q)) for i, q in iq)
    return _packet(TYPE_FRAME, seq, n, flags, t_ms, payload)


def encode_info(frame_rate_hz, range_start_m, range_step_m, wavelength_mm, n_bins):
    if n_bins == 0 or n_bins > MAX_BINS:
        raise ValueError("n_bins must be 1..%d" % MAX_BINS)
    payload = struct.pack("<ffffB", frame_rate_hz, range_start_m, range_step_m, wavelength_mm, n_bins)
    return _packet(TYPE_INFO, 0, 0, 0, 0, payload)


class Decoder:
    """Byte-wise decoder with resynchronisation, the same contract as rs::FrameParser: it returns every
    CRC-valid packet and counts what it had to throw away. Used for checking recordings and in tests."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.bad_packets = 0
        self.skipped = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            pkt = self._next()
            if pkt is None:
                break
            out.append(pkt)
        return out

    def _next(self):
        b = self.buf
        while True:
            # find a sync pair
            i = 0
            while i + 1 < len(b) and not (b[i] == SYNC0 and b[i + 1] == SYNC1):
                i += 1
            if i + 1 >= len(b):
                keep = 1 if b and b[-1] == SYNC0 else 0
                self.skipped += len(b) - keep
                del b[:len(b) - keep]
                return None
            if i:
                self.skipped += i
                del b[:i]
            if len(b) < HEADER:
                return None
            ver, ptype, seq, nbins, flags, t_ms = struct.unpack_from("<BBHBBI", b, 2)
            if ver != VERSION or ptype not in (TYPE_FRAME, TYPE_INFO) or \
                    (ptype == TYPE_FRAME and not 1 <= nbins <= MAX_BINS) or (ptype == TYPE_INFO and nbins != 0):
                self.bad_packets += 1
                del b[:1]
                self.skipped += 1
                continue
            n = 4 * nbins if ptype == TYPE_FRAME else INFO_PAYLOAD
            total = HEADER + n + 2
            if len(b) < total:
                return None
            if crc16_ccitt(bytes(b[2:total - 2])) != struct.unpack_from("<H", b, total - 2)[0]:
                self.crc_errors += 1
                del b[:1]
                self.skipped += 1
                continue
            payload = bytes(b[HEADER:HEADER + n])
            del b[:total]
            if ptype == TYPE_FRAME:
                iq = [struct.unpack_from("<hh", payload, 4 * k) for k in range(nbins)]
                return {"type": "frame", "seq": seq, "t_ms": t_ms, "flags": flags, "iq": iq}
            rate, start, step, wl, nb = struct.unpack("<ffffB", payload)
            return {"type": "info", "frame_rate_hz": rate, "range_start_m": start, "range_step_m": step,
                    "wavelength_mm": wl, "n_bins": nb}
