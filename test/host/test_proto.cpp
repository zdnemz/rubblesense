// Serial I/Q protocol: encoder/parser round trips, resynchronisation, corruption, fuzz.
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include "rs_proto.h"
#include "rs_simgen.h"
#include "rs_test.h"

using namespace rs;

namespace {

RadarFrame makeFrame(Rng& rng, uint16_t seq, uint8_t nb) {
  RadarFrame f{};
  f.n_bins = nb;
  f.seq = seq;
  f.t_ms = 50u * seq + 123456789u;
  f.saturated = (seq % 7) == 0;
  for (uint8_t b = 0; b < nb; b++) {
    f.iq[b].i = static_cast<int16_t>(static_cast<int>(rng.next() & 0xFFFF) - 32768);
    f.iq[b].q = static_cast<int16_t>(static_cast<int>(rng.next() & 0xFFFF) - 32768);
  }
  return f;
}

bool sameFrame(const RadarFrame& a, const RadarFrame& b) {
  if (a.n_bins != b.n_bins || a.seq != b.seq || a.t_ms != b.t_ms || a.saturated != b.saturated) return false;
  for (uint8_t i = 0; i < a.n_bins; i++)
    if (a.iq[i].i != b.iq[i].i || a.iq[i].q != b.iq[i].q) return false;
  return true;
}

// Feed a byte vector, collect decoded frames.
struct Collector {
  FrameParser p;
  std::vector<RadarFrame> frames;
  int infos = 0;
  void feed(const std::vector<uint8_t>& v) {
    for (uint8_t b : v) {
      FrameParser::Event e = p.push(b);
      while (e != FrameParser::Event::kNone) {
        if (e == FrameParser::Event::kFrame) frames.push_back(p.frame()); else infos++;
        e = p.poll();
      }
    }
  }
};

void append(std::vector<uint8_t>& v, const RadarFrame& f) {
  uint8_t buf[kProtoMaxPacket];
  const size_t n = encodeFrame(f, buf, sizeof(buf));
  v.insert(v.end(), buf, buf + n);
}

}  // namespace

void test_proto() {
  // CRC-16/CCITT-FALSE reference value.
  const char* ref = "123456789";
  CHECK(crc16Ccitt(reinterpret_cast<const uint8_t*>(ref), 9) == 0x29B1);

  Rng rng(42);

  // Flag bits: saturation comes from the frame, the others from the sender; all survive the round trip.
  {
    RadarFrame f = makeFrame(rng, 3, 8);
    f.saturated = false;
    uint8_t buf[kProtoMaxPacket];
    const uint8_t extra = kProtoFlagMotion | kProtoFlagHostImu;
    const size_t n = encodeFrame(f, buf, sizeof(buf), extra);
    CHECK(n > 0 && buf[7] == extra);
    FrameParser p;
    FrameParser::Event e = FrameParser::Event::kNone;
    for (size_t i = 0; i < n; i++) e = p.push(buf[i]);
    CHECK(e == FrameParser::Event::kFrame && p.flags() == extra && !p.frame().saturated);
    f.saturated = true;
    encodeFrame(f, buf, sizeof(buf), kProtoFlagDropped);
    CHECK(buf[7] == (kProtoFlagSaturated | kProtoFlagDropped));
    CHECK(kProtoFlagMotion == 2 && kProtoFlagDropped == 4 && kProtoFlagHostImu == 8);   // wire format: do not renumber
  }

  // Golden packets: tools/iqproto.py (the PC side of the link) must produce exactly these bytes, and
  // test_tools.py asserts the same strings there.
  {
    auto hexOf = [](const uint8_t* b, size_t n) {
      static const char* d = "0123456789abcdef";
      std::string s;
      for (size_t i = 0; i < n; i++) { s += d[b[i] >> 4]; s += d[b[i] & 15]; }
      return s;
    };
    uint8_t buf[kProtoMaxPacket];
    RadarFrame f{};
    f.n_bins = 3; f.seq = 0x1234; f.t_ms = 0x0A0B0C0Du; f.saturated = false;
    f.iq[0].i = 1;     f.iq[0].q = -1;
    f.iq[1].i = 256;   f.iq[1].q = -256;
    f.iq[2].i = 32767; f.iq[2].q = -32768;
    size_t n = encodeFrame(f, buf, sizeof(buf), kProtoFlagMotion | kProtoFlagHostImu);
    CHECK(hexOf(buf, n) == "a55a01013412030a0d0c0b0a0100ffff000100ffff7f0080073a");
    f.saturated = true; f.seq = 0xFFFF; f.t_ms = 0;
    n = encodeFrame(f, buf, sizeof(buf));
    CHECK(hexOf(buf, n) == "a55a0101ffff0301000000000100ffff000100ffff7f008022ef");
    RadarInfo in{};
    in.name = "x"; in.frame_rate_hz = 20.0f; in.range_start_m = 0.5f; in.range_step_m = 0.15f;
    in.wavelength_mm = 5.0f; in.n_bins = 4;
    n = encodeInfo(in, buf, sizeof(buf));
    CHECK(hexOf(buf, n) == "a55a010200000000000000000000a0410000003f9a99193e0000a04004809e");
  }

  // Frame round trip over every legal bin count, including extreme sample values and seq wrap.
  for (uint8_t nb = 1; nb <= kMaxBins; nb++) {
    RadarFrame f = makeFrame(rng, static_cast<uint16_t>(65530u + nb), nb);
    f.iq[0].i = -32768; f.iq[0].q = 32767;
    uint8_t buf[kProtoMaxPacket];
    const size_t n = encodeFrame(f, buf, sizeof(buf));
    CHECK(n == kProtoHeader + 4u * nb + 2u);
    FrameParser p;
    FrameParser::Event e = FrameParser::Event::kNone;
    for (size_t i = 0; i < n; i++) e = p.push(buf[i]);
    CHECK(e == FrameParser::Event::kFrame);
    CHECK(sameFrame(f, p.frame()));
  }

  // Encoder refuses bad arguments.
  {
    RadarFrame f = makeFrame(rng, 1, 8);
    uint8_t small[10];
    CHECK(encodeFrame(f, small, sizeof(small)) == 0);
    f.n_bins = 0;
    uint8_t buf[kProtoMaxPacket];
    CHECK(encodeFrame(f, buf, sizeof(buf)) == 0);
    f.n_bins = kMaxBins + 1;
    CHECK(encodeFrame(f, buf, sizeof(buf)) == 0);
  }

  // Info packet round trip.
  {
    RadarInfo in{};
    in.name = "x"; in.frame_rate_hz = 20.0f; in.range_start_m = 0.45f; in.range_step_m = 0.075f;
    in.wavelength_mm = 125.0f; in.n_bins = 16;
    uint8_t buf[64];
    const size_t n = encodeInfo(in, buf, sizeof(buf));
    CHECK(n == kProtoHeader + kProtoInfoPayload + 2u);
    FrameParser p;
    FrameParser::Event e = FrameParser::Event::kNone;
    for (size_t i = 0; i < n; i++) e = p.push(buf[i]);
    CHECK(e == FrameParser::Event::kInfo);
    CHECK_NEAR(p.info().frame_rate_hz, 20.0, 0);
    CHECK_NEAR(p.info().range_step_m, 0.075, 1e-7);
    CHECK_NEAR(p.info().range_start_m, 0.45, 1e-7);
    CHECK_NEAR(p.info().wavelength_mm, 125.0, 0);
    CHECK(p.info().n_bins == 16);
  }

  // A well-formed but insane info packet (NaN rate) is rejected and does not poison the stream.
  {
    RadarInfo bad{};
    bad.name = "bad"; bad.frame_rate_hz = NAN; bad.range_start_m = 0.4f; bad.range_step_m = 0.07f;
    bad.wavelength_mm = 125.0f; bad.n_bins = 16;
    std::vector<uint8_t> s(64);
    s.resize(encodeInfo(bad, s.data(), s.size()));
    RadarFrame f = makeFrame(rng, 5, 16);
    append(s, f);
    Collector c; c.feed(s);
    CHECK(c.infos == 0 && c.p.badPackets() == 1);
    CHECK(c.frames.size() == 1 && sameFrame(c.frames[0], f));
  }

  // Back-to-back stream with a garbage prefix; payload bytes that look like sync words.
  {
    std::vector<uint8_t> s = {0x00, 0xA5, 0xA5, 0x5A, 0x13, 0xFF, 0xA5};
    std::vector<RadarFrame> sent;
    for (uint16_t i = 0; i < 100; i++) {
      RadarFrame f = makeFrame(rng, i, 16);
      if (i % 5 == 0) { f.iq[3].i = 0x5AA5; f.iq[3].q = 0x5AA5; f.iq[4].i = static_cast<int16_t>(0xA55A); }
      sent.push_back(f);
      append(s, f);
    }
    Collector c; c.feed(s);
    CHECK(c.frames.size() == 100);
    bool same = c.frames.size() == 100;
    for (size_t i = 0; same && i < 100; i++) same = sameFrame(c.frames[i], sent[i]);
    CHECK(same);
    CHECK(c.p.crcErrors() == 0);
    CHECK(c.p.bytesSkipped() == 7);
  }

  // One corrupted byte (payload, header, or length field) loses exactly that packet.
  {
    for (int where = 0; where < 4; where++) {
      std::vector<uint8_t> s;
      std::vector<size_t> start;
      for (uint16_t i = 0; i < 100; i++) { start.push_back(s.size()); append(s, makeFrame(rng, i, 16)); }
      const size_t base = start[50];
      if (where == 0) s[base + 20] ^= 0x40;            // payload
      if (where == 1) s[base + 4] ^= 0x01;             // seq (CRC catches it)
      if (where == 2) s[base + 6] = 24;                // nbins: packet now claims to be longer
      if (where == 3) s[base + 1] = 0x00;              // sync byte destroyed
      Collector c; c.feed(s);
      CHECK(c.frames.size() == 99);
      bool has49 = false, has50 = false, has51 = false;
      for (const RadarFrame& f : c.frames) { has49 |= f.seq == 49; has50 |= f.seq == 50; has51 |= f.seq == 51; }
      CHECK(has49 && !has50 && has51);
    }
  }

  // Truncated packet followed by good ones.
  {
    std::vector<uint8_t> a, b;
    append(a, makeFrame(rng, 1, 16));
    RadarFrame fb = makeFrame(rng, 2, 16);
    append(b, fb);
    std::vector<uint8_t> s(a.begin(), a.begin() + 30);
    s.insert(s.end(), b.begin(), b.end());
    Collector c; c.feed(s);
    CHECK(c.frames.size() == 1 && sameFrame(c.frames[0], fb));
  }

  // Burst of random garbage between packets (a noisy UART): packets either side survive.
  {
    std::vector<uint8_t> s;
    for (uint16_t i = 0; i < 20; i++) append(s, makeFrame(rng, i, 16));
    for (int i = 0; i < 300; i++) s.push_back(static_cast<uint8_t>(rng.next()));
    for (uint16_t i = 20; i < 40; i++) append(s, makeFrame(rng, i, 16));
    Collector c; c.feed(s);
    int pre = 0, post = 0;
    for (const RadarFrame& f : c.frames) { if (f.seq < 20) pre++; else post++; }
    CHECK(pre == 20);
    CHECK(post >= 19);                                   // at most the packet right after the burst can be lost
  }

  // Fuzz: random bytes never crash and (CRC-16 + header checks) essentially never decode.
  {
    FrameParser p;
    uint32_t events = 0;
    for (int i = 0; i < 2000000; i++) {
      FrameParser::Event e = p.push(static_cast<uint8_t>(rng.next() >> 11));
      while (e != FrameParser::Event::kNone) { events++; e = p.poll(); }
    }
    CHECK(events == 0);
  }

  // Sequence gap helper.
  CHECK(seqGap(10, 11) == 0);
  CHECK(seqGap(10, 13) == 2);
  CHECK(seqGap(65535, 0) == 0);
  CHECK(seqGap(65534, 1) == 2);
}
