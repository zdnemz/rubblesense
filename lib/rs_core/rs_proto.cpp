#include "rs_proto.h"
#include <math.h>
#include <string.h>

namespace rs {

namespace {

inline void wr16(uint8_t* p, uint16_t v) { p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8); }
inline void wr32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);       p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16); p[3] = static_cast<uint8_t>(v >> 24);
}
inline void wrf(uint8_t* p, float f) { uint32_t u; memcpy(&u, &f, 4); wr32(p, u); }
inline uint16_t rd16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t rd32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline float rdf(const uint8_t* p) { const uint32_t u = rd32(p); float f; memcpy(&f, &u, 4); return f; }

void writeHeader(uint8_t* out, uint8_t type, uint16_t seq, uint8_t nbins, uint8_t flags, uint32_t t_ms) {
  out[0] = kProtoSync0;
  out[1] = kProtoSync1;
  out[2] = kProtoVersion;
  out[3] = type;
  wr16(out + 4, seq);
  out[6] = nbins;
  out[7] = flags;
  wr32(out + 8, t_ms);
}

}  // namespace

uint16_t crc16Ccitt(const uint8_t* data, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; i++) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

size_t encodeFrame(const RadarFrame& f, uint8_t* out, size_t cap, uint8_t extra_flags) {
  if (f.n_bins == 0 || f.n_bins > kMaxBins) return 0;
  const size_t total = kProtoHeader + 4u * f.n_bins + 2u;
  if (cap < total) return 0;
  const uint8_t flags = static_cast<uint8_t>(extra_flags | (f.saturated ? kProtoFlagSaturated : 0));
  writeHeader(out, kProtoTypeFrame, f.seq, f.n_bins, flags, f.t_ms);
  uint8_t* p = out + kProtoHeader;
  for (uint8_t b = 0; b < f.n_bins; b++, p += 4) {
    wr16(p,     static_cast<uint16_t>(f.iq[b].i));
    wr16(p + 2, static_cast<uint16_t>(f.iq[b].q));
  }
  wr16(p, crc16Ccitt(out + 2, total - 4));
  return total;
}

size_t encodeInfo(const RadarInfo& info, uint8_t* out, size_t cap) {
  const size_t total = kProtoHeader + kProtoInfoPayload + 2u;
  if (cap < total || info.n_bins == 0 || info.n_bins > kMaxBins) return 0;
  writeHeader(out, kProtoTypeInfo, 0, 0, 0, 0);
  uint8_t* p = out + kProtoHeader;
  wrf(p,      info.frame_rate_hz);
  wrf(p + 4,  info.range_start_m);
  wrf(p + 8,  info.range_step_m);
  wrf(p + 12, info.wavelength_mm);
  p[16] = info.n_bins;
  wr16(out + kProtoHeader + kProtoInfoPayload, crc16Ccitt(out + 2, total - 4));
  return total;
}

void FrameParser::reset() {
  len_ = 0;
  flags_ = 0;
  n_frames_ = n_info_ = n_crc_ = n_bad_ = n_skip_ = 0;
  frame_ = RadarFrame{};
  info_ = ProtoInfo{};
}

void FrameParser::drop(size_t n) {
  if (n >= len_) { len_ = 0; return; }
  memmove(buf_, buf_ + n, len_ - n);
  len_ -= n;
}

FrameParser::Event FrameParser::push(uint8_t byte) {
  if (len_ >= sizeof(buf_)) { drop(1); n_skip_++; }      // cannot happen with valid framing; stay safe
  buf_[len_++] = byte;
  return process();
}

FrameParser::Event FrameParser::poll() { return process(); }

FrameParser::Event FrameParser::process() {
  for (;;) {
    if (len_ >= 1 && buf_[0] != kProtoSync0) { drop(1); n_skip_++; continue; }
    if (len_ >= 2 && buf_[1] != kProtoSync1) { drop(1); n_skip_++; continue; }
    if (len_ < kProtoHeader) return Event::kNone;

    const uint8_t ver = buf_[2], type = buf_[3], nb = buf_[6];
    size_t total = 0;
    if (ver == kProtoVersion) {
      if (type == kProtoTypeFrame && nb >= 1 && nb <= kMaxBins) total = kProtoHeader + 4u * nb + 2u;
      else if (type == kProtoTypeInfo)                          total = kProtoHeader + kProtoInfoPayload + 2u;
    }
    if (total == 0) { n_bad_++; drop(1); n_skip_++; continue; }
    if (len_ < total) return Event::kNone;

    if (crc16Ccitt(buf_ + 2, total - 4) != rd16(buf_ + total - 2)) {
      n_crc_++; drop(1); n_skip_++;                      // maybe the real sync is inside these bytes
      continue;
    }

    flags_ = buf_[7];
    if (type == kProtoTypeFrame) {
      frame_.n_bins = nb;
      frame_.seq = rd16(buf_ + 4);
      frame_.t_ms = rd32(buf_ + 8);
      frame_.saturated = (flags_ & kProtoFlagSaturated) != 0;
      const uint8_t* p = buf_ + kProtoHeader;
      for (uint8_t b = 0; b < kMaxBins; b++) {
        if (b < nb) {
          frame_.iq[b].i = static_cast<int16_t>(rd16(p + 4u * b));
          frame_.iq[b].q = static_cast<int16_t>(rd16(p + 4u * b + 2));
        } else {
          frame_.iq[b] = IqSample{0, 0};
        }
      }
      drop(total);
      n_frames_++;
      return Event::kFrame;
    }

    // Info packet: validate the content (CRC only proves the sender meant it).
    const uint8_t* p = buf_ + kProtoHeader;
    ProtoInfo in;
    in.frame_rate_hz = rdf(p);
    in.range_start_m = rdf(p + 4);
    in.range_step_m  = rdf(p + 8);
    in.wavelength_mm = rdf(p + 12);
    in.n_bins        = p[16];
    const bool sane = isfinite(in.frame_rate_hz) && in.frame_rate_hz >= 1.0f && in.frame_rate_hz <= 1000.0f &&
                      isfinite(in.range_start_m) && isfinite(in.range_step_m) && in.range_step_m > 0.0f &&
                      isfinite(in.wavelength_mm) && in.wavelength_mm > 0.0f &&
                      in.n_bins >= 1 && in.n_bins <= kMaxBins;
    drop(total);
    if (!sane) { n_bad_++; continue; }
    info_ = in;
    n_info_++;
    return Event::kInfo;
  }
}

}  // namespace rs
