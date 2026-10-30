// RubbleSense core - serial I/Q link protocol.
//
// Used to feed the firmware with I/Q frames from outside the ESP32: a PC replaying a
// recording, a hardware-in-the-loop rig, or a real radar front-end with its own MCU.
// All fields are little-endian. One packet:
//
//   offset size  field
//   0      1     0xA5            sync 0
//   1      1     0x5A            sync 1
//   2      1     version         currently 1
//   3      1     type            1 = range frame, 2 = front-end info
//   4      2     seq             frame counter (type 1); 0 for type 2
//   6      1     nbins           range bins in the frame (type 1); 0 for type 2
//   7      1     flags           bit0 = ADC / I-Q saturated
//   8      4     t_ms            front-end timestamp, milliseconds
//   12     N     payload         type 1: nbins x (int16 I, int16 Q)
//                                type 2: float32 frame_rate_hz, range_start_m,
//                                        range_step_m, wavelength_mm; uint8 n_bins
//   12+N   2     crc16           CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
//                                over bytes [2, 12+N), i.e. version .. payload
//
// The parser re-synchronises after any corruption by re-scanning its own buffer, so
// a single flipped byte costs at most the packets that overlap it.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "rs_types.h"

namespace rs {

constexpr uint8_t  kProtoSync0   = 0xA5;
constexpr uint8_t  kProtoSync1   = 0x5A;
constexpr uint8_t  kProtoVersion = 1;
constexpr uint8_t  kProtoHeader  = 12;
constexpr uint8_t  kProtoInfoPayload = 17;
constexpr size_t   kProtoMaxPacket   = kProtoHeader + 4u * kMaxBins + 2u;   // largest frame packet
constexpr uint8_t  kProtoFlagSaturated = 1u << 0;   // ADC / I-Q clipped
constexpr uint8_t  kProtoFlagMotion    = 1u << 1;   // the sender's own IMU saw the sensor move
constexpr uint8_t  kProtoFlagDropped   = 1u << 2;   // frame is a hold of the previous one (sender lost it)
constexpr uint8_t  kProtoFlagHostImu   = 1u << 3;   // the sender vouches that kProtoFlagMotion is valid, i.e. the
                                                    // IMU is mounted with the radar (a replayed recording must
                                                    // carry this: the receiver's own IMU is not on that radar)

enum ProtoType : uint8_t { kProtoTypeFrame = 1, kProtoTypeInfo = 2 };

uint16_t crc16Ccitt(const uint8_t* data, size_t n, uint16_t crc = 0xFFFF);

// Encoders return the packet length, or 0 if `cap` is too small / arguments are invalid.
// `extra_flags` is OR-ed with the saturation bit taken from the frame.
size_t encodeFrame(const RadarFrame& f, uint8_t* out, size_t cap, uint8_t extra_flags = 0);
size_t encodeInfo(const RadarInfo& info, uint8_t* out, size_t cap);

// Number of frames missing between two consecutive sequence numbers (wrap-safe).
inline uint16_t seqGap(uint16_t prev, uint16_t cur) { return static_cast<uint16_t>(cur - prev - 1u); }

// Front-end description as received (RadarInfo without the name pointer).
struct ProtoInfo {
  float   frame_rate_hz;
  float   range_start_m;
  float   range_step_m;
  float   wavelength_mm;
  uint8_t n_bins;
};

class FrameParser {
 public:
  enum class Event : uint8_t { kNone, kFrame, kInfo };

  void reset();

  // Feed one byte. Returns kFrame / kInfo when a complete, CRC-valid packet has just
  // been decoded (then frame() / info() hold it). After an event, call poll() until it
  // returns kNone to drain packets that were already buffered.
  Event push(uint8_t byte);
  Event poll();

  const RadarFrame& frame() const { return frame_; }
  const ProtoInfo&  info()  const { return info_; }
  uint8_t           flags() const { return flags_; }   // packet flags of the last event

  uint32_t framesOk()      const { return n_frames_; }
  uint32_t infoOk()        const { return n_info_; }
  uint32_t crcErrors()     const { return n_crc_; }
  uint32_t badPackets()    const { return n_bad_; }     // header-level rejects (version, type, size)
  uint32_t bytesSkipped()  const { return n_skip_; }

 private:
  Event process();
  void  drop(size_t n);

  uint8_t   buf_[kProtoMaxPacket];
  size_t    len_ = 0;
  RadarFrame frame_{};
  ProtoInfo  info_{};
  uint8_t   flags_ = 0;
  uint32_t  n_frames_ = 0, n_info_ = 0, n_crc_ = 0, n_bad_ = 0, n_skip_ = 0;
};

}  // namespace rs
