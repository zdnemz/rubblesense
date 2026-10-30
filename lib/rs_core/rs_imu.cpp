#include "rs_imu.h"
#include <math.h>
#include <string.h>

namespace rs {

namespace {
constexpr float kRadToDeg = 57.29577951f;

inline void normalize(const float* v, float* out) {
  const float m = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  const float inv = (m > 1e-6f) ? 1.0f / m : 0.0f;
  out[0] = v[0] * inv; out[1] = v[1] * inv; out[2] = v[2] * inv;
}

// Angle between two unit vectors, accurate for tiny angles (atan2 of |a x b| and a.b).
float angleDeg(const float* a, const float* b) {
  const float cx = a[1] * b[2] - a[2] * b[1];
  const float cy = a[2] * b[0] - a[0] * b[2];
  const float cz = a[0] * b[1] - a[1] * b[0];
  const float s = sqrtf(cx * cx + cy * cy + cz * cz);
  const float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return atan2f(s, d) * kRadToDeg;
}
}  // namespace

void ImuProcessor::configure(const ImuGateParams& gp, float imu_hz, float frame_hz) {
  gp_ = gp;
  imu_hz_ = imu_hz;
  frame_hz_ = frame_hz;
  a_alpha_ = 1.0f / (0.5f * imu_hz_);     // gravity / high-pass time constant 0.5 s
  b_alpha_ = 1.0f / (3.0f * imu_hz_);     // gyro-bias tracking time constant 3 s
  hist_len_ = static_cast<int>(frame_hz_ + 0.5f);   // ~1 s of history for the tilt rate
  if (hist_len_ < 2) hist_len_ = 2;
  if (hist_len_ > kHist) hist_len_ = kHist;
  reset();
}

void ImuProcessor::reset() {
  init_ = false; init_n_ = 0; update_bias_ = true;
  for (int i = 0; i < 3; i++) { acc_lp_[i] = 0; gbias_[i] = 0; }
  sum_a2_ = sum_g2_ = 0; n_ = 0;
  hist_n_ = 0; hist_head_ = 0; hold_ = 0;
}

void ImuProcessor::push(float ax, float ay, float az, float gx, float gy, float gz) {
  const float a[3] = {ax, ay, az};
  const float g[3] = {gx, gy, gz};
  if (!init_) {                       // first sample: start the filters at the current value
    for (int i = 0; i < 3; i++) acc_lp_[i] = a[i];
    init_ = true; init_n_ = 0;
  }
  // Gyro bias: plain running mean over the first 0.5 s, then a slow EMA that is
  // frozen while the sensor is being moved (so motion is not absorbed as bias).
  const int init_len = static_cast<int>(0.5f * imu_hz_);
  if (init_n_ < init_len) {
    init_n_++;
    const float w = 1.0f / static_cast<float>(init_n_);
    for (int i = 0; i < 3; i++) gbias_[i] += w * (g[i] - gbias_[i]);
  } else if (update_bias_) {
    for (int i = 0; i < 3; i++) gbias_[i] += b_alpha_ * (g[i] - gbias_[i]);
  }

  float da2 = 0.0f, dg2 = 0.0f;
  for (int i = 0; i < 3; i++) {
    const float dyn = a[i] - acc_lp_[i];          // high-passed acceleration
    da2 += dyn * dyn;
    const float gd = g[i] - gbias_[i];
    dg2 += gd * gd;
    acc_lp_[i] += a_alpha_ * dyn;                 // low-pass = gravity estimate
  }
  sum_a2_ += da2;
  sum_g2_ += dg2;
  n_++;
}

ImuStats ImuProcessor::takeFrame() {
  ImuStats s{};
  if (n_ == 0) {
    s.valid = false;
    s.motion = true;                              // unknown stability is treated as unstable
    return s;
  }
  s.valid = true;
  s.a_rms = sqrtf(sum_a2_ / static_cast<float>(n_));
  s.g_rms = sqrtf(sum_g2_ / static_cast<float>(n_));
  sum_a2_ = sum_g2_ = 0; n_ = 0;

  float gdir[3];
  normalize(acc_lp_, gdir);
  if (hist_n_ >= hist_len_) {
    const int oldest = (hist_head_ + kHist - (hist_len_ - 1)) % kHist;
    // `oldest` was stored hist_len_ frames before the current one.
    s.tilt_rate_dps = angleDeg(gdir, hist_[oldest]) * frame_hz_ / static_cast<float>(hist_len_);
  } else {
    s.tilt_rate_dps = 0.0f;
  }
  hist_head_ = (hist_head_ + 1) % kHist;
  memcpy(hist_[hist_head_], gdir, sizeof(gdir));
  if (hist_n_ < kHist) hist_n_++;

  const bool now = s.a_rms > gp_.a_rms_max || s.g_rms > gp_.g_rms_max ||
                   s.tilt_rate_dps > gp_.tilt_rate_max_dps;
  if (now) hold_ = gp_.hold_frames;
  s.motion = now || hold_ > 0;
  if (!now && hold_ > 0) hold_--;
  update_bias_ = !s.motion;
  return s;
}

}  // namespace rs
