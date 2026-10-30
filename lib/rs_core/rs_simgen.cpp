#include "rs_simgen.h"
#include <math.h>

namespace rs {

namespace {
constexpr float kTwoPi = 6.28318530718f;

inline int16_t quant(float v, bool& sat) {
  if (v >= 32767.0f)  { sat = true; return 32767; }
  if (v <= -32767.0f) { sat = true; return -32767; }
  return static_cast<int16_t>(lroundf(v));
}

// Bessel J1 by power series (test/label use only).
double besselJ1(double x) {
  double term = x / 2.0, sum = term;
  for (int m = 1; m < 40; m++) {
    term *= -(x * x / 4.0) / (static_cast<double>(m) * static_cast<double>(m + 1));
    sum += term;
  }
  return sum;
}
}  // namespace

float Rng::normal() {
  if (have_) { have_ = false; return spare_; }
  const float u1 = uniform();
  const float u2 = uniform();
  const float r = sqrtf(-2.0f * logf(u1));
  const float a = kTwoPi * u2;
  spare_ = r * sinf(a);
  have_ = true;
  return r * cosf(a);
}

void SimRadar::reset(uint32_t seed) {
  rng_.reseed(seed);
  for (int b = 0; b < kMaxBins; b++) {
    ph_[b] = kTwoPi * rng_.uniform();
    gre_[b] = rng_.normal();
    gim_[b] = rng_.normal();
  }
  ph_t_ = kTwoPi * rng_.uniform();
  {
    Rng mg(static_cast<uint64_t>(seed) ^ 0x5DEECE66Dull);   // separate stream: existing seeds keep their scenes
    for (int b = 0; b < kMaxBins; b++) gmm_[b] = mg.normal();
  }
  bphase_ = kTwoPi * rng_.uniform();
  hphase_ = kTwoPi * rng_.uniform();
  vphase_ = kTwoPi * rng_.uniform();
  wander_ = 0.0f; bump_ = 0.0f; drift_ = 0.0f; last_dbump_ = 0.0f;
  seq_ = 0; t_ms_ = 0;
  rebuild();
}

void SimRadar::rebuild() {
  const SimScene& s = scene;
  for (int b = 0; b < kMaxBins; b++) {
    float re = 0.0f, im = 0.0f;
    if (b < 2) {
      const float a = s.leak_amp * (b == 0 ? 1.0f : 0.5f);
      re = a * cosf(ph_[b]); im = a * sinf(ph_[b]);
    } else {
      const float d = fabsf(static_cast<float>(b) - static_cast<float>(s.wall_bin));
      const float c = s.clutter_amp * expf(-d / 12.0f);
      re = c * gre_[b]; im = c * gim_[b];
      float w = 0.0f;
      if (b == s.wall_bin) w = s.wall_amp;
      else if (d < 1.5f)   w = 0.3f * s.wall_amp;
      re += w * cosf(ph_[b]); im += w * sinf(ph_[b]);
    }
    sre_[b] = re; sim_[b] = im;
  }
  tre_ = cosf(ph_t_); tim_ = sinf(ph_t_);
}

void SimRadar::step(RadarFrame& f) {
  const SimScene& s = scene;
  const float dt = 1.0f / s.fs_hz;
  const float k = 2.0f * kTwoPi / s.wavelength_mm;       // 4*pi/lambda  [rad/mm]
  const int nb = (s.n_bins > kMaxBins) ? kMaxBins : s.n_bins;

  // platform displacement, common to every external reflector
  const float x_vib = s.vib_mm * sinf(vphase_);
  vphase_ += kTwoPi * s.vib_hz * dt;
  if (vphase_ > kTwoPi) vphase_ -= kTwoPi;
  const float bump_prev = bump_;
  if (s.bump_rate_hz > 0.0f && rng_.uniform() < s.bump_rate_hz * dt) bump_ += s.bump_mm * rng_.normal();
  bump_ *= 0.92f;
  last_dbump_ = fabsf(bump_ - bump_prev);
  if (s.drift_mm_rms > 0.0f) drift_ += s.drift_mm_rms * sqrtf(dt) * rng_.normal();
  const float xp = x_vib + bump_ + drift_;
  const float cpc = cosf(k * xp), cps = sinf(k * xp);

  // chest displacement
  wander_ += (-wander_ / 30.0f) * dt + s.breath_jitter * sqrtf(2.0f * dt / 30.0f) * rng_.normal();
  bphase_ += kTwoPi * s.breath_hz * (1.0f + wander_) * dt;
  hphase_ += kTwoPi * s.heart_hz * dt;
  if (bphase_ > kTwoPi) bphase_ -= kTwoPi;
  if (hphase_ > kTwoPi) hphase_ -= kTwoPi;
  const float xt = s.breath_mm * (sinf(bphase_) + 0.15f * sinf(2.0f * bphase_ + 0.7f)) +
                   s.heart_mm * sinf(hphase_);
  const float ctt = cosf(k * xt), stt = sinf(k * xt);

  bool sat = false;
  for (int b = 0; b < nb; b++) {
    float re, im;
    if (b < 2) {                                  // antenna leakage: rigid with the sensor
      re = sre_[b]; im = sim_[b];
    } else {
      float er = sre_[b], ei = sim_[b];
      if (s.target) {
        const int d = abs(b - static_cast<int>(s.target_bin));
        const float w = (d == 0) ? 1.0f : (d == 1 ? 0.35f : 0.0f);
        if (w > 0.0f) {
          const float a = s.target_amp * w;
          er += a * (tre_ * ctt - tim_ * stt);
          ei += a * (tre_ * stt + tim_ * ctt);
        }
      }
      float cb = cpc, sb = cps;
      if (s.cpe_mismatch > 0.0f) {                // this bin sees a slightly different platform phase
        const float ph = k * xp * (1.0f + s.cpe_mismatch * gmm_[b]);
        cb = cosf(ph); sb = sinf(ph);
      }
      re = er * cb - ei * sb;                     // times exp(j k x_p)
      im = er * sb + ei * cb;
    }
    re += s.noise_sigma * rng_.normal();
    im += s.noise_sigma * rng_.normal();
    f.iq[b].i = quant(re, sat);
    f.iq[b].q = quant(im, sat);
  }
  f.n_bins = static_cast<uint8_t>(nb);
  f.seq = seq_++;
  t_ms_ += static_cast<uint32_t>(1000.0f / s.fs_hz + 0.5f);
  f.t_ms = t_ms_;
  f.saturated = sat;
}

void SimRadar::imuStep(ImuProcessor& imu) {
  const SimScene& s = scene;
  const float bias[3] = {0.10f, -0.15f, 0.06f};      // typical MPU6050 zero-rate offsets (rad/s)
  for (int i = 0; i < 5; i++) {
    const float sign = (rng_.uniform() < 0.5f) ? -1.0f : 1.0f;
    const float ka = 0.4f * last_dbump_ * sign;      // ~0.4 m/s^2 per mm of impulsive displacement
    const float kg = 0.1f * last_dbump_ * sign;
    imu.push(s.imu_noise_a * rng_.normal() + ka,
             s.imu_noise_a * rng_.normal(),
             9.81f + s.imu_noise_a * rng_.normal(),
             bias[0] + s.imu_noise_g * rng_.normal() + kg,
             bias[1] + s.imu_noise_g * rng_.normal(),
             bias[2] + s.imu_noise_g * rng_.normal());
  }
}

RadarInfo SimRadar::info() const {
  RadarInfo r;
  r.name = "SIM";
  r.frame_rate_hz = scene.fs_hz;
  r.range_start_m = 0.5f;
  r.range_step_m = 0.15f;
  r.wavelength_mm = scene.wavelength_mm;
  r.n_bins = scene.n_bins;
  return r;
}

float SimRadar::nominalToneSnrDb(const SimScene& s, uint16_t n_frames) {
  const double k = 4.0 * 3.14159265358979 / static_cast<double>(s.wavelength_mm);
  const double j1 = besselJ1(k * static_cast<double>(s.breath_mm));
  const double a2 = static_cast<double>(s.target_amp) * static_cast<double>(s.target_amp);
  const double sc2 = 2.0 * static_cast<double>(s.noise_sigma) * static_cast<double>(s.noise_sigma);
  const double snr = a2 * j1 * j1 * static_cast<double>(n_frames) / (1.5 * sc2);
  return static_cast<float>(10.0 * log10(snr + 1e-30));
}

}  // namespace rs
