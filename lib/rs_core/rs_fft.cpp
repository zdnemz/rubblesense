#include "rs_fft.h"
#include <math.h>

namespace rs {

namespace {
float g_cos[kSegFrames / 2];
float g_sin[kSegFrames / 2];   // holds -sin(): forward transform
bool  g_ready = false;
}  // namespace

void fftInit() {
  if (g_ready) return;
  const double two_pi = 6.283185307179586476925286766559;
  for (uint16_t i = 0; i < kSegFrames / 2; i++) {
    const double a = two_pi * static_cast<double>(i) / static_cast<double>(kSegFrames);
    g_cos[i] = static_cast<float>(cos(a));
    g_sin[i] = static_cast<float>(-sin(a));
  }
  g_ready = true;
}

void fftComplex(float* re, float* im, uint16_t n) {
  if (!g_ready) fftInit();

  // Bit-reversal permutation.
  for (uint16_t i = 1, j = 0; i < n; i++) {
    uint16_t bit = static_cast<uint16_t>(n >> 1);
    for (; j & bit; bit = static_cast<uint16_t>(bit >> 1)) j = static_cast<uint16_t>(j ^ bit);
    j = static_cast<uint16_t>(j ^ bit);
    if (i < j) {
      const float tr = re[i]; re[i] = re[j]; re[j] = tr;
      const float ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }

  // Iterative decimation-in-time butterflies.
  for (uint16_t len = 2; len <= n; len = static_cast<uint16_t>(len << 1)) {
    const uint16_t half = static_cast<uint16_t>(len >> 1);
    const uint16_t step = static_cast<uint16_t>(kSegFrames / len);
    for (uint16_t i = 0; i < n; i = static_cast<uint16_t>(i + len)) {
      for (uint16_t j = 0; j < half; j++) {
        const float wr = g_cos[j * step];
        const float wi = g_sin[j * step];
        const uint16_t u = static_cast<uint16_t>(i + j);
        const uint16_t v = static_cast<uint16_t>(u + half);
        const float tr = re[v] * wr - im[v] * wi;
        const float ti = re[v] * wi + im[v] * wr;
        re[v] = re[u] - tr;
        im[v] = im[u] - ti;
        re[u] += tr;
        im[u] += ti;
      }
    }
  }
}

void hannWindow(float* w, uint16_t n) {
  const double two_pi = 6.283185307179586476925286766559;
  for (uint16_t i = 0; i < n; i++) {
    w[i] = static_cast<float>(0.5 - 0.5 * cos(two_pi * static_cast<double>(i) / static_cast<double>(n)));
  }
}

}  // namespace rs
