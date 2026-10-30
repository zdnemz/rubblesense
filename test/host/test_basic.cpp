// FFT, window and CFAR closed-form checks.
#include <cmath>
#include <complex>
#include <random>
#include <vector>
#include "rs_cfar.h"
#include "rs_fft.h"
#include "rs_test.h"

void test_fft_matches_dft() {
  rs::fftInit();
  std::mt19937 rng(1);
  std::normal_distribution<float> nd(0.f, 1.f);
  for (uint16_t n : {8, 64, 256, 512}) {
    std::vector<float> re(n), im(n);
    std::vector<std::complex<double>> x(n);
    for (uint16_t i = 0; i < n; i++) {
      re[i] = nd(rng); im[i] = nd(rng);
      x[i] = {re[i], im[i]};
    }
    rs::fftComplex(re.data(), im.data(), n);
    double maxerr = 0, maxmag = 0;
    for (uint16_t k = 0; k < n; k++) {
      std::complex<double> s = 0;
      for (uint16_t i = 0; i < n; i++) {
        const double a = -2.0 * M_PI * double(k) * double(i) / double(n);
        s += x[i] * std::complex<double>(std::cos(a), std::sin(a));
      }
      maxerr = std::max(maxerr, std::abs(s - std::complex<double>(re[k], im[k])));
      maxmag = std::max(maxmag, std::abs(s));
    }
    CHECK(maxerr < 2e-5 * maxmag * std::sqrt(double(n)));
  }
}

void test_fft_tone_bin() {
  // A complex tone exactly on bin 7 must land in bin 7 only.
  const uint16_t n = 256;
  std::vector<float> re(n), im(n);
  for (uint16_t i = 0; i < n; i++) {
    re[i] = std::cos(2 * M_PI * 7 * i / n);
    im[i] = std::sin(2 * M_PI * 7 * i / n);
  }
  rs::fftComplex(re.data(), im.data(), n);
  CHECK_NEAR(re[7], n, 1e-2);
  CHECK_NEAR(im[7], 0, 1e-2);
  CHECK_NEAR(std::hypot(re[8], im[8]), 0, 1e-2);
}

void test_hann_sums() {
  for (uint16_t n : {128, 256, 512}) {
    std::vector<float> w(n);
    rs::hannWindow(w.data(), n);
    double s = 0, s2 = 0;
    for (float v : w) { s += static_cast<double>(v); s2 += static_cast<double>(v) * static_cast<double>(v); }
    CHECK_NEAR(s, n / 2.0, 1e-3);
    CHECK_NEAR(s2, 3.0 * n / 8.0, 1e-3);
  }
}

void test_cfar_closed_forms() {
  // Closed-form inverse round trip.
  for (uint16_t n : {4, 8, 16}) {
    for (float pfa : {1e-2f, 1e-3f, 1e-5f}) {
      const float a = rs::cfarAlphaFold(n, pfa);
      CHECK_REL(rs::cfarPfaFold(n, a), pfa, 1e-3);
    }
  }
  CHECK_NEAR(rs::cfarAlphaExp(16, 1e-3f), 16.0 * (std::pow(1e-3, -1.0 / 16) - 1.0), 1e-3);

  // Monte-Carlo against the exact statistics (Fold cells = sum of 2 exponentials).
  std::mt19937_64 rng(42);
  std::exponential_distribution<double> ex(1.0);
  for (float pfa : {1e-2f, 1e-3f}) {
    const uint16_t nref = 16;
    const float alpha = rs::cfarAlphaFold(nref, pfa);
    const long trials = 1500000;
    long hits = 0;
    for (long t = 0; t < trials; t++) {
      const double cut = ex(rng) + ex(rng);
      double ref = 0;
      for (int i = 0; i < nref; i++) ref += ex(rng) + ex(rng);
      ref /= nref;
      if (cut > static_cast<double>(alpha) * ref) hits++;
    }
    const double emp = double(hits) / trials;
    CHECK_REL(emp, pfa, 0.12);
  }
  // Exp cells
  {
    const uint16_t nref = 12;
    const float pfa = 1e-2f, alpha = rs::cfarAlphaExp(nref, pfa);
    long hits = 0; const long trials = 1000000;
    for (long t = 0; t < trials; t++) {
      const double cut = ex(rng);
      double ref = 0;
      for (int i = 0; i < nref; i++) ref += ex(rng);
      ref /= nref;
      if (cut > static_cast<double>(alpha) * ref) hits++;
    }
    CHECK_REL(double(hits) / trials, pfa, 0.08);
  }
}

void test_basic() {
  RUN(test_fft_matches_dft);
  RUN(test_fft_tone_bin);
  RUN(test_hann_sums);
  RUN(test_cfar_closed_forms);
}
