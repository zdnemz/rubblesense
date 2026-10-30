#include "rs_cfar.h"
#include <math.h>

namespace rs {

float cfarAlphaExp(uint16_t nref, float pfa) {
  if (nref == 0) return 1e9f;
  const double n = nref;
  return static_cast<float>(n * (pow(static_cast<double>(pfa), -1.0 / n) - 1.0));
}

// ln of the Fold-cell false-alarm probability.
static double lnPfaFold(double n, double a) {
  return 2.0 * n * log(n / (n + a)) + log(1.0 + 2.0 * n * a / (n + a));
}

float cfarPfaFold(uint16_t nref, float alpha) {
  if (nref == 0) return 1.0f;
  return static_cast<float>(exp(lnPfaFold(nref, alpha)));
}

float cfarAlphaFold(uint16_t nref, float pfa) {
  if (nref == 0) return 1e9f;
  const double n = nref;
  const double target = log(static_cast<double>(pfa));
  double lo = 0.0, hi = 1e6;                 // lnPfaFold is strictly decreasing in alpha
  for (int it = 0; it < 80; it++) {
    const double mid = 0.5 * (lo + hi);
    if (lnPfaFold(n, mid) > target) lo = mid; else hi = mid;
  }
  return static_cast<float>(0.5 * (lo + hi));
}

}  // namespace rs
