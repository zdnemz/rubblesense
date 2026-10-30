// RubbleSense core - CA-CFAR threshold factors (closed form).
//
// A detection is declared when  P_cut > alpha * mean(P_ref)  where the reference
// cells are neighbours of the cell under test. alpha is chosen so that, on pure
// noise, the cell-level false-alarm probability equals `pfa`.
//
// Two cell statistics are supported:
//   * Exp   : |FFT bin|^2 of white complex Gaussian noise   (chi-square, 2 dof)
//   * Fold  : |X[k]|^2 + |X[N-k]|^2, both Doppler side-bands added (Gamma(2) cells)
//             -> exact tail  Pfa = (N/(N+a))^(2N) * (1 + 2Na/(N+a))
#pragma once
#include <stdint.h>

namespace rs {

// Exp cells:  alpha = N (Pfa^(-1/N) - 1)
float cfarAlphaExp(uint16_t nref, float pfa);

// Fold cells: alpha such that cfarPfaFold(nref, alpha) == pfa (bisection).
float cfarAlphaFold(uint16_t nref, float pfa);

// Exact false-alarm probability of a Fold cell for a given alpha (used by tests).
float cfarPfaFold(uint16_t nref, float alpha);

}  // namespace rs
