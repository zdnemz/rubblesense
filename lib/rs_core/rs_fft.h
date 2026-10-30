// RubbleSense core - radix-2 complex FFT (single precision) and Hann window.
#pragma once
#include <stdint.h>
#include "rs_config.h"

namespace rs {

// Build the twiddle table (idempotent, thread-safe enough: call once at boot).
void fftInit();

// In-place forward FFT, X[k] = sum x[n] exp(-j 2 pi k n / N).
// n must be a power of two and <= kSegFrames.
void fftComplex(float* re, float* im, uint16_t n);

// Periodic ("DFT-even") Hann window: w[i] = 0.5 - 0.5 cos(2 pi i / n).
// sum(w) = n/2 and sum(w^2) = 3n/8 exactly, which the sensitivity maths relies on.
void hannWindow(float* w, uint16_t n);

}  // namespace rs
