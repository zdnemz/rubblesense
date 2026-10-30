// RubbleSense core - compile-time geometry and run-time tunable parameters.
//
// Everything in lib/rs_core is portable C++11 (no Arduino headers) so the exact
// same DSP runs on the ESP32 and in the host test-bench (test/host).
#pragma once
#include <stdint.h>

namespace rs {

// ---------------------------------------------------------------------------
// Compile-time geometry (changing these changes RAM use; see docs/ARSITEKTUR.md)
// ---------------------------------------------------------------------------
constexpr uint16_t kSegFrames   = 512;  // slow-time frames per decision segment (25.6 s @ 20 Hz)
constexpr uint16_t kPrevFrames  = 256;  // frames per live-preview window       (12.8 s @ 20 Hz)
constexpr uint8_t  kMaxBins     = 24;   // range bins stored per frame (49 KB of int16 I/Q)
constexpr uint8_t  kMaxSegments = 3;    // valid segments per scan (2 needed, 3rd = tie-break)
constexpr uint8_t  kMaxInvalid  = 2;    // invalid segments tolerated before giving up
constexpr uint8_t  kMaxDet      = 4;    // detections kept per segment
constexpr uint8_t  kMaxSeeds    = 4;    // seeds carried to the next segment
constexpr uint16_t kMaxSpecCells = 64;  // spectral cells kept per range bin (band + CFAR refs)

static_assert((kSegFrames & (kSegFrames - 1)) == 0, "kSegFrames must be a power of two");
static_assert((kPrevFrames & (kPrevFrames - 1)) == 0, "kPrevFrames must be a power of two");
static_assert(kPrevFrames <= kSegFrames, "preview window cannot exceed segment");

// ---------------------------------------------------------------------------
// Run-time parameters. Defaults are conservative and were calibrated on the
// *simulated* noise model only (see docs/ARSITEKTUR.md section 8). They MUST be
// re-calibrated with recorded data from the real radar front-end.
// ---------------------------------------------------------------------------
struct DspParams {
  // Radar slow-time frame rate (Hz). Must equal the real front-end's rate.
  float fs_hz = 20.0f;

  // Respiration search band. 0.10-0.80 Hz = 6-48 breaths/min: covers adults,
  // children and slow/laboured breathing. (The original 0.15-0.50 Hz band
  // excluded children and bradypnoea.)
  float resp_lo_hz = 0.10f;
  float resp_hi_hz = 0.80f;

  // First range bin searched. Bins below are antenna leakage / near-field
  // (also where the operator's own body is) and are never used for decisions.
  uint8_t min_bin = 2;

  // --- CA-CFAR over the folded (+f/-f) Doppler spectrum -------------------
  uint8_t cfar_guard = 2;       // guard cells each side (Hann main lobe = +-2 bins)
  uint8_t cfar_ref   = 8;       // reference cells each side
  float   pfa_segment = 0.01f;  // family-wise false-alarm prob. of the full search (per segment)
  float   pfa_confirm = 0.02f;  // family-wise false-alarm prob. of the guided confirmation
  float   freq_tol_hz = 0.08f;  // confirmation neighbourhood: frequency (+-)
  uint8_t bin_tol     = 1;      // confirmation neighbourhood: range bins (+-)

  // --- Common-phase-error (platform motion) handling ------------------------
  // For every range bin the common phase of the scene is estimated from the OTHER
  // bins only (the bin under test and its two neighbours are left out), so a person
  // breathing in a bin can never contaminate the estimate used to judge that bin.
  bool    cpe_enable     = true;   // apply the compensation (the coherence test always runs)
  uint8_t cpe_guard_bins = 1;      // bins left out on each side of the bin under test (widen for blurry range response)
  float   cpe_comp_ratio = 5.0f;   // compensate a bin only if the other bins hold >= this x its static power
  float   coh_thr_db     = 10.0f;  // a common-phase line must exceed its noise floor by this much
  float   coh_margin_db  = 3.0f;   // a detection is "tested" if the test would have seen it with this margin
  float   coh_tol_db     = 15.0f;  // a bin line is "explained" by a common-phase line if it is at most this much
                                   // stronger than that line could have made it (allows non-common motion)

  // A person fills 2-3 adjacent range bins at most; platform motion shows up in all of them.
  // A line that also appears at the same frequency in >= sw_min_bins bins at least sw_dist
  // bins away is "scene-wide" and is attributed to the platform (unless it is overwhelmingly
  // stronger than all of those, sw_exempt_db).
  uint8_t sw_dist        = 3;
  uint8_t sw_min_bins    = 4;
  float   sw_snr_db      = 8.0f;
  float   sw_exempt_db   = 20.0f;

  // --- Data-quality gates ---------------------------------------------------
  float   max_bad_frac   = 0.05f;  // frames flagged IMU-motion / saturated / bad timing
  float   max_drop_frac  = 0.02f;  // frames lost on the radar link
  bool    require_imu    = true;   // fail-safe: no IMU => segment cannot be trusted
  // A dead / frozen / disconnected front-end delivers (almost) no noise at all. That must
  // never read as "nothing there", so a noise power below this is an invalid segment.
  float   min_noise_power = 0.05f; // counts^2 per complex sample (a 16-bit ADC alone gives >= ~0.17)

  // --- Sensitivity (negative-result credibility) ----------------------------
  bool    have_baseline     = false;  // bench baseline noise floor available?
  float   baseline_floor_db = 0.0f;   // median in-band noise floor measured at calibration
  float   low_sens_db       = 3.0f;   // floor this far above baseline => result inconclusive
};

}  // namespace rs
