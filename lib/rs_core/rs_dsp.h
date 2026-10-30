// RubbleSense core - per-segment signal analysis.
//
// Pipeline (all on complex I/Q slow-time data, one series per range bin):
//   1. frame-quality gating (IMU motion, saturation, link drops)
//   2. common-phase-error (platform motion) estimate per range bin from the
//      OTHER bins (leave-neighbourhood-out) and compensation of that bin
//   3. static-clutter removal (per-bin complex mean), Hann window, FFT
//   4. folded (+f/-f) Doppler power spectrum in the respiration band
//   5. CA-CFAR with a multiple-testing-corrected false-alarm probability
//   6. platform-coherence test (is the line also present in the common phase of
//      the other bins? then it is the sensor moving, not a person)
//   7. guided confirmation of seeds from earlier segments
//
// The module never says "no person": it only reports detections, the noise
// floor and data quality. Interpretation lives in rs_decision.
#pragma once
#include <stdint.h>
#include "rs_config.h"
#include "rs_types.h"

namespace rs {

struct Detection {
  uint8_t  bin;        // range bin
  uint16_t k;          // FFT cell
  float    hz;         // sub-bin refined frequency
  float    snr_db;     // peak over local noise floor
  float    margin_db;  // peak over CFAR threshold (> 0 means detected)
  bool     tested;     // platform-coherence test was sensitive enough to vet this line
  bool     platform_like;  // coincides with platform motion (common-phase line / scene-wide line)
};

enum class InvalidReason : uint8_t {
  kNone = 0, kTooShort, kMotion, kDropout, kImuMissing, kConfig, kNoSignal, kSaturated, kTiming,
  kCount                                   // sentinel: number of reasons (tables and loops are sized with it)
};

struct Seed {
  uint8_t bin;
  float   hz;
  float   margin_db;
};

struct SegmentResult {
  bool          valid;
  InvalidReason reason;
  uint16_t      n_used;                  // frames analysed (kSegFrames or kPrevFrames)
  float         bad_frac;                // frames flagged IMU-motion, saturated or with inconsistent timing
  float         drop_frac;               // frames lost on the radar link
  float         imu_missing_frac;

  uint8_t       n_det;                   // real candidates, strongest first
  Detection     det[kMaxDet];
  uint8_t       n_platform;              // detections rejected as platform-coherent
  uint8_t       n_untested;              // detections the coherence test could not vet
  bool          confirmed;               // guided confirmation against `seeds` passed
  Detection     conf;
  uint8_t       conf_seed;               // index of the seed that was confirmed

  float         floor_db;                // median in-band noise floor, counts^2 per complex sample
  bool          cpe_applied;             // platform-motion compensation active (>= half of the bins)
  uint8_t       cpe_bins;                // range bins that were compensated
  uint8_t       ref_bin;                 // strongest static reflector (informational)
  float         ref_snr_db;              // its static power over the noise floor
  bool          platform_peak;           // strong common-phase line inside the band
  float         platform_hz;
  float         theta_rms_rad;           // rms of the common phase of the scene (platform micro-motion);
                                         // x_rms = theta_rms * wavelength / (4 pi)
  uint16_t      n_cells;                 // cells covered by the full search
};

// Optional by-product for the operator display (best range bin only).
struct PreviewData {
  bool     have;
  uint8_t  best_bin;
  uint16_t k0;                           // frequency index of spec_db[0]
  uint16_t n_cells;
  float    df_hz;
  float    spec_db[kMaxSpecCells];       // folded power of the best bin
  float    thr_db[kMaxSpecCells];        // CFAR threshold of that bin (-200 = undefined)
  uint16_t wave_n;
  float    wave[kPrevFrames];            // dominant-axis projection, normalised to +-1
};

// Scratch memory for analyzeSegment (about 36 KB). One instance, owned by the DSP task.
struct Workspace {
  float re[kSegFrames];
  float im[kSegFrames];
  float win[kSegFrames];
  uint16_t win_n = 0;                    // window length currently cached in win[]
  float cs[kSegFrames];                  // cos(common phase) per frame (current bin)
  float sn[kSegFrames];                  // sin(common phase) per frame (current bin)
  float theta[kSegFrames];               // common phase estimate (current bin)
  float sr[kSegFrames];                  // sum over bins of Re(z * conj(mean))
  float si[kSegFrames];                  // sum over bins of Im(z * conj(mean))
  float spec[kMaxBins][kMaxSpecCells];   // folded power cells per range bin
  float pth[kMaxBins][kMaxSpecCells];    // power spectrum of the common phase seen by each bin
  float thth[kMaxBins];                  // coherence threshold per bin (linear)
  float vis_db[kMaxBins];                // how visible a line of this bin is in its common phase
  float spw[kMaxBins];                   // static power |mean|^2 per bin
  float cmi[kMaxBins], cmq[kMaxBins];    // per-bin mean I and Q
  float noise2[kMaxBins];                // per-bin noise power estimate
  bool  have_th[kMaxBins];
  bool  comp_b[kMaxBins];                // bin was compensated for common-phase motion
  float floorv[kMaxBins * kMaxSpecCells];
};

// Analyse the most recent kSegFrames (if available) or kPrevFrames frames of `buf`.
// `seeds` are detections from earlier segments; the guided confirmation test is
// run in their neighbourhood with the (more permissive) pfa_confirm threshold.
void analyzeSegment(const SegmentBuffer& buf, uint16_t n_avail, const DspParams& p,
                    const Seed* seeds, uint8_t n_seeds, Workspace& ws,
                    SegmentResult& out, PreviewData* pv = nullptr);

}  // namespace rs
