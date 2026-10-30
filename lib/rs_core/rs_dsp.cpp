#include "rs_dsp.h"

#include <algorithm>
#include <math.h>
#include <string.h>

#include "rs_cfar.h"
#include "rs_fft.h"

namespace rs {
namespace {

constexpr int   kMaxRef = 16;
// Hann-windowed periodogram bins are correlated (power correlation 0.44 at lag 1, 0.03 at
// lag 2), so the mean of n adjacent reference cells has the variance of only ~0.55*n
// independent cells. The CFAR factor is computed for that effective count.
constexpr float kRefDecorr = 0.55f;

inline float dbf(float x) { return 10.0f * log10f(x > 1e-12f ? x : 1e-12f); }
inline float undb(float db) { return powf(10.0f, db / 10.0f); }

inline float medianOf(float* v, int n) {
  std::nth_element(v, v + n / 2, v + n);
  return v[n / 2];
}

// Mean of exponentially distributed cells (power of a noise-only spectrum) estimated from
// the LOWER QUARTILE. Strong lines and their window skirts occupy the upper cells, so unlike
// a median this is not pulled up by them (-ln(1-0.25) = 0.28768 converts quartile to mean).
inline float lowerQuartileMean(float* v, int n) {
  const int q = n / 4;
  std::nth_element(v, v + q, v + n);
  return v[q] / 0.28768f;
}

// Lazily computed CFAR factors indexed by the number of usable reference cells.
struct AlphaTable {
  float pfa = 0.0f;
  float a[2 * kMaxRef + 1];
  void init(float pfa_cell) { pfa = pfa_cell; for (float& v : a) v = -1.0f; }
  float get(int n) {
    if (n < 1) n = 1;
    if (n > 2 * kMaxRef) n = 2 * kMaxRef;
    if (a[n] < 0.0f) {
      int ne = static_cast<int>(kRefDecorr * static_cast<float>(n) + 0.5f);
      if (ne < 2) ne = 2;
      a[n] = cfarAlphaFold(static_cast<uint16_t>(ne), pfa);
    }
    return a[n];
  }
};

// Mean of the reference cells around cell k (guard cells excluded). Cells outside
// the stored range [kmin,kmax] (DC region, Nyquist) are not used.
bool refMean(const float* s, int kmin, int kmax, int k, int G, int R, float& mean, int& cnt) {
  float sum = 0.0f;
  int n = 0;
  for (int kk = k - G - R; kk <= k - G - 1; kk++)
    if (kk >= kmin && kk <= kmax) { sum += s[kk - kmin]; n++; }
  for (int kk = k + G + 1; kk <= k + G + R; kk++)
    if (kk >= kmin && kk <= kmax) { sum += s[kk - kmin]; n++; }
  if (n < 4) return false;
  mean = sum / static_cast<float>(n);
  cnt = n;
  return true;
}

// Parabolic interpolation of the log-power peak (accurate for the Hann window).
float refineHz(const float* s, int kmin, int kmax, int k, float df) {
  float delta = 0.0f;
  if (k - 1 >= kmin && k + 1 <= kmax) {
    const float a = logf(s[k - 1 - kmin] + 1e-12f);
    const float b = logf(s[k - kmin] + 1e-12f);
    const float c = logf(s[k + 1 - kmin] + 1e-12f);
    const float den = a - 2.0f * b + c;
    if (den < -1e-6f) {
      delta = 0.5f * (a - c) / den;
      if (delta > 0.5f) delta = 0.5f;
      if (delta < -0.5f) delta = -0.5f;
    }
  }
  return (static_cast<float>(k) + delta) * df;
}

// Ranking of a detection: vetted lines first, then lines the platform test could not vet,
// platform-like lines last; within a class by margin. In a vibrating scene the many
// platform-like lines must not crowd a real lead out of the short detection list.
inline float rankOf(const Detection& d) {
  return d.margin_db + (d.platform_like ? -200.0f : (d.tested ? 100.0f : 0.0f));
}

// Keep the kMaxDet best detections, sorted by rank.
void insertDet(Detection* list, uint8_t& n, const Detection& d) {
  int pos = n;
  while (pos > 0 && rankOf(list[pos - 1]) < rankOf(d)) pos--;
  if (pos >= kMaxDet) return;
  const int last = (n < kMaxDet) ? n : kMaxDet - 1;
  for (int i = last; i > pos; i--) list[i] = list[i - 1];
  list[pos] = d;
  if (n < kMaxDet) n++;
}

void analyzeCore(const SegmentBuffer& buf, int N, int start, const DspParams& p,
                 const Seed* seeds, int n_seeds, Workspace& ws,
                 SegmentResult& out, PreviewData* pv) {
  const int nb = buf.n_bins;
  const int b0 = p.min_bin;
  const int Nh = N / 2;
  const float df = p.fs_hz / static_cast<float>(N);
  const float invN = 1.0f / static_cast<float>(N);
  const int G = p.cfar_guard;
  const int R = std::min<int>(p.cfar_ref, kMaxRef);
  const float nf_scale = 0.75f * static_cast<float>(N);   // folded noise = 2 * sum(w^2) * sigma^2

  out.n_det = 0; out.n_platform = 0; out.n_untested = 0; out.confirmed = false; out.conf_seed = 0;
  out.platform_peak = false; out.platform_hz = 0.0f; out.theta_rms_rad = 0.0f; out.cpe_applied = false; out.cpe_bins = 0;
  out.floor_db = 0.0f; out.ref_bin = 0; out.ref_snr_db = 0.0f; out.n_cells = 0;

  if (ws.win_n != N) { hannWindow(ws.win, static_cast<uint16_t>(N)); ws.win_n = static_cast<uint16_t>(N); }

  // ---- A. static returns (per-bin complex mean) and the noise power
  for (int b = b0; b < nb; b++) {
    const IqSample* x = &buf.iq[b][start];
    float mi = 0.0f, mq = 0.0f;
    for (int n = 0; n < N; n++) { mi += x[n].i; mq += x[n].q; }
    mi *= invN; mq *= invN;
    ws.cmi[b] = mi; ws.cmq[b] = mq; ws.spw[b] = mi * mi + mq * mq;
    float acc = 0.0f;                       // second difference removes slow motion, keeps noise
    for (int n = 2; n < N; n++) {
      const float dr = float(x[n].i) - 2.0f * float(x[n - 1].i) + float(x[n - 2].i);
      const float di = float(x[n].q) - 2.0f * float(x[n - 1].q) + float(x[n - 2].q);
      acc += dr * dr + di * di;
    }
    ws.noise2[b] = acc / (6.0f * static_cast<float>(N - 2));
  }
  float sigma2;                             // complex noise power per sample (counts^2), median over bins
  {
    int m = 0;
    for (int b = b0; b < nb; b++) ws.floorv[m++] = ws.noise2[b];
    sigma2 = medianOf(ws.floorv, m) + 1e-6f;
  }
  if (sigma2 < p.min_noise_power) {          // dead, frozen or disconnected front-end
    out.valid = false;
    out.reason = InvalidReason::kNoSignal;
    return;
  }
  int bs = b0;
  for (int b = b0; b < nb; b++) if (ws.spw[b] > ws.spw[bs]) bs = b;
  out.ref_bin = static_cast<uint8_t>(bs);
  out.ref_snr_db = dbf(ws.spw[bs] / sigma2);

  // ---- B. common rotation of the scene: sum over bins of z * conj(mean)
  for (int n = 0; n < N; n++) { ws.sr[n] = 0.0f; ws.si[n] = 0.0f; }
  float W = 0.0f;
  for (int b = b0; b < nb; b++) {
    const IqSample* x = &buf.iq[b][start];
    const float cr = ws.cmi[b], ci = ws.cmq[b];
    W += ws.spw[b];
    for (int n = 0; n < N; n++) {
      const float zr = x[n].i, zi = x[n].q;
      ws.sr[n] += zr * cr + zi * ci;
      ws.si[n] += zi * cr - zr * ci;
    }
  }

  // Common phase seen by bin b = ML estimate from all bins EXCEPT b-gW .. b+gW (and, when
  // bx >= 0, the same around bx). Fills ws.theta / ws.cs / ws.sn and returns the static
  // power Wb it was built from.
  const int gW = p.cpe_guard_bins;
  auto commonPhase = [&](int b, float& Wb, int bx) -> bool {
    for (int n = 0; n < N; n++) { ws.cs[n] = ws.sr[n]; ws.sn[n] = ws.si[n]; }
    float lw = 0.0f;
    for (int j = b0; j < nb; j++) {
      if (abs(j - b) > gW && (bx < 0 || abs(j - bx) > gW)) continue;
      const IqSample* y = &buf.iq[j][start];
      const float cr = ws.cmi[j], ci = ws.cmq[j];
      lw += ws.spw[j];
      for (int n = 0; n < N; n++) {
        const float zr = y[n].i, zi = y[n].q;
        ws.cs[n] -= zr * cr + zi * ci;
        ws.sn[n] -= zi * cr - zr * ci;
      }
    }
    Wb = W - lw;
    if (!(Wb > 10.0f * sigma2)) return false;
    for (int n = 0; n < N; n++) {
      const float rr = ws.cs[n], ii = ws.sn[n];
      float mag = sqrtf(rr * rr + ii * ii);
      if (mag < 1e-9f) mag = 1e-9f;
      ws.theta[n] = atan2f(ii, rr);
      ws.cs[n] = rr / mag;
      ws.sn[n] = ii / mag;
    }
    return true;
  };

  // ---- spectral geometry
  const int k_lo = std::max(2, static_cast<int>(ceilf(p.resp_lo_hz / df - 1e-4f)));
  int k_hi = std::min(Nh - 1, static_cast<int>(floorf(p.resp_hi_hz / df + 1e-4f)));
  const int kmin = std::max(2, k_lo - G - R);
  int kmax = std::min(Nh - 1, k_hi + G + R);
  // Never search a smaller band than was asked for: a silently truncated band would make a
  // negative result mean less than it claims. Refuse the configuration instead.
  if (k_hi < k_lo || kmax - kmin + 1 > kMaxSpecCells) {
    out.valid = false;
    out.reason = InvalidReason::kConfig;
    return;
  }
  const int ncell = kmax - kmin + 1;
  const int n_bins_s = nb - b0;
  const int n_band = k_hi - k_lo + 1;

  // ---- C. per range bin: common phase, compensation, clutter removal, window, FFT, fold
  int cpe_bins = 0;
  for (int b = b0; b < nb; b++) {
    float Wb = 0.0f;
    const bool have = commonPhase(b, Wb, -1);
    ws.have_th[b] = have;
    ws.vis_db[b] = have ? dbf(2.0f * Wb / (ws.spw[b] + 1e-3f)) : -100.0f;
    bool comp = false;
    if (have) {
      comp = p.cpe_enable && Wb >= p.cpe_comp_ratio * ws.spw[b];
      float mt = 0.0f;                      // spectrum of this bin's common phase (platform test)
      for (int n = 0; n < N; n++) mt += ws.theta[n];
      mt *= invN;
      for (int n = 0; n < N; n++) { ws.re[n] = (ws.theta[n] - mt) * ws.win[n]; ws.im[n] = 0.0f; }
      fftComplex(ws.re, ws.im, static_cast<uint16_t>(N));
      for (int k = kmin; k <= kmax; k++) ws.pth[b][k - kmin] = ws.re[k] * ws.re[k] + ws.im[k] * ws.im[k];
      for (int c = 0; c < ncell; c++) ws.floorv[c] = ws.pth[b][c];
      ws.thth[b] = lowerQuartileMean(ws.floorv, ncell) * undb(p.coh_thr_db);
    }
    ws.comp_b[b] = comp;
    if (comp) cpe_bins++;

    const IqSample* x = &buf.iq[b][start];
    float mr = 0.0f, mi = 0.0f;
    for (int n = 0; n < N; n++) {
      float zr = x[n].i, zi = x[n].q;
      if (comp) {                           // z * exp(-j theta)
        const float c = ws.cs[n], s = ws.sn[n];
        const float tr = zr * c + zi * s;
        zi = zi * c - zr * s;
        zr = tr;
      }
      ws.re[n] = zr; ws.im[n] = zi; mr += zr; mi += zi;
    }
    mr *= invN; mi *= invN;
    for (int n = 0; n < N; n++) {
      ws.re[n] = (ws.re[n] - mr) * ws.win[n];
      ws.im[n] = (ws.im[n] - mi) * ws.win[n];
    }
    fftComplex(ws.re, ws.im, static_cast<uint16_t>(N));
    float* sp = ws.spec[b];
    for (int k = kmin; k <= kmax; k++) {
      sp[k - kmin] = ws.re[k] * ws.re[k] + ws.im[k] * ws.im[k] +
                     ws.re[N - k] * ws.re[N - k] + ws.im[N - k] * ws.im[N - k];
    }
  }
  out.cpe_bins = static_cast<uint8_t>(cpe_bins);
  out.cpe_applied = (2 * cpe_bins >= n_bins_s);

  // Platform-motion vetting of a bin line (bin b, cell k, SNR snr_db).
  //  testable : the test is sensitive enough - a platform line of this size would show up in
  //             the common phase with margin (visibility vis_db = how much louder a line is in
  //             the common phase than in this bin).
  //  platform : the common phase seen by this bin has a line within +-1 cell AND that line is
  //             big enough to account for the bin line (within coh_tol_db). A line that is far
  //             stronger than the common-phase line could have made it is not platform motion
  //             even if some leakage of a different, strong line sits at the same frequency.
  auto testable = [&](int b, float snr_db) {
    return ws.have_th[b] && (snr_db + ws.vis_db[b] >= p.coh_thr_db + p.coh_margin_db);
  };
  // Only a spectral PEAK counts as a line at cell k: the flank of a stronger line two or
  // three cells away (window main lobe / skirts) is not.
  auto isPeak = [&](const float* v, int kk) {
    return (kk - 1 < kmin || v[kk - 1 - kmin] <= v[kk - kmin]) && (kk + 1 > kmax || v[kk + 1 - kmin] <= v[kk - kmin]);
  };
  // Strongest line SNR (dB over its local CFAR floor) of bin j: peaks within +-1 cell of cell k.
  auto lineSnrDb = [&](int j, int k) {
    float best = -200.0f;
    for (int kk = std::max(k_lo, k - 1); kk <= std::min(k_hi, k + 1); kk++) {
      float fl; int cnt;
      if (!isPeak(ws.spec[j], kk) || !refMean(ws.spec[j], kmin, kmax, kk, G, R, fl, cnt)) continue;
      best = std::max(best, dbf(ws.spec[j][kk - kmin] / fl));
    }
    return best;
  };
  // Scene-wide line: present at this frequency in several bins that are too far away to be
  // the same person. Platform motion does that; a person does not.
  auto sceneWide = [&](int b, int k, float snr_db) {
    int cnt = 0;
    float far_max = -200.0f;
    for (int j = b0; j < nb; j++) {
      if (abs(j - b) < p.sw_dist) continue;
      const float v = lineSnrDb(j, k);
      far_max = std::max(far_max, v);
      if (v >= p.sw_snr_db) cnt++;
    }
    return cnt >= p.sw_min_bins && snr_db < far_max + p.sw_exempt_db;
  };
  auto platformExplains = [&](int b, int k, float snr_db) {
    float v = 0.0f;                                       // strongest common-phase PEAK within +-1 cell
    for (int kk = std::max(kmin, k - 1); kk <= std::min(kmax, k + 1); kk++)
      if (isPeak(ws.pth[b], kk)) v = std::max(v, ws.pth[b][kk - kmin]);
    if (!(v > ws.thth[b])) return false;
    const float line_db = dbf(v / ws.thth[b]) + p.coh_thr_db;     // common-phase line over its floor
    return snr_db <= line_db - ws.vis_db[b] + p.coh_tol_db;
  };

  // ---- D. CFAR search over all (range bin, frequency) cells
  out.n_cells = static_cast<uint16_t>(n_bins_s * n_band);
  AlphaTable at;
  at.init(p.pfa_segment / static_cast<float>(out.n_cells));
  Detection list[kMaxDet];
  uint8_t nl = 0, n_plat = 0;
  int nfl = 0;
  float bin_snr[kMaxBins];
  for (int b = 0; b < kMaxBins; b++) bin_snr[b] = -1e30f;

  for (int b = b0; b < nb; b++) {
    const float* sp = ws.spec[b];
    for (int k = k_lo; k <= k_hi; k++) {
      float fl; int cnt;
      if (!refMean(sp, kmin, kmax, k, G, R, fl, cnt)) continue;
      ws.floorv[nfl++] = fl;
      const float P = sp[k - kmin];
      if (P / fl > bin_snr[b]) bin_snr[b] = P / fl;
      const float thr = at.get(cnt) * fl;
      if (P <= thr) continue;
      if (k - 1 >= kmin && sp[k - 1 - kmin] > P) continue;      // keep local maxima only
      if (k + 1 <= kmax && sp[k + 1 - kmin] > P) continue;
      Detection d;
      d.bin = static_cast<uint8_t>(b);
      d.k = static_cast<uint16_t>(k);
      d.hz = refineHz(sp, kmin, kmax, k, df);
      d.snr_db = dbf(P / fl);
      d.margin_db = dbf(P / thr);
      d.tested = testable(b, d.snr_db);
      d.platform_like = false;
      if ((d.tested && platformExplains(b, k, d.snr_db)) || sceneWide(b, k, d.snr_db)) {
        if (!ws.comp_b[b]) { n_plat++; continue; }   // sensor motion itself, not a person
        d.tested = false;                            // compensated, but a residual cannot be excluded
        d.platform_like = true;
      }
      insertDet(list, nl, d);
    }
  }
  out.n_det = nl;
  for (int i = 0; i < nl; i++) { out.det[i] = list[i]; if (!list[i].tested) out.n_untested++; }
  out.n_platform = n_plat;
  if (nfl > 0) out.floor_db = dbf(medianOf(ws.floorv, nfl) / nf_scale);

  int bb = b0;                              // most promising range bin (also used by the preview)
  for (int b = b0; b < nb; b++) if (bin_snr[b] > bin_snr[bb]) bb = b;

  // Vibration notice: a line in the common phase of the scene as seen from the strongest
  // reflector, with the most promising bin (the likely person) left out as well.
  {
    float Wb = 0.0f;
    if (commonPhase(bs, Wb, bb)) {
      float mt = 0.0f;
      for (int n = 0; n < N; n++) mt += ws.theta[n];
      mt *= invN;
      float var = 0.0f;
      for (int n = 0; n < N; n++) { const float d = ws.theta[n] - mt; var += d * d; ws.re[n] = d * ws.win[n]; ws.im[n] = 0.0f; }
      out.theta_rms_rad = sqrtf(var * invN);
      fftComplex(ws.re, ws.im, static_cast<uint16_t>(N));
      float* pt = ws.floorv;                // reuse: cells kmin..kmax
      for (int k = kmin; k <= kmax; k++) pt[k - kmin] = ws.re[k] * ws.re[k] + ws.im[k] * ws.im[k];
      float tmp[kMaxSpecCells];
      for (int c = 0; c < ncell; c++) tmp[c] = pt[c];
      const float thr = lowerQuartileMean(tmp, ncell) * undb(p.coh_thr_db);
      float pk = 0.0f;
      int kpk = -1;
      for (int k = k_lo; k <= k_hi; k++) {
        const float v = pt[k - kmin];
        if (v > thr && v > pk && (k - 1 < kmin || pt[k - 1 - kmin] <= v) && (k + 1 > kmax || pt[k + 1 - kmin] <= v)) {
          pk = v; kpk = k;
        }
      }
      if (kpk >= 0) { out.platform_peak = true; out.platform_hz = refineHz(pt, kmin, kmax, kpk, df); }
    }
  }

  // ---- E. guided confirmation of seeds from earlier segments
  if (n_seeds > 0) {
    const int fw = static_cast<int>(floorf(p.freq_tol_hz / df + 1e-4f));
    const int fam = n_seeds * (2 * p.bin_tol + 1) * (2 * fw + 1);
    AlphaTable ac;
    ac.init(p.pfa_confirm / static_cast<float>(fam));
    float best = -1e30f;
    for (int s = 0; s < n_seeds; s++) {
      const int kc = static_cast<int>(lroundf(seeds[s].hz / df));
      const int bl = std::max(b0, static_cast<int>(seeds[s].bin) - p.bin_tol);
      const int bh = std::min(nb - 1, static_cast<int>(seeds[s].bin) + p.bin_tol);
      for (int b = bl; b <= bh; b++) {
        const float* sp = ws.spec[b];
        for (int k = std::max(k_lo, kc - fw); k <= std::min(k_hi, kc + fw); k++) {
          float fl; int cnt;
          if (!isPeak(sp, k) || !refMean(sp, kmin, kmax, k, G, R, fl, cnt)) continue;   // a line, not a flank
          const float P = sp[k - kmin];
          const float snr = dbf(P / fl);
          if (!testable(b, snr) || platformExplains(b, k, snr) || sceneWide(b, k, snr)) continue;   // cannot vet => cannot confirm
          const float m = dbf(P / (ac.get(cnt) * fl));
          if (m > best) {
            best = m;
            out.conf.bin = static_cast<uint8_t>(b);
            out.conf.k = static_cast<uint16_t>(k);
            out.conf.hz = refineHz(sp, kmin, kmax, k, df);
            out.conf.snr_db = snr;
            out.conf.margin_db = m;
            out.conf.tested = true;
            out.conf.platform_like = false;
            out.conf_seed = static_cast<uint8_t>(s);
          }
        }
      }
    }
    out.confirmed = best > 0.0f;
  }

  // ---- operator preview (best range bin by in-band SNR)
  if (pv) {
    pv->have = true;
    pv->best_bin = static_cast<uint8_t>(bb);
    pv->k0 = static_cast<uint16_t>(kmin);
    pv->n_cells = static_cast<uint16_t>(ncell);
    pv->df_hz = df;
    const float* sp = ws.spec[bb];
    for (int c = 0; c < ncell; c++) {
      const int k = kmin + c;
      pv->spec_db[c] = dbf(sp[c] / nf_scale);
      float fl; int cnt;
      pv->thr_db[c] = (k >= k_lo && k <= k_hi && refMean(sp, kmin, kmax, k, G, R, fl, cnt))
                          ? dbf(at.get(cnt) * fl / nf_scale) : -200.0f;
    }
    float Wb = 0.0f;
    const bool have = commonPhase(bb, Wb, -1);
    const bool comp = have && p.cpe_enable && Wb >= p.cpe_comp_ratio * ws.spw[bb];
    const int M = std::min(N, static_cast<int>(kPrevFrames));
    const int off = N - M;
    const IqSample* x = &buf.iq[bb][start + off];
    float mr = 0.0f, mi = 0.0f;
    for (int n = 0; n < M; n++) {
      float zr = x[n].i, zi = x[n].q;
      if (comp) {
        const float c = ws.cs[off + n], s = ws.sn[off + n];
        const float tr = zr * c + zi * s;
        zi = zi * c - zr * s;
        zr = tr;
      }
      ws.re[n] = zr; ws.im[n] = zi; mr += zr; mi += zi;
    }
    mr /= static_cast<float>(M); mi /= static_cast<float>(M);
    float sii = 0.0f, sqq = 0.0f, siq = 0.0f;
    for (int n = 0; n < M; n++) {
      ws.re[n] -= mr; ws.im[n] -= mi;
      sii += ws.re[n] * ws.re[n]; sqq += ws.im[n] * ws.im[n]; siq += ws.re[n] * ws.im[n];
    }
    const float ang = 0.5f * atan2f(2.0f * siq, sii - sqq);   // dominant axis of the I/Q cloud
    const float ca = cosf(ang), sa = sinf(ang);
    float mx = 1e-9f;
    for (int n = 0; n < M; n++) { pv->wave[n] = ws.re[n] * ca + ws.im[n] * sa; mx = std::max(mx, fabsf(pv->wave[n])); }
    for (int n = 0; n < M; n++) pv->wave[n] /= mx;
    pv->wave_n = static_cast<uint16_t>(M);
  }
}

}  // namespace

void analyzeSegment(const SegmentBuffer& buf, uint16_t n_avail, const DspParams& p,
                    const Seed* seeds, uint8_t n_seeds, Workspace& ws,
                    SegmentResult& out, PreviewData* pv) {
  memset(&out, 0, sizeof(out));
  if (pv) pv->have = false;
  if (n_avail > kSegFrames) n_avail = kSegFrames;

  const int N = (n_avail >= kSegFrames) ? kSegFrames : (n_avail >= kPrevFrames ? kPrevFrames : 0);
  out.n_used = static_cast<uint16_t>(N);
  if (N == 0) { out.reason = InvalidReason::kTooShort; return; }
  if (p.min_bin >= buf.n_bins || p.cfar_ref < 4 || p.fs_hz <= 0.0f) { out.reason = InvalidReason::kConfig; return; }
  const int start = static_cast<int>(n_avail) - N;

  // ---- frame-quality gate
  int bad = 0, mot = 0, sat = 0, tim = 0, drop = 0, imu_miss = 0;
  for (int i = 0; i < N; i++) {
    const uint8_t f = buf.flags[start + i];
    if (f & kFlagImuMotion) mot++;
    if (f & kFlagSaturated) sat++;
    if (f & kFlagTiming) tim++;
    if (f & (kFlagImuMotion | kFlagSaturated | kFlagTiming)) bad++;
    if (f & kFlagDropped) drop++;
    if (f & kFlagImuMissing) imu_miss++;
  }
  out.bad_frac = static_cast<float>(bad) / static_cast<float>(N);
  out.drop_frac = static_cast<float>(drop) / static_cast<float>(N);
  out.imu_missing_frac = static_cast<float>(imu_miss) / static_cast<float>(N);
  if (p.require_imu && out.imu_missing_frac > 0.5f) { out.reason = InvalidReason::kImuMissing; return; }
  // Too many unusable frames: say what the operator has to fix. Clipping needs less gain, not a firmer grip;
  // a time base that does not match the announced frame rate needs the front-end configuration fixed.
  if (out.bad_frac > p.max_bad_frac) {
    if (tim > sat && tim > mot)  out.reason = InvalidReason::kTiming;
    else                         out.reason = (sat > mot) ? InvalidReason::kSaturated : InvalidReason::kMotion;
    return;
  }
  if (out.drop_frac > p.max_drop_frac)             { out.reason = InvalidReason::kDropout;    return; }
  out.valid = true;
  out.reason = InvalidReason::kNone;

  analyzeCore(buf, N, start, p, seeds, n_seeds, ws, out, pv);
}

}  // namespace rs
