#include "params_cli.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace {

enum class Kind : uint8_t { kFloat, kUint, kBool };

struct Entry {
  ParamInfo info;
  Kind      kind;
  size_t    offset;                 // into DspParams
};

// Table of the exposed parameters.
const Entry kTable[] = {
  {{"pfa",     "family-wise false-alarm probability of one segment search (1e-6..0.2)"}, Kind::kFloat, offsetof(rs::DspParams, pfa_segment)},
  {{"pfc",     "false-alarm probability of the guided confirmation (1e-6..0.2)"},         Kind::kFloat, offsetof(rs::DspParams, pfa_confirm)},
  {{"lo",      "lower edge of the respiration search band, Hz (>= 0.03)"},                Kind::kFloat, offsetof(rs::DspParams, resp_lo_hz)},
  {{"hi",      "upper edge of the respiration search band, Hz (<= 1.5 and <= 0.45 fs)"},  Kind::kFloat, offsetof(rs::DspParams, resp_hi_hz)},
  {{"minbin",  "first range bin searched (antenna leakage below), 0..8"},                 Kind::kUint,  offsetof(rs::DspParams, min_bin)},
  {{"cpe",     "platform-vibration compensation on (1) / off (0)"},                        Kind::kBool,  offsetof(rs::DspParams, cpe_enable)},
  {{"guard",   "range bins left out around a bin when estimating platform phase, 0..4"},  Kind::kUint,  offsetof(rs::DspParams, cpe_guard_bins)},
  {{"lowsens", "noise rise over baseline (dB) that makes a negative inconclusive"},       Kind::kFloat, offsetof(rs::DspParams, low_sens_db)},
};
constexpr size_t kN = sizeof(kTable) / sizeof(kTable[0]);

const Entry* find(const char* name) {
  for (size_t i = 0; i < kN; i++)
    if (strcasecmp(kTable[i].info.name, name) == 0) return &kTable[i];
  return nullptr;
}

}  // namespace

size_t paramCount() { return kN; }
const ParamInfo& paramInfo(size_t i) { return kTable[i < kN ? i : 0].info; }

bool paramGet(const rs::DspParams& p, const char* name, char* out, size_t cap) {
  const Entry* e = find(name);
  if (!e || cap == 0) return false;
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&p) + e->offset;
  switch (e->kind) {
    case Kind::kFloat: snprintf(out, cap, "%g", static_cast<double>(*reinterpret_cast<const float*>(base))); break;
    case Kind::kUint:  snprintf(out, cap, "%u", static_cast<unsigned>(*base)); break;
    case Kind::kBool:  snprintf(out, cap, "%u", (*reinterpret_cast<const bool*>(base)) ? 1u : 0u); break;
  }
  return true;
}

bool paramSet(rs::DspParams& p, const char* name, const char* value, const char** err) {
  const char* msg = nullptr;
  const Entry* e = find(name);
  if (!e) { if (err) *err = "unknown parameter"; return false; }
  if (!value || !value[0]) { if (err) *err = "missing value"; return false; }
  uint8_t* base = reinterpret_cast<uint8_t*>(&p) + e->offset;
  char* end = nullptr;
  switch (e->kind) {
    case Kind::kFloat: {
      const float v = strtof(value, &end);
      if (end == value || *end != '\0' || !isfinite(v)) msg = "not a number";
      else *reinterpret_cast<float*>(base) = v;
      break;
    }
    case Kind::kUint: {
      const long v = strtol(value, &end, 10);
      if (end == value || *end != '\0' || v < 0 || v > 255) msg = "not an integer 0..255";
      else *base = static_cast<uint8_t>(v);
      break;
    }
    case Kind::kBool: {
      if (!strcmp(value, "1") || !strcasecmp(value, "on") || !strcasecmp(value, "true")) *reinterpret_cast<bool*>(base) = true;
      else if (!strcmp(value, "0") || !strcasecmp(value, "off") || !strcasecmp(value, "false")) *reinterpret_cast<bool*>(base) = false;
      else msg = "use 0/1/on/off";
      break;
    }
  }
  if (err) *err = msg;
  return msg == nullptr;
}

void paramList(const rs::DspParams& p, Print& out) {
  char v[24];
  for (size_t i = 0; i < kN; i++) {
    paramGet(p, kTable[i].info.name, v, sizeof(v));
    out.print("# ");
    out.print(kTable[i].info.name);
    out.print(" = ");
    out.print(v);
    out.print("   (");
    out.print(kTable[i].info.help);
    out.println(")");
  }
}
