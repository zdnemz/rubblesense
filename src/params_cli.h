// RubbleSense firmware - named access to the tunable DSP parameters (console get / set).
//
// Only parameters that are safe to change are exposed. The safety gates (require_imu, the
// data-quality limits, min_noise_power, the platform tests) are deliberately NOT reachable
// from the console: switching them off would let a scan claim more than it can know.
#pragma once
#include <stddef.h>
#include "rs_config.h"

class Print;

struct ParamInfo {
  const char* name;
  const char* help;
};

size_t paramCount();
const ParamInfo& paramInfo(size_t i);

// Format the value of `name` into out. False if the name is unknown.
bool paramGet(const rs::DspParams& p, const char* name, char* out, size_t cap);

// Parse `value` and store it in p (no range check beyond the type: validate with paramsValid()).
// On failure `err` points to a short static reason.
bool paramSet(rs::DspParams& p, const char* name, const char* value, const char** err);

// Print "name = value  (help)" for every parameter.
void paramList(const rs::DspParams& p, Print& out);
