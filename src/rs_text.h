// RubbleSense firmware - operator text (English / Indonesian).
//
// Rules the text must keep (checked by the host harness):
//   * no verdict text may read as "all clear"; the negative verdict carries the mandatory
//     sentence "No reliable life-sign pattern detected in this scan. DO NOT clear area."
//   * every T_F_* format string has the same conversion specifiers in both languages
//   * plain ASCII only (the built-in 5x7 TFT font has no accents)
#pragma once
#include <stdint.h>
#include "rs_decision.h"

namespace txt {

enum class Lang : uint8_t { EN = 0, ID = 1 };

void  setLang(Lang l);
Lang  lang();

enum Id : uint8_t {
  T_SUBTITLE,
  T_LBL_TRIAL, T_LBL_SOURCE,
  T_SRC_WAITING, T_SRC_CONNECTED,
  T_IDLE_1, T_IDLE_2, T_IDLE_3, T_IDLE_POTS,
  T_IMU, T_IMU_OK, T_IMU_MOVING, T_IMU_MISSING, T_IMU_SIM, T_IMU_HOST,
  T_BASELINE, T_BASELINE_NONE, T_BASELINE_SIMDEF,
  T_LOG, T_LOG_OFF, T_SOUND_ON, T_SOUND_OFF,
  T_SAFETY_1, T_SAFETY_2,
  T_HINT_IDLE, T_HINT_SCAN, T_HINT_RESULT, T_HINT_MENU, T_HINT_ANY,
  T_CALIB_TITLE, T_LIVE_NOT_RESULT, T_HOLD_STEADY, T_ANALYZING, T_SETTLING, T_STALLED, T_COLLECTING,
  T_NEGATIVE_NOTE,                 // shown under every negative / inconclusive result
  T_VIB_NOTE, T_VIB_COMP,
  T_SENS_UNKNOWN, T_SENS_OK, T_SENS_LOW,
  T_CALIB_OK, T_CALIB_FAIL, T_CALIB_HINT,
  T_MENU_TITLE, T_MENU_SOURCE, T_MENU_CALIB, T_MENU_CLEARBL, T_MENU_LANG, T_MENU_SOUND, T_MENU_SELFTEST, T_MENU_ABOUT,
  T_ON, T_OFF,
  T_MSG_NOT_READY, T_MSG_ABORTED, T_MSG_SRC_SIM, T_MSG_SRC_SER, T_MSG_BL_CLEARED, T_MSG_BASE_NEED,
  T_ST_TITLE, T_ST_IMU, T_ST_STORAGE, T_ST_RADAR, T_ST_MEM, T_ST_OK, T_ST_FAIL,
  T_ABOUT_1, T_ABOUT_2, T_ABOUT_3, T_ABOUT_4,
  // format strings (same conversion specifiers in both languages)
  T_F_SCAN_HDR,        // "SCAN #%lu  SEG %u  %02u:%02u"                      (ulong, uint, uint, uint)
  T_F_CALIB_HDR,       // "CALIBRATING  %02u:%02u"                            (uint, uint)
  T_F_FRAMES,          // "frames %u/%u  drops %u"                            (uint, uint, uint)
  T_F_EVID_RANGE,      // "Range ~%.1f m (air-eq.)   Rate %.1f /min"          (double, double)
  T_F_EVID_SNR,        // "SNR %.1f dB  Margin %.1f dB  Segments %u"          (double, double, uint)
  T_F_FLOOR_BASE,      // "Noise floor %.1f dB, baseline %.1f (%+.1f dB)"     (double, double, double)
  T_F_FLOOR_ONLY,      // "Noise floor %.1f dB"                               (double)
  T_F_VIB,             // "Platform vibration seen at %.2f Hz, %.2f mm rms"   (double, double)
  T_F_BASELINE,        // "%.1f dB"                                           (double)
  T_F_LOG,             // "%lu scans"                                         (ulong)
  T_F_CALIB_VAL,       // "Baseline stored: %.1f dB"                          (double)
  T_COUNT
};

const char* t(Id id);                         // current language
const char* tEn(Id id);                       // English (logs, JSON)

// Verdict texts. Headline is shown at text size 2 (<= 25 characters per line, wrapped).
const char* verdictHeadline(rs::Verdict v);
const char* verdictAction(rs::Verdict v, rs::InvalidReason why);
const char* verdictHeadlineEn(rs::Verdict v);
const char* verdictActionEn(rs::Verdict v, rs::InvalidReason why);

// Second banner line of the negative verdict ("DO NOT clear area.").
const char* negativeBannerLine();
const char* negativeBannerLineEn();

// The sentence the operator must always be shown with a negative result (English, mandatory).
const char* mandatoryNegativeEn();

// Trial preset display name for the current language ("" if idx is out of range).
const char* presetName(uint8_t idx);

}  // namespace txt
