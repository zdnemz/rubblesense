#include "rs_text.h"
#include "radar_sim.h"

namespace txt {
namespace {

struct S { const char* en; const char* id; };

// Order must match enum Id (checked by a static_assert on the table size and by the host harness).
const S kText[T_COUNT] = {
  /* T_SUBTITLE        */ {"Support tool only - follow rescue protocol", "Alat bantu saja - ikuti protokol SAR"},
  /* T_LBL_TRIAL       */ {"TRIAL:", "UJI:"},
  /* T_LBL_SOURCE      */ {"SOURCE:", "SUMBER:"},
  /* T_SRC_WAITING     */ {"serial I/Q: waiting for front-end info", "serial I/Q: menunggu info front-end"},
  /* T_SRC_CONNECTED   */ {"serial I/Q: front-end connected", "serial I/Q: front-end terhubung"},
  /* T_IDLE_1          */ {"1. Press the sensor firmly on the barrier.", "1. Tempelkan sensor kuat pada penghalang."},
  /* T_IDLE_2          */ {"2. Hold still. A scan takes about 50 s.", "2. Diam. Satu scan sekitar 50 detik."},
  /* T_IDLE_3          */ {"3. GREEN = scan, BLUE = trial, WHITE = menu.", "3. HIJAU = scan, BIRU = uji, PUTIH = menu."},
  /* T_IDLE_POTS       */ {"MANUAL: top slider = target, bottom = noise", "MANUAL: slider atas = target, bawah = derau"},
  /* T_IMU             */ {"IMU:", "IMU:"},
  /* T_IMU_OK          */ {"stable", "stabil"},
  /* T_IMU_MOVING      */ {"MOVING", "BERGERAK"},
  /* T_IMU_MISSING     */ {"MISSING - scans invalid", "TIDAK ADA - scan tidak valid"},
  /* T_IMU_SIM         */ {"simulated", "simulasi"},
  /* T_IMU_HOST        */ {"from front-end", "dari front-end"},
  /* T_BASELINE        */ {"Baseline:", "Baseline:"},
  /* T_BASELINE_NONE   */ {"NOT SET - sensitivity unknown", "BELUM ADA - sensitivitas tak diketahui"},
  /* T_BASELINE_SIMDEF */ {"(sim default)", "(bawaan sim)"},
  /* T_LOG             */ {"Log:", "Log:"},
  /* T_LOG_OFF         */ {"storage off", "penyimpanan mati"},
  /* T_SOUND_ON        */ {"Sound: ON", "Suara: AKTIF"},
  /* T_SOUND_OFF       */ {"Sound: OFF", "Suara: MATI"},
  /* T_SAFETY_1        */ {"SAFETY: a negative scan does NOT mean", "KESELAMATAN: hasil negatif BUKAN berarti"},
  /* T_SAFETY_2        */ {"nobody is there. Aid only - verify on site.", "tidak ada orang. Alat bantu - verifikasi."},
  /* T_HINT_IDLE       */ {"GREEN scan | BLUE trial | WHITE menu", "HIJAU scan | BIRU uji | PUTIH menu"},
  /* T_HINT_SCAN       */ {"GREEN: abort scan", "HIJAU: batalkan scan"},
  /* T_HINT_RESULT     */ {"GREEN rescan | BLUE trial | WHITE menu", "HIJAU ulang | BIRU uji | PUTIH menu"},
  /* T_HINT_MENU       */ {"BLUE next | GREEN select | WHITE back", "BIRU lanjut | HIJAU pilih | PUTIH kembali"},
  /* T_HINT_ANY        */ {"press any button", "tekan tombol apa saja"},
  /* T_CALIB_TITLE     */ {"BASELINE CALIBRATION", "KALIBRASI BASELINE"},
  /* T_LIVE_NOT_RESULT */ {"LIVE VIEW - NOT A RESULT", "TAMPILAN LANGSUNG - BUKAN HASIL"},
  /* T_HOLD_STEADY     */ {"Hold the sensor steady. Do not touch it.", "Tahan sensor diam. Jangan disentuh."},
  /* T_ANALYZING       */ {"analysing segment...", "menganalisis segmen..."},
  /* T_SETTLING        */ {"settling IMU filters...", "menstabilkan filter IMU..."},
  /* T_STALLED         */ {"NO RADAR DATA - check the link", "TIDAK ADA DATA RADAR - cek koneksi"},
  /* T_COLLECTING      */ {"collecting data...", "mengumpulkan data..."},
  /* T_NEGATIVE_NOTE   */ {"No pattern is NOT proof that nobody is there.", "Tanpa pola BUKAN bukti tidak ada orang."},
  /* T_VIB_NOTE        */ {"Sensor platform vibrates (IMU cannot see it)", "Platform sensor bergetar (IMU tak bisa lihat)"},
  /* T_VIB_COMP        */ {"compensated in the radar signal", "dikompensasi pada sinyal radar"},
  /* T_SENS_UNKNOWN    */ {"Baseline not set: sensitivity unknown", "Baseline belum ada: sensitivitas tak diketahui"},
  /* T_SENS_OK         */ {"sensitivity as at calibration", "sensitivitas setara saat kalibrasi"},
  /* T_SENS_LOW        */ {"sensitivity LOST", "sensitivitas HILANG"},
  /* T_CALIB_OK        */ {"CALIBRATION STORED", "KALIBRASI TERSIMPAN"},
  /* T_CALIB_FAIL      */ {"CALIBRATION FAILED", "KALIBRASI GAGAL"},
  /* T_CALIB_HINT      */ {"Calibrate in a quiet, empty reference spot.", "Kalibrasi di titik rujukan kosong yang tenang."},
  /* T_MENU_TITLE      */ {"MENU", "MENU"},
  /* T_MENU_SOURCE     */ {"Source", "Sumber"},
  /* T_MENU_CALIB      */ {"Calibrate baseline", "Kalibrasi baseline"},
  /* T_MENU_CLEARBL    */ {"Clear baseline", "Hapus baseline"},
  /* T_MENU_LANG       */ {"Language", "Bahasa"},
  /* T_MENU_SOUND      */ {"Sound", "Suara"},
  /* T_MENU_SELFTEST   */ {"Self-test", "Uji mandiri"},
  /* T_MENU_ABOUT      */ {"About / safety", "Tentang / keselamatan"},
  /* T_ON              */ {"ON", "AKTIF"},
  /* T_OFF             */ {"OFF", "MATI"},
  /* T_MSG_NOT_READY   */ {"Radar source not ready", "Sumber radar belum siap"},
  /* T_MSG_ABORTED     */ {"Scan aborted", "Scan dibatalkan"},
  /* T_MSG_SRC_SIM     */ {"Source: SIM", "Sumber: SIM"},
  /* T_MSG_SRC_SER     */ {"Source: SERIAL I/Q", "Sumber: SERIAL I/Q"},
  /* T_MSG_BL_CLEARED  */ {"Baseline cleared", "Baseline dihapus"},
  /* T_MSG_BASE_NEED   */ {"Calibrate a baseline first", "Kalibrasi baseline dulu"},
  /* T_ST_TITLE        */ {"SELF-TEST", "UJI MANDIRI"},
  /* T_ST_IMU          */ {"IMU (MPU6050)", "IMU (MPU6050)"},
  /* T_ST_STORAGE      */ {"Log storage", "Penyimpanan log"},
  /* T_ST_RADAR        */ {"Radar source", "Sumber radar"},
  /* T_ST_MEM          */ {"Free heap", "Heap bebas"},
  /* T_ST_OK           */ {"OK", "OK"},
  /* T_ST_FAIL         */ {"FAIL", "GAGAL"},
  /* T_ABOUT_1         */ {"RubbleSense SAR aid - research prototype", "RubbleSense alat bantu SAR - prototipe riset"},
  /* T_ABOUT_2         */ {"NOT validated on real rubble. Never use a", "BELUM divalidasi pada puing nyata. Jangan"},
  /* T_ABOUT_3         */ {"negative scan to declare an area clear.", "pakai hasil negatif untuk menyatakan area aman."},
  /* T_ABOUT_4         */ {"Confirm every lead by other means.", "Konfirmasi setiap petunjuk dengan cara lain."},
  /* T_F_SCAN_HDR      */ {"SCAN #%lu  SEG %u  %02u:%02u", "SCAN #%lu  SEG %u  %02u:%02u"},
  /* T_F_CALIB_HDR     */ {"CALIBRATING  %02u:%02u", "KALIBRASI  %02u:%02u"},
  /* T_F_FRAMES        */ {"frames %u/%u  drops %u", "frame %u/%u  hilang %u"},
  /* T_F_EVID_RANGE    */ {"Range ~%.1f m (air-eq.)  Rate %.1f /min", "Jarak ~%.1f m (setara udara) Laju %.1f/mnt"},
  /* T_F_EVID_SNR      */ {"SNR %.1f dB  Margin %.1f dB  Segments %u", "SNR %.1f dB  Margin %.1f dB  Segmen %u"},
  /* T_F_FLOOR_BASE    */ {"Noise floor %.1f dB, baseline %.1f (%+.1f dB)", "Lantai derau %.1f dB, baseline %.1f (%+.1f)"},
  /* T_F_FLOOR_ONLY    */ {"Noise floor %.1f dB", "Lantai derau %.1f dB"},
  /* T_F_VIB           */ {"Platform vibration %.2f Hz, %.2f mm rms", "Getaran platform %.2f Hz, %.2f mm rms"},
  /* T_F_BASELINE      */ {"%.1f dB", "%.1f dB"},
  /* T_F_LOG           */ {"%lu scans", "%lu scan"},
  /* T_F_CALIB_VAL     */ {"Baseline stored: %.1f dB", "Baseline tersimpan: %.1f dB"},
};
static_assert(sizeof(kText) / sizeof(kText[0]) == T_COUNT, "text table does not match enum Id");

Lang g_lang = Lang::EN;

const char* pick(const S& s, bool id) { return id ? s.id : s.en; }

const S kHead[] = {
  /* kNone           */ {"SCANNING", "MEMINDAI"},
  /* kInvalid        */ {"INVALID SCAN - RESCAN", "SCAN TIDAK VALID - ULANGI"},
  /* kConfounded     */ {"SIGNAL MAY BE PLATFORM MOTION", "SINYAL MUNGKIN GETARAN ALAT"},
  /* kLowSensitivity */ {"LOW SENSITIVITY - INCONCLUSIVE", "SENSITIVITAS RENDAH - TIDAK PASTI"},
  /* kNoPattern      */ {"No reliable life-sign pattern detected in this scan.", "Tidak ada pola tanda-hidup yang andal pada scan ini."},
  /* kPossible       */ {"POSSIBLE PATTERN - NOT CONFIRMED", "KEMUNGKINAN POLA - BELUM TERKONFIRMASI"},
  /* kCandidate      */ {"CANDIDATE LIFE-SIGN PATTERN", "POLA TANDA-HIDUP KANDIDAT"},
};

const S kAct[] = {
  /* kNone           */ {"", ""},
  /* kInvalid        */ {"", ""},   // reason specific, see below
  /* kConfounded     */ {"A periodic line matches sensor/platform vibration. Clamp the sensor to solid ground and rescan. Do not clear area.",
                         "Garis periodik sama dengan getaran sensor/platform. Jepit sensor pada bidang kokoh lalu scan ulang. Jangan nyatakan area aman."},
  /* kLowSensitivity */ {"Noise is far above the calibrated level: a negative here means NOTHING. Reduce interference or move, then rescan. Do not clear area.",
                         "Derau jauh melebihi kalibrasi: hasil negatif TIDAK berarti apa-apa. Kurangi gangguan atau pindah, scan ulang. Jangan nyatakan area aman."},
  /* kNoPattern      */ {"Absence of a pattern is NOT absence of a survivor. Rescan from other positions and use other methods.",
                         "Tidak adanya pola BUKAN tidak adanya korban. Scan ulang dari posisi lain dan gunakan metode lain."},
  /* kPossible       */ {"Rescan from the same spot, then from a 2nd position. Do not clear area.",
                         "Scan ulang di titik yang sama, lalu dari posisi ke-2. Jangan nyatakan area aman."},
  /* kCandidate      */ {"Confirm with a 2nd position and audio / canine / camera. This is NOT proof of life.",
                         "Konfirmasi dari posisi ke-2 dan audio / anjing pelacak / kamera. Ini BUKAN bukti kehidupan."},
};

const S kInvalidAct[] = {
  /* kNone       */ {"Not enough valid data. Hold the sensor steady and rescan.", "Data valid kurang. Tahan sensor diam lalu scan ulang."},
  /* kTooShort   */ {"Not enough data was collected. Rescan.", "Data yang terkumpul kurang. Scan ulang."},
  /* kMotion     */ {"The sensor moved during the scan. Fix it firmly (clamp / brace) and rescan.", "Sensor bergerak saat scan. Pasang kokoh (jepit / sandarkan) lalu scan ulang."},
  /* kDropout    */ {"The radar link lost frames. Check the cable / front-end and rescan.", "Link radar kehilangan frame. Cek kabel / front-end lalu scan ulang."},
  /* kImuMissing */ {"No IMU data: sensor stability cannot be verified, so the scan is rejected.", "Tidak ada data IMU: kestabilan sensor tak bisa diverifikasi, scan ditolak."},
  /* kConfig     */ {"Invalid DSP configuration. Check the parameters (console: get / set).", "Konfigurasi DSP tidak valid. Cek parameter (console: get / set)."},
  /* kNoSignal   */ {"No radar signal: front-end dead, frozen or disconnected.", "Tidak ada sinyal radar: front-end mati, beku atau terlepas."},
  /* kSaturated  */ {"The radar input is clipping (ADC saturated). Reduce the gain or move away from the strong reflector and rescan.", "Input radar terpotong (ADC jenuh). Kurangi gain atau jauhi pemantul kuat lalu scan ulang."},
  /* kTiming     */ {"The front-end's timestamps do not match the frame rate it announced. Check its frame-rate setting and rescan.", "Cap waktu front-end tidak cocok dengan laju frame yang diumumkan. Cek pengaturan laju frame-nya lalu scan ulang."},
};
static_assert(sizeof(kInvalidAct) / sizeof(kInvalidAct[0]) == static_cast<size_t>(rs::InvalidReason::kCount),
              "every InvalidReason needs an operator text");

int vi(rs::Verdict v) {
  const int i = static_cast<int>(v);
  return (i >= 0 && i <= static_cast<int>(rs::Verdict::kCandidate)) ? i : 0;
}

const S& invalidEntry(rs::InvalidReason r) {
  const int i = static_cast<int>(r);
  const int n = static_cast<int>(sizeof(kInvalidAct) / sizeof(kInvalidAct[0]));
  return kInvalidAct[(i >= 0 && i < n) ? i : 0];
}

}  // namespace

void setLang(Lang l) { g_lang = l; }
Lang lang() { return g_lang; }

const char* t(Id id) { return (id < T_COUNT) ? pick(kText[id], g_lang == Lang::ID) : ""; }
const char* tEn(Id id) { return (id < T_COUNT) ? kText[id].en : ""; }

const char* verdictHeadline(rs::Verdict v) { return pick(kHead[vi(v)], g_lang == Lang::ID); }
const char* verdictHeadlineEn(rs::Verdict v) { return kHead[vi(v)].en; }

const char* verdictAction(rs::Verdict v, rs::InvalidReason why) {
  if (v == rs::Verdict::kInvalid) return pick(invalidEntry(why), g_lang == Lang::ID);
  return pick(kAct[vi(v)], g_lang == Lang::ID);
}
const char* verdictActionEn(rs::Verdict v, rs::InvalidReason why) {
  if (v == rs::Verdict::kInvalid) return invalidEntry(why).en;
  return kAct[vi(v)].en;
}

const char* negativeBannerLine() { return (g_lang == Lang::ID) ? "JANGAN NYATAKAN AREA AMAN" : "DO NOT CLEAR AREA."; }
const char* negativeBannerLineEn() { return "DO NOT CLEAR AREA."; }

const char* mandatoryNegativeEn() { return "No reliable life-sign pattern detected in this scan. DO NOT clear area."; }

const char* presetName(uint8_t idx) {
  if (idx >= kNumPresets) return "";
  const TrialPreset& p = trialPreset(idx);
  return (g_lang == Lang::ID) ? p.name_id : p.name_en;
}

}  // namespace txt
