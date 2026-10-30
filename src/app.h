// RubbleSense firmware - application: scan state machine, operator interaction, logging.
//
// Threads:  update()          UI thread (Arduino loop): buttons, console, screens, state machine
//           backgroundStep()  acquisition thread: UART pump + IMU + radar polling
// The host test-bench calls both from one thread with a virtual clock.
#pragma once
#include <stdint.h>
#include "acq.h"
#include "alerts.h"
#include "buttons.h"
#include "console.h"
#include "dsp_worker.h"
#include "imu_hw.h"
#include "radar_serial.h"
#include "radar_sim.h"
#include "rs_decision.h"
#include "rx_pump.h"
#include "scan_json.h"
#include "settings.h"
#include "storage.h"
#include "ui.h"
#include <Adafruit_ILI9341.h>

class App : public ConsoleHost {
 public:
  enum class St : uint8_t { kIdle, kScanning, kCalibrating, kResult, kMenu, kSelfTest, kAbout, kMessage, kFatal };

  App();
  ~App();

  bool begin();
  void update();                       // UI thread, every ~5 ms
  void backgroundStep();               // acquisition thread, every ~1 ms

  // ---- read access (test-bench, status) ------------------------------------------------
  St                     state() const { return st_; }
  const rs::ScanOutcome& lastOutcome() const { return last_out_; }
  const char*            lastJson() const { return json_; }
  uint32_t               scanCount() const { return settings_.get().scan_counter; }
  SimSource&             sim() { return sim_; }
  Ui&                    ui() { return ui_; }
  Alerts&                alerts() { return alerts_; }
  Acquisition&           acq() { return acq_; }
  SettingsStore&         settings() { return settings_; }
  Storage&               storage() { return storage_; }
  DspWorker&             dsp() { return dsp_; }
  bool                   rebootRequested() const { return reboot_req_; }
  bool                   sourceReady() { return activeSource()->ready(); }

  // ---- operator actions (buttons and console) --------------------------------------------
  bool startScan(bool calibration);
  void abortScan();
  bool selectSource(SourceKind k);
  bool selectPreset(uint8_t idx);

  // ---- ConsoleHost -----------------------------------------------------------------------
  void conStatus(Print& out) override;
  bool conScan(Print& out) override;
  void conAbort(Print& out) override;
  bool conCalibrate(Print& out) override;
  void conClearBaseline(Print& out) override;
  bool conSource(Print& out, SourceKind k) override;
  bool conPreset(Print& out, int idx) override;
  void conLang(Print& out, uint8_t lang) override;
  void conSound(Print& out, bool on) override;
  void conLogDump(Print& out) override;
  bool conLogClear(Print& out) override;
  void conParams(Print& out) override;
  bool conGet(Print& out, const char* name) override;
  bool conSet(Print& out, const char* name, const char* value) override;
  void conSelfTest(Print& out) override;
  void conFactoryReset(Print& out) override;
  void conReboot() override;

 private:
  IRadarSource* activeSource();
  const char*   sourceTag() const;
  void          routeInput(SourceKind k);          // UART bytes -> I/Q ring (SERIAL) and console noise policy
  void configureAcq();
  void baselineFor(rs::DspParams& p, const char*& src_name) const;
  bool baselineKnown(float& db, bool& sim_default) const;

  void enterIdle(const char* message = "");
  void drawIdle();
  void refreshIdle(uint32_t now);
  IdleView idleView(const char* message);
  ImuState imuState(const AcqStatus& s) const;
  void pollManualPots(uint32_t now);

  void handleButton(const BtnEvent& e);
  void openMenu();
  void drawMenu();
  void activateMenu();
  void buildSelfTest(SelfTestView& v);
  void showMessage(const char* title, const char* text, uint16_t color);

  void stepScan(uint32_t now);
  void onSegment(const rs::SegmentResult& r);
  void finishScan();
  void finishCalibration(const rs::SegmentResult& r);
  void armNextSegment();
  void stopScanCleanly();
  void waitAcqIdle();
  void feedConsole();
  void printBanner();
  void fatal(const char* what);
  void applyVerdictAlerts(rs::Verdict v);

  // modules
  Adafruit_ILI9341 tft_;
  Ui               ui_;
  Buttons          buttons_;
  Alerts           alerts_;
  ImuHw            imu_;
  SettingsStore    settings_;
  Storage          storage_;
  RxPump           pump_;
  SimSource        sim_;
  SerialIqSource   ser_;
  Acquisition      acq_;
  DspWorker        dsp_;
  Console          console_;
  rs::ScanSession  session_;

  rs::SegmentBuffer* buf_ = nullptr;
  rs::Workspace*     ws_ = nullptr;

  // state
  St       st_ = St::kIdle;
  bool     settings_ok_ = false;
  bool     imu_ok_ = false;
  bool     reboot_req_ = false;
  uint8_t  menu_sel_ = 0;
  char     message_[48] = "";
  uint32_t message_until_ = 0;

  // current scan
  rs::RadarInfo  info_{};
  rs::DspParams  params_;
  const char*    baseline_src_ = "none";
  uint32_t       scan_id_ = 0;
  uint8_t        seg_no_ = 1;
  uint32_t       t_scan_ms_ = 0;
  bool           analyzing_ = false;
  uint32_t       analysis_t0_ = 0;
  bool           preview_pending_ = false;
  uint16_t       last_preview_frames_ = 0;
  uint32_t       last_preview_ms_ = 0;
  uint32_t       ui_ms_ = 0;
  uint32_t       idle_ms_ = 0;
  uint32_t       pot_ms_ = 0;
  float          pot_amp_ = 0.0f, pot_noise_ = 8.0f;
  bool           pots_init_ = false;
  rs::PreviewData pv_;
  rs::ScanOutcome last_out_{};
  char           json_[kScanJsonMax];
};
