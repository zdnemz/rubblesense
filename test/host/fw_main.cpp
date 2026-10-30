// Firmware host test-bench: runs the unmodified firmware sources (src/*.cpp) on the host under a virtual
// clock with Arduino / ESP32 stubs, and writes a markdown report.
//
//   fw_harness [--only <substring>] [--list] [--report <file.md>] [--screens <dir>] [--log-out <file.jsonl>]
//
// Exit status 0 = every check passed.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include "fw_tests.h"
#include "rs_build.h"
#include "rs_test.h"

int g_pass = 0;
int g_fail = 0;
std::string g_report_path;

namespace {
struct Suite {
  const char* name;
  void (*fn)();
};

const Suite kSuites[] = {
    {"fw_boot", fw_boot},
    {"fw_text_tables", fw_text_tables},
    {"fw_ui_screens", fw_ui_screens},
    {"fw_buttons_menu", fw_buttons_menu},
    {"fw_console", fw_console},
    {"fw_persistence", fw_persistence},
    {"fw_storage", fw_storage},
    {"fw_json", fw_json},
    {"fw_presets", fw_presets},
    {"fw_manual_pots", fw_manual_pots},
    {"fw_calibration", fw_calibration},
    {"fw_abort_and_misc", fw_abort_and_misc},
    {"fw_clock_wrap", fw_clock_wrap},
    {"fw_imu_faults", fw_imu_faults},
    {"fw_out_of_memory", fw_out_of_memory},
    {"fw_memory_report", fw_memory_report},
    {"fw_iq_clean", fw_iq_clean},
    {"fw_iq_calibration", fw_iq_calibration},
    {"fw_iq_faults", fw_iq_faults},
    {"fw_iq_stall", fw_iq_stall},
    {"fw_iq_host_imu", fw_iq_host_imu},
    {"fw_iq_geometry", fw_iq_geometry},
    {"fw_iq_console", fw_iq_console},
    {"fw_iq_timing", fw_iq_timing},
};
}  // namespace

int main(int argc, char** argv) {
  std::string only;
  bool list = false;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--only" && i + 1 < argc) only = argv[++i];
    else if (a == "--list") list = true;
    else if (a == "--report" && i + 1 < argc) g_report_path = argv[++i];
    else if (a == "--screens" && i + 1 < argc) g_screens_dir = argv[++i];
    else if (a == "--log-out" && i + 1 < argc) g_log_out = argv[++i];
    else {
      std::printf("usage: %s [--only <substring>] [--list] [--report f.md] [--screens dir] [--log-out f.jsonl]\n", argv[0]);
      return 2;
    }
  }
  if (list) {
    for (size_t i = 0; i < sizeof(kSuites) / sizeof(kSuites[0]); i++) std::printf("%s\n", kSuites[i].name);
    return 0;
  }

  std::printf("RubbleSense firmware harness (host, virtual clock) - build: %s\n",
#if RS_IQ_ON_UART0
              "I/Q on UART0"
#else
              "I/Q on UART2"
#endif
  );
  const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  int ran = 0;
  for (size_t i = 0; i < sizeof(kSuites) / sizeof(kSuites[0]); i++) {
    if (!only.empty() && std::strstr(kSuites[i].name, only.c_str()) == nullptr) continue;
    const std::chrono::steady_clock::time_point s0 = std::chrono::steady_clock::now();
    const int f0 = g_fail;
    std::printf("[ RUN  ] %s\n", kSuites[i].name);
    std::fflush(stdout);
    kSuites[i].fn();
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - s0).count();
    std::printf("[ %s ] %s (%.1f s)\n", g_fail == f0 ? " OK " : "FAIL", kSuites[i].name, secs);
    std::fflush(stdout);
    ran++;
  }
  const double total = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("\n%d suites, %d checks passed, %d failed (%.1f s)\n", ran, g_pass, g_fail, total);

  if (!g_report_path.empty()) {
    std::ofstream f(g_report_path.c_str());
    f << "# RubbleSense firmware harness report (SIMULATION ONLY)\n\n";
    f << "Unmodified firmware sources (`src/*.cpp`) compiled for the host with Arduino/ESP32 stubs and run under a virtual clock.\n";
    f << "Nothing here has run on an ESP32 or against a real radar.\n\n";
    f << "Build: " <<
#if RS_IQ_ON_UART0
        "serial I/Q on UART0 (shared with the console)"
#else
        "serial I/Q on UART2 (dedicated)"
#endif
      << ". Result: " << g_pass << " checks passed, " << g_fail << " failed, " << ran << " suites.\n\n";
    for (size_t i = 0; i < g_report.size(); i++) f << g_report[i] << "\n";
  }
  if (!g_log_out.empty()) std::printf("scan log written to %s\n", g_log_out.c_str());
  return g_fail == 0 ? 0 : 1;
}
