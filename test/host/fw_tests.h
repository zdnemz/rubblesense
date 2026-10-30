// Firmware host-test suites (see fw_main.cpp).
#pragma once
#include <string>
#include <vector>

extern std::vector<std::string> g_report;   // markdown lines of the harness report (written by fw_main)
extern std::string g_log_out;               // when set, the scan log produced by fw_presets is written there
extern std::string g_screens_dir;   // when set, key screens are written there as display lists
extern std::string g_report_path;   // when set, a markdown report is written there

void fw_boot();
void fw_text_tables();
void fw_ui_screens();
void fw_buttons_menu();
void fw_console();
void fw_persistence();
void fw_storage();
void fw_json();

void fw_presets();
void fw_manual_pots();
void fw_calibration();
void fw_abort_and_misc();
void fw_clock_wrap();
void fw_imu_faults();
void fw_out_of_memory();
void fw_memory_report();

void fw_iq_clean();
void fw_iq_calibration();
void fw_iq_faults();
void fw_iq_stall();
void fw_iq_host_imu();
void fw_iq_geometry();
void fw_iq_console();
void fw_iq_timing();
