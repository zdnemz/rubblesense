// Host test-bench entry point. Add new suites to the list below.
#include <cstdio>
#include <cstring>
#include "rs_test.h"

int g_pass = 0;
int g_fail = 0;

void test_basic();
#ifdef HAVE_TEST_DSP
void test_dsp();
#endif
#ifdef HAVE_TEST_DECISION
void test_decision();
#endif
#ifdef HAVE_TEST_IMU_PROTO
void test_imu();
void test_proto();
#endif

int main(int argc, char** argv) {
  const char* only = (argc > 1) ? argv[1] : nullptr;
  auto want = [&](const char* name) { return !only || std::strcmp(only, name) == 0; };

  if (want("basic")) test_basic();
#ifdef HAVE_TEST_DSP
  if (want("dsp")) test_dsp();
#endif
#ifdef HAVE_TEST_DECISION
  if (want("decision")) test_decision();
#endif
#ifdef HAVE_TEST_IMU_PROTO
  if (want("imu")) test_imu();
  if (want("proto")) test_proto();
#endif

  std::printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
