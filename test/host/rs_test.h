// Minimal dependency-free test helpers for the host test-bench.
#pragma once
#include <cmath>
#include <cstdio>

extern int g_pass;
extern int g_fail;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (cond) { g_pass++; }                                                  \
    else { g_fail++; std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
  do {                                                                       \
    const double _a = (a), _b = (b), _t = (tol);                             \
    if (std::fabs(_a - _b) <= _t) { g_pass++; }                              \
    else { g_fail++; std::printf("  FAIL %s:%d: %s=%.6g vs %s=%.6g (tol %.3g)\n", \
                                 __FILE__, __LINE__, #a, _a, #b, _b, _t); } \
  } while (0)

// Relative closeness: |a-b| <= rel * |b|
#define CHECK_REL(a, b, rel)                                                 \
  do {                                                                       \
    const double _a = (a), _b = (b), _r = (rel);                             \
    if (std::fabs(_a - _b) <= _r * std::fabs(_b)) { g_pass++; }              \
    else { g_fail++; std::printf("  FAIL %s:%d: %s=%.6g vs %s=%.6g (rel %.3g)\n", \
                                 __FILE__, __LINE__, #a, _a, #b, _b, _r); } \
  } while (0)

#define RUN(fn)                                                              \
  do { std::printf("[ RUN  ] %s\n", #fn); const int _f0 = g_fail; fn();      \
       std::printf("[ %s ] %s\n", (g_fail == _f0) ? " OK " : "FAIL", #fn); } while (0)
