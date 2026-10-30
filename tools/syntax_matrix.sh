#!/bin/sh
# -fsyntax-only over the build matrix of the firmware sources (C++11, strict warnings):
#   PlatformIO environments {esp32dev, esp32dev_usb_iq, wokwi}
#   x {device code path RS_HOST=0, host test-bench path RS_HOST=1}
#   x {Arduino-ESP32 2.x LEDC API, 3.x LEDC API}
# against the stub headers in test/stubs. It catches code that only breaks in one configuration (an
# #if branch nobody compiles); it is NOT a build for the ESP32 (no real toolchain, no real headers).
#
#   sh tools/syntax_matrix.sh            # from anywhere; exit status 0 = clean
cd "$(dirname "$0")/.." || exit 1
CXX="${CXX:-g++}"
rc=0
n=0
for env in "-DRS_WOKWI=0 -DRS_IQ_ON_UART0=0" "-DRS_WOKWI=0 -DRS_IQ_ON_UART0=1" "-DRS_WOKWI=1"; do
  for host in 0 1; do
    for v in 2 3; do
      if [ "$v" = 3 ]; then EXTRA="-DSTUB_ARDUINO_V3 -Itest/stubs_v3"; else EXTRA=""; fi
      for f in src/*.cpp; do
        n=$((n + 1))
        # shellcheck disable=SC2086
        out=$($CXX -std=c++11 -fsyntax-only -Wall -Wextra -Wshadow -Wdouble-promotion $env -DRS_HOST=$host $EXTRA \
              -Itest/stubs -Ilib/rs_core -Isrc "$f" 2>&1)
        if [ -n "$out" ]; then
          echo "== $env host=$host arduino-v$v $f"
          echo "$out" | head -20
          rc=1
        fi
      done
    done
  done
done
echo "syntax matrix: $n compilations, status $rc"
exit $rc
