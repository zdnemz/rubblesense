# RubbleSense - host test-bench and tools.
#
# This is NOT the ESP32 build (that is `pio run`, see platformio.ini). It compiles the unmodified firmware
# sources (src/*.cpp) and the portable DSP core (lib/rs_core) for the host with stub Arduino / ESP32 headers
# (test/stubs), runs them under a virtual clock, and builds the replay / recording tools.
#
#   make test            core DSP / decision / protocol tests
#   make harness         the firmware on the host, both UART variants; reports -> docs/results/harness_*.md
#   make tools           build/bin/dump_sim and build/bin/fw_replay
#   make e2e             python3 tools/test_tools.py (protocol, diagram, validator, replay end to end)
#   make syntax          -fsyntax-only over the PlatformIO environments x Arduino-ESP32 2.x / 3.x
#   make check-diagram   netlist check of diagram.json against src/rs_pins.h
#   make montecarlo      Monte-Carlo study (~minutes) -> docs/results/ ; `make montecarlo-quick` for a fast run
#   make screens         UI screens -> PNG in build/screens_png (needs Pillow)
#   make check           test + harness + tools + e2e + syntax + check-diagram
#   make clean
#
# Everything is simulation. A pass says the logic holds under the simulator; it says nothing about a real
# radar, real rubble or the timing of an actual ESP32.

CXX      ?= g++
BUILD    := build
BIN      := $(BUILD)/bin
PY       ?= python3

INC      := -Itest/stubs -Ilib/rs_core -Isrc -Itest/host
WARN     := -Wall -Wextra -Wshadow -Wdouble-promotion
HOSTDEF  := -DRS_HOST=1
DEP      = -MMD -MP

CORE_SRCS := $(wildcard lib/rs_core/*.cpp)
FW_SRCS   := $(wildcard src/*.cpp)
CORE_OBJS := $(patsubst lib/rs_core/%.cpp,$(BUILD)/obj_core/%.o,$(CORE_SRCS))
FW_OBJS_U2 := $(patsubst src/%.cpp,$(BUILD)/obj_u2/%.o,$(FW_SRCS))
FW_OBJS_U0 := $(patsubst src/%.cpp,$(BUILD)/obj_u0/%.o,$(FW_SRCS))
STUB_OBJ  := $(BUILD)/obj_stub/stub_impl.o

HARNESS_NAMES := fw_main fw_tests_core fw_tests_scan fw_tests_iq fw_tests_iq2
HOBJ_U2 := $(patsubst %,$(BUILD)/hobj_u2/%.o,$(HARNESS_NAMES))
HOBJ_U0 := $(patsubst %,$(BUILD)/hobj_u0/%.o,$(HARNESS_NAMES))

TEST_SRCS := test/host/test_main.cpp test/host/test_basic.cpp test/host/test_dsp.cpp test/host/test_decision.cpp \
             test/host/test_imu.cpp test/host/test_proto.cpp

.PHONY: all test harness tools e2e syntax check-diagram montecarlo montecarlo-quick screens check clean help
all: tools test harness

help:
	@sed -n '1,22p' Makefile

# ------------------------------------------------------------------------------------------ objects
# the DSP core is the same for every variant
$(BUILD)/obj_core/%.o: lib/rs_core/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++11 -O1 -g $(WARN) $(HOSTDEF) $(INC) $(DEP) -c $< -o $@

# firmware: UART2 (default build) and UART0 (hardware-in-the-loop / Wokwi) flavours
$(BUILD)/obj_u2/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++11 -O1 -g $(WARN) $(HOSTDEF) $(INC) $(DEP) -c $< -o $@
$(BUILD)/obj_u0/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++11 -O1 -g $(WARN) $(HOSTDEF) -DRS_IQ_ON_UART0=1 $(INC) $(DEP) -c $< -o $@

# the stubs and the test-bench are C++17
$(STUB_OBJ): test/stubs/stub_impl.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Itest/stubs $(DEP) -c $< -o $@
$(BUILD)/hobj_u2/%.o: test/host/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra $(HOSTDEF) $(INC) $(DEP) -c $< -o $@
$(BUILD)/hobj_u0/%.o: test/host/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra $(HOSTDEF) -DRS_IQ_ON_UART0=1 $(INC) $(DEP) -c $< -o $@

# ------------------------------------------------------------------------------------------ binaries
$(BIN)/rs_tests: $(TEST_SRCS) $(CORE_SRCS) $(wildcard test/host/*.h) $(wildcard lib/rs_core/*.h)
	@mkdir -p $(BIN)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wshadow -Wdouble-promotion -DHAVE_TEST_DSP -DHAVE_TEST_DECISION -DHAVE_TEST_IMU_PROTO \
	  -Ilib/rs_core -Itest/host $(TEST_SRCS) $(CORE_SRCS) -o $@

$(BIN)/fw_harness_u2: $(HOBJ_U2) $(FW_OBJS_U2) $(CORE_OBJS) $(STUB_OBJ)
	@mkdir -p $(BIN)
	$(CXX) -g $^ -lpthread -o $@
$(BIN)/fw_harness_u0: $(HOBJ_U0) $(FW_OBJS_U0) $(CORE_OBJS) $(STUB_OBJ)
	@mkdir -p $(BIN)
	$(CXX) -g $^ -lpthread -o $@

# fw_replay is the UART0 flavour: stream and console share one port, as with a PC on USB or Wokwi
$(BUILD)/hobj_u0/fw_replay.o: test/host/fw_replay.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Wshadow $(HOSTDEF) -DRS_IQ_ON_UART0=1 $(INC) $(DEP) -c $< -o $@
$(BIN)/fw_replay: $(BUILD)/hobj_u0/fw_replay.o $(FW_OBJS_U0) $(CORE_OBJS) $(STUB_OBJ)
	@mkdir -p $(BIN)
	$(CXX) -g $^ -lpthread -o $@

# dump_sim writes a simulated scene as an I/Q recording (the firmware's simulator, see src/radar_sim.cpp)
$(BIN)/dump_sim: tools/dump_sim.cpp src/radar_sim.cpp $(CORE_SRCS) $(wildcard src/*.h) $(wildcard lib/rs_core/*.h)
	@mkdir -p $(BIN)
	$(CXX) -std=c++17 -O2 -Wall -Wextra $(HOSTDEF) $(INC) tools/dump_sim.cpp src/radar_sim.cpp $(CORE_SRCS) -o $@

$(BIN)/montecarlo: test/host/montecarlo.cpp $(CORE_SRCS) $(wildcard test/host/*.h) $(wildcard lib/rs_core/*.h)
	@mkdir -p $(BIN)
	$(CXX) -std=c++17 -O2 -pthread -Wall -Wextra -Ilib/rs_core -Itest/host test/host/montecarlo.cpp $(CORE_SRCS) -o $@

# ------------------------------------------------------------------------------------------ targets
tools: $(BIN)/dump_sim $(BIN)/fw_replay

test: $(BIN)/rs_tests
	$(BIN)/rs_tests

harness: $(BIN)/fw_harness_u2 $(BIN)/fw_harness_u0
	@mkdir -p docs/results $(BUILD)/screens
	$(BIN)/fw_harness_u2 --report docs/results/harness_uart2.md --screens $(BUILD)/screens --log-out $(BUILD)/scan_log_u2.jsonl
	$(BIN)/fw_harness_u0 --report docs/results/harness_uart0.md --log-out $(BUILD)/scan_log_u0.jsonl
	$(PY) tools/validate_scan_log.py $(BUILD)/scan_log_u2.jsonl
	$(PY) tools/validate_scan_log.py $(BUILD)/scan_log_u0.jsonl

e2e: tools
	$(PY) tools/test_tools.py --bin-dir $(BIN)

syntax:
	sh tools/syntax_matrix.sh

check-diagram:
	$(PY) tools/check_diagram.py

montecarlo: $(BIN)/montecarlo
	@mkdir -p docs/results
	$(BIN)/montecarlo --out=docs/results > docs/results/montecarlo_summary.md
	@echo "wrote docs/results/montecarlo_summary.md"

montecarlo-quick: $(BIN)/montecarlo
	@mkdir -p $(BUILD)/mc_quick
	$(BIN)/montecarlo --quick --out=$(BUILD)/mc_quick

screens: $(BIN)/fw_harness_u2
	@mkdir -p $(BUILD)/screens
	$(BIN)/fw_harness_u2 --only fw_ui_screens --screens $(BUILD)/screens > /dev/null
	$(PY) tools/render_screens.py $(BUILD)/screens --out $(BUILD)/screens_png --sheet

check: test harness e2e syntax check-diagram
	@echo "all checks passed"

clean:
	rm -rf $(BUILD)

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)
