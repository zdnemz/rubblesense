# RubbleSense firmware harness report (SIMULATION ONLY)

Unmodified firmware sources (`src/*.cpp`) compiled for the host with Arduino/ESP32 stubs and run under a virtual clock.
Nothing here has run on an ESP32 or against a real radar.

Build: serial I/Q on UART0 (shared with the console). Result: 9626 checks passed, 0 failed, 24 suites.

### Trial presets through the real firmware (button -> acquisition -> DSP -> decision -> JSON log)

White-Gaussian-noise simulator, default SIM baseline 21.1 dB, 30 scans per preset, virtual clock, hardware-IMU stub at rest.

| preset | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN |
|---|---|---|---|---|---|---|---|
| A: person, quiet | 30 | 30 | 0 | 0 | 0 | 0 | 0 |
| B: empty (control) | 30 | 0 | 0 | 0 | 0 | 0 | 30 |
| C: sensor knocked | 30 | 0 | 0 | 0 | 0 | 30 | 0 |
| D: person + vibration 0.45 Hz | 30 | 30 | 0 | 0 | 0 | 0 | 0 |
| E: vibration = breathing | 30 | 0 | 0 | 30 | 0 | 0 | 0 |
| F: empty, 3x noise | 30 | 0 | 1 | 0 | 29 | 0 | 0 |

### Memory (host build; the 32-bit target differs only by pointer sizes)

| object | bytes | where |
|---|---|---|
| App (all modules, rings, JSON buffer) | 15016 | static (.bss) |
| rs::SegmentBuffer (24 bins x 512 frames x int16 I/Q) | 49668 | heap, one block |
| rs::Workspace (FFT / CFAR scratch) | 35444 | heap, one block |
| ScanRecord (JSON builder input) | 400 | stack of the UI thread |


### Serial I/Q source, clean stream (I/Q on UART0 (USB, shared with the console))

External front-end model on the serial link, 20 Hz x 16 bins, 25.6 s segments, white-Gaussian simulator, local IMU stub at rest, no calibration (sensitivity unknown).

| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |
|---|---|---|---|---|---|---|---|---|---|---|
| person (amplitude 30) | 6 | 6 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| empty scene | 6 | 0 | 0 | 0 | 0 | 0 | 6 | 0 | 0 | 0 |
| person, front-end at 25 Hz | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| person, 24 range bins (target in bin 20) | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| person, 8 range bins (target in bin 5) | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### Serial I/Q source, calibrated (I/Q on UART0 (USB, shared with the console))

Baseline measured on an empty scene (sigma 8 counts); then scans on a quiet and on a noisy front-end.

| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |
|---|---|---|---|---|---|---|---|---|---|---|
| person, quiet front-end | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| empty, floor +9.6 dB over the baseline | 4 | 0 | 0 | 0 | 4 | 0 | 0 | 0 | 0 | 0 |
| person, floor +9.6 dB over the baseline | 4 | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### Serial I/Q source, link faults (I/Q on UART0 (USB, shared with the console))

Person of amplitude 30 unless noted. Up to 2 % lost frames are tolerated (zero-order hold, flagged); more makes the segment invalid ('radar_dropout'), never a verdict from damaged data.

| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 % of the frames lost | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 20 | 0 | 0 |
| 1 % of the packets corrupted (CRC) | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 20 | 20 | 0 |
| every 3rd packet duplicated | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1-40 garbage bytes before every 5th packet | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| mix: 0.5 % lost, 0.5 % corrupt, dup 1/7, garbage 1/9 | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 20 | 10 | 0 |
| 3 % of the frames lost | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 64 | 0 | 0 |
| 10 % of the frames lost | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 212 | 0 | 0 |
| 12.5 % of the packets corrupted (CRC) | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 264 | 258 | 0 |
| mix as above, empty scene | 3 | 0 | 0 | 0 | 0 | 0 | 3 | 30 | 12 | 0 |

### Serial I/Q source, stalls (I/Q on UART0 (USB, shared with the console))

The front-end stops delivering frames in the middle of a scan.

| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |
|---|---|---|---|---|---|---|---|---|---|---|
| front-end stops mid-segment and never returns | 1 | 0 | 0 | 0 | 0 | 1 | 0 | - | - | - |
| front-end pauses 4 s, then resumes | 1 | 1 | 0 | 0 | 0 | 0 | 0 | - | - | - |

### Serial I/Q source, stability information (I/Q on UART0 (USB, shared with the console))

The IMU must move with the antenna. A front-end that carries its own IMU says so in every frame (protocol flag); the device then ignores its local IMU for those frames.

| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |
|---|---|---|---|---|---|---|---|---|---|---|
| no local IMU, front-end IMU flags (person) | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| no local IMU, front-end IMU flags (empty) | 2 | 0 | 0 | 0 | 0 | 0 | 2 | 0 | 0 | 0 |
| no local IMU, no front-end flags (person) | 1 | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 |
| radar head bumped 1.0 mm at 0.3/s, flagged by front-end IMU | 3 | 0 | 0 | 0 | 0 | 2 | 1 | 0 | 0 | 0 |
| local IMU knocked every 3 s, front-end still | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| local IMU knocked every 3 s, no front-end flags | 1 | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 0 | 0 |

### Serial I/Q source, front-end timestamps (I/Q on UART0 (USB, shared with the console))

Person of amplitude 30. The announced frame rate is what the info packet says; the real rate is what the front-end's clock produces. A mismatch of more than 2 % must be rejected, a smaller one is tolerated.

| scenario | scans | CANDIDATE | POSSIBLE | CONFOUNDED | LOW_SENS | INVALID | NO_PATTERN | lost frames | CRC errors | bad-timing frames |
|---|---|---|---|---|---|---|---|---|---|---|
| honest front-end, 20 Hz | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| clock 1 % fast (20.2 Hz, announces 20) | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| clock 1 % slow (19.8 Hz, announces 20) | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| clock 3 % fast (20.6 Hz, announces 20) | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 0 | 0 | 1708 |
| clock 3 % slow (19.4 Hz, announces 20) | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 0 | 0 | 1708 |
| runs at 25 Hz, announces 20 | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 0 | 0 | 1708 |
| runs at 20 Hz, announces 25 | 2 | 0 | 0 | 0 | 0 | 2 | 0 | 0 | 0 | 1708 |
| no timestamps (t_ms = 0): cannot be checked | 2 | 2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
