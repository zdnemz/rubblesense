// RubbleSense firmware - build-time configuration.
//
// Override any of these from platformio.ini (build_flags = -DRS_WOKWI=1 ...).
#pragma once

#define RS_FW_VERSION "1.0.0"

// 1 = running inside the Wokwi simulator (no real radar, IMU is the virtual MPU6050).
#ifndef RS_WOKWI
#define RS_WOKWI 0
#endif

// 1 = firmware is compiled for the host test-bench (no FreeRTOS tasks, virtual clock).
#ifndef RS_HOST
#define RS_HOST 0
#endif

// Speed-up of the *simulated* radar (SimSource only). A real-time scan takes ~50 s; the
// simulator can run faster than real time so demos do not take that long. The real IMU is
// then sampled 5x per simulated frame instead of at 100 Hz (so IMU statistics are only
// meaningful at warp 1). The serial I/Q source and any real front-end always run at 1.
#ifndef RS_SIM_WARP_DEFAULT
#if RS_WOKWI
#define RS_SIM_WARP_DEFAULT 8
#else
#define RS_SIM_WARP_DEFAULT 1
#endif
#endif

// 1 = run the segment analysis inline in the UI thread instead of a FreeRTOS task on core 0.
// Wokwi's ESP32 does not keep the FPU registers apart when the acquisition task (prio 4) preempts
// the DSP task on the same core: the DSP picks up the other task's floats (noise floor ~+77 dB,
// every preset ends yellow). Inline, each core has a single FPU user. Costs a ~1 s UI pause per segment.
#ifndef RS_DSP_INLINE
#if RS_HOST || RS_WOKWI
#define RS_DSP_INLINE 1
#else
#define RS_DSP_INLINE 0
#endif
#endif

// A live MPU6050 never returns two identical raw sample blocks in a row (LSB noise). A
// sensor that does is stuck / frozen and must not vouch for sensor stability. The Wokwi
// MPU6050 model is noiseless, so the check is off there.
#ifndef RS_IMU_FROZEN_CHECK
#if RS_WOKWI
#define RS_IMU_FROZEN_CHECK 0
#else
#define RS_IMU_FROZEN_CHECK 1
#endif
#endif

// 1 = serial I/Q frames arrive on UART0 (the USB port, shared with the console). That is the
// hardware-in-the-loop / replay set-up and what Wokwi can reach. 0 = dedicated UART2 on
// PIN_IQ_RX / PIN_IQ_TX for a real front-end MCU.
#ifndef RS_IQ_ON_UART0
#if RS_WOKWI
#define RS_IQ_ON_UART0 1
#else
#define RS_IQ_ON_UART0 0
#endif
#endif

#ifndef RS_SERIAL_BAUD
#define RS_SERIAL_BAUD 115200
#endif

// Baud rate of the dedicated I/Q UART (RS_IQ_ON_UART0 == 0). 24 bins x 20 Hz needs ~21 kbit/s.
#ifndef RS_IQ_BAUD
#define RS_IQ_BAUD 460800
#endif

// Size of the UART driver's receive ring for the I/Q stream. It is what survives a hiccup of the
// acquisition task (a flash write, a priority inversion): a 16-bin stream at 20 Hz is ~1.6 KB/s, a
// 24-bin one ~2.2 KB/s, so 4 KB holds well over a second. What does not fit is lost, the sequence
// gap is counted, and the affected segment is rejected instead of being analysed.
#ifndef RS_IQ_RX_RING
#if RS_IQ_ON_UART0
#define RS_IQ_RX_RING 2048                       // Serial's ring, see App::begin()
#else
#define RS_IQ_RX_RING 4096
#endif
#endif

// Scan log (JSON lines on LittleFS): the file is rotated when it grows past this.
#ifndef RS_LOG_MAX_BYTES
#define RS_LOG_MAX_BYTES (192 * 1024)
#endif

// Noise floor (dB) the simulator produces with its default noise (sigma = 8 counts). Used as
// the baseline of the SIM source until the operator calibrates one. Never used for real radars.
#define RS_SIM_DEFAULT_BASELINE_DB 21.1f
