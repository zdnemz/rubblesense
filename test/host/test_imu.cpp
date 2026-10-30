// IMU stability monitor: false-alarm rate, bias removal, motion detection, hold time.
#include <cmath>
#include "rs_imu.h"
#include "rs_simgen.h"
#include "rs_test.h"

using namespace rs;

namespace {

constexpr float kG = 9.80665f;

struct ImuRig {
  ImuProcessor imu;
  Rng rng{7};
  float noise_a = 0.02f, noise_g = 0.004f;
  float bias[3] = {0.10f, -0.15f, 0.06f};   // rad/s zero-rate offset (MPU6050 can be far worse)
  float tilt_dps = 0.0f;                    // constant gravity-direction rotation about y
  float theta = 0.0f;                       // rad
  float spike_a = 0.0f;                     // m/s^2 added on x for the next sample only

  void configure(const ImuGateParams& gp = ImuGateParams()) { imu.configure(gp, 100.0f, 20.0f); }

  // One radar frame = 5 IMU samples at 100 Hz.
  ImuStats frame() {
    for (int k = 0; k < 5; k++) {
      theta += tilt_dps * 0.01f * 0.017453293f;
      const float ax = kG * sinf(theta) + noise_a * rng.normal() + spike_a;
      const float ay = noise_a * rng.normal();
      const float az = kG * cosf(theta) + noise_a * rng.normal();
      spike_a = 0.0f;
      imu.push(ax, ay, az, bias[0] + noise_g * rng.normal(), bias[1] + noise_g * rng.normal(),
               bias[2] + noise_g * rng.normal());
    }
    return imu.takeFrame();
  }
};

}  // namespace

void test_imu() {
  // Stationary sensor with a typical gyro offset: no false motion over one hour of frames.
  {
    ImuRig r; r.configure();
    for (int i = 0; i < 60; i++) r.frame();               // 3 s settling
    int flagged = 0;
    for (int i = 0; i < 72000; i++) if (r.frame().motion) flagged++;
    CHECK(flagged == 0);
  }

  // A gyro offset of ~29 deg/s is removed by the bias tracker.
  {
    ImuRig r; r.configure();
    r.bias[0] = 0.5f; r.bias[1] = -0.3f; r.bias[2] = 0.2f;
    for (int i = 0; i < 60; i++) r.frame();
    int flagged = 0;
    for (int i = 0; i < 2000; i++) if (r.frame().motion) flagged++;
    CHECK(flagged == 0);
  }

  // A single knock is flagged and held for hold_frames afterwards, then released.
  {
    ImuRig r; r.configure();
    for (int i = 0; i < 100; i++) r.frame();
    r.spike_a = 3.0f;
    int flagged = 0;
    for (int i = 0; i < 60; i++) if (r.frame().motion) flagged++;
    CHECK(flagged == 1 + ImuGateParams().hold_frames);
  }

  // Slow tilt: 0.5 deg/s is tolerated, 3 deg/s is flagged.
  {
    ImuRig r; r.configure();
    r.tilt_dps = 0.5f;
    for (int i = 0; i < 100; i++) r.frame();
    int flagged = 0;
    for (int i = 0; i < 200; i++) if (r.frame().motion) flagged++;
    CHECK(flagged == 0);
    r.tilt_dps = 3.0f;
    flagged = 0;
    for (int i = 0; i < 100; i++) if (r.frame().motion) flagged++;
    CHECK(flagged >= 90);
  }

  // A slip the accelerometer high-pass and the (bias-tracking) gyro both miss is caught by the tilt gate.
  {
    ImuRig r; r.configure();
    r.noise_a = 0.0f; r.noise_g = 0.0f;                    // deterministic
    r.tilt_dps = 1.65f;
    r.bias[0] = r.bias[1] = r.bias[2] = 0.0f;
    for (int i = 0; i < 100; i++) r.frame();
    int tiltOnly = 0, total = 0;
    for (int i = 0; i < 100; i++) {
      const ImuStats s = r.frame();
      total++;
      if (s.motion && s.a_rms < ImuGateParams().a_rms_max && s.g_rms < ImuGateParams().g_rms_max) tiltOnly++;
    }
    CHECK(tiltOnly >= 90 && total == 100);
  }

  // No samples during a frame: stability unknown -> reported invalid and unstable.
  {
    ImuRig r; r.configure();
    for (int i = 0; i < 40; i++) r.frame();
    const ImuStats s = r.imu.takeFrame();                  // nothing pushed since the last frame
    CHECK(!s.valid && s.motion);
  }
}
