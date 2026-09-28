// Deterministic closed-loop regressions. Build this driver against the previous
// headers as well to compare strategies without duplicating their implementation.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include "components/vpd_kalman/vpd_controller_core.h"

using namespace esphome::vpd_kalman;

struct Scenario {
  const char *name;
  float plant_tau, sensor_tau, noise;
  int delay_ticks;
  bool spikes;
  float controller_speed = 0.25f;
};

struct Metrics {
  float error = 0, outside_band = 0, fan_travel = 0, peak_error = 0, final_error = 0;
  float high_fan_minutes = 0;
};

static float svp(float t) { return 0.6107f * std::pow(10.0f, 7.5f * t / (237.3f + t)); }
static float noise(uint32_t &seed) {
  seed = 1664525U * seed + 1013904223U;
  return 2.0f * float(seed >> 8) / 16777215.0f - 1.0f;
}

template<typename Controller> Metrics simulate(const Scenario &scenario) {
  Controller core;
  Inputs in;
  Outputs out;
  in.auto_on = true;
  in.night_has_state = true;
  in.t = 25;
  in.room_t = 22;
  in.room_rh = 50;
  in.target_day = 1.1f;
  in.transition = 0;
  in.sacrifice = 0;
  in.speed = scenario.controller_speed;
  in.rh_max = 95;
  const float room_pressure = svp(in.room_t) * in.room_rh / 100;
  float tent_pressure = room_pressure + 0.4f;
  float sensor_pressure = tent_pressure;
  float delayed[12];
  for (float &value : delayed) value = tent_pressure;
  uint32_t seed = 123456U;
  Metrics metrics;
  constexpr int TICKS = 720;
  for (int i = 0; i < TICKS; i++) {
    const float previous_fan = core.fan_level;
    const float q = 0.2f + 0.8f * previous_fan / 100;
    const float load = i < 120 ? 0.12f : (i < 360 ? 0.20f : 0.14f);
    const float equilibrium = room_pressure + load / q;
    tent_pressure = equilibrium + (tent_pressure - equilibrium) * std::exp(-10 * q / scenario.plant_tau);
    delayed[i % 12] = tent_pressure;
    const float delayed_pressure = delayed[(i - scenario.delay_ticks + 12) % 12];
    sensor_pressure += (scenario.sensor_tau > 0 ? 1 - std::exp(-10 / scenario.sensor_tau) : 1) *
                       (delayed_pressure - sensor_pressure);
    float disturbance = scenario.noise * noise(seed);
    if (scenario.spikes && i > 60 && i % 73 == 0) disturbance += 0.25f;
    in.rh = 100 * (sensor_pressure + disturbance) / svp(in.t);
    in.now_ms += 10000;
    core.step(in, out);
    assert(std::isfinite(out.fan_output) && out.fan_output >= 1 && out.fan_output <= 100);
    const float actual_vpd = svp(23) - tent_pressure;
    const float error = std::fabs(actual_vpd - in.target_day);
    if (i >= 60) {
      metrics.error += error * 10 / 60;
      metrics.outside_band += std::max(0.0f, error - 0.05f) * 10 / 60;
      metrics.fan_travel += std::fabs(out.fan_output - previous_fan);
      metrics.peak_error = std::max(metrics.peak_error, error);
      if (out.fan_output > 90) metrics.high_fan_minutes += 10.0f / 60;
    }
    if (i >= TICKS - 60) metrics.final_error += error / 60;
  }
  return metrics;
}

template<typename Controller> void run(const char *controller) {
  const Scenario scenarios[] = {
      {"ideal", 21, 0, 0, 0, false},
      {"noise_spikes", 21, 0, 0.025f, 0, true},
      {"sensor_lag_30s", 21, 30, 0.01f, 0, false},
      {"slow_delayed", 60, 60, 0.01f, 3, false},
      {"fast_delayed", 21, 30, 0.01f, 3, false, 0.7f},
  };
  for (const auto &scenario : scenarios) {
    const Metrics m = simulate<Controller>(scenario);
    std::printf("%s/%s: IAE=%.4f kPa*min outside=%.4f kPa*min travel=%.1f pp peak=%.4f kPa "
                "final_MAE=%.4f kPa high_fan=%.1f min\n", controller, scenario.name,
                m.error, m.outside_band, m.fan_travel, m.peak_error, m.final_error, m.high_fan_minutes);
    assert(m.final_error < 0.07f);
    assert(m.peak_error < 0.4f);
    assert(m.high_fan_minutes < 1.0f);
    // Scenario-specific ceilings preserve the noise/spike improvement, rather
    // than treating total error inside the allowed deadband as a failure.
    if (scenario.spikes)
      assert(m.fan_travel < (controller[0] == 'k' ? 35.0f : 175.0f));
  }
}

int main() {
  run<VpdKalmanCore>("kalman");
  run<VpdFallbackCore>("fallback");
}
