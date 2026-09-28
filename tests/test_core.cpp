#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "components/vpd_kalman/vpd_controller_core.h"

using namespace esphome::vpd_kalman;

static Inputs climate() {
  Inputs in;
  in.auto_on = true;
  in.night_has_state = true;
  in.t = 25.0f;
  in.rh = 60.0f;
  in.room_t = 22.0f;
  in.room_rh = 50.0f;
  in.target_day = 1.0f;
  in.transition = 45.0f;
  return in;
}

template<typename Controller> static void tick(Controller &core, Inputs &in, Outputs &out, int count = 1) {
  for (int i = 0; i < count; i++) {
    in.now_ms += 10000;
    core.step(in, out);
    assert(std::isfinite(out.fan_output));
    assert(out.fan_output >= 1.0f && out.fan_output <= 100.0f);
    for (const auto &event : out.events)
      if (event.kind == Event::SET_LEVEL)
        assert(std::isfinite(event.level) && event.level >= 0.01f && event.level <= 1.0f);
  }
}

static void freshness() {
  SampleClock clock;
  assert(!clock.fresh(0, 120000));
  clock.received(100);
  assert(clock.fresh(120099, 120000));
  assert(!clock.fresh(120100, 120000));
  clock.received(120100);  // Repeated identical readings are new samples too.
  assert(clock.fresh(120100, 120000));
  clock.received(UINT32_MAX - 100);
  assert(clock.fresh(99, 201));
  assert(!clock.fresh(100, 201));
  assert(!clock.fresh(UINT32_MAX - 99, 201));
}

template<typename Controller> static void target_ramp() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  tick(core, in, out, 30);
  in.target_day = 1.8f;
  tick(core, in, out);
  assert(out.target_active > 1.0f);
  assert(out.target_active < 1.01f);
  tick(core, in, out, 271);
  assert(std::fabs(out.target_active - 1.8f) < 0.001f);
  in.target_day = 0.5f;
  tick(core, in, out);
  assert(out.target_active < 1.8f && out.target_active > 1.79f);
  in.transition = 0;
  in.target_day = 1.3f;
  tick(core, in, out);
  assert(out.target_active == 1.3f);
  in.transition = 45;
  in.target_night = 1.1f;
  in.night = true;
  tick(core, in, out);
  assert(out.control_vpd < 1.1f);
  assert(out.target_active > 1.29f && out.target_active < 1.3f);
  for (int i = 0; i < 270; i++) {
    const float previous = out.target_active;
    tick(core, in, out);
    assert(out.target_active <= previous && out.target_active >= 1.1f);
  }
  assert(out.target_active == 1.1f);
  in.target_night = 1.5f;
  tick(core, in, out, 30);
  const float interrupted = out.target_active;
  in.target_night = 1.2f;
  tick(core, in, out);
  assert(out.target_active >= interrupted && out.target_active < interrupted + 0.01f);
  in.transition = 0;
  tick(core, in, out);
  assert(out.target_active == 1.2f);
  in.transition = 45;
  in.target_night = 1.8f;
  tick(core, in, out, 30);
  const float before_shortening = out.target_active;
  in.transition = 1;
  for (int i = 1; i <= 6; i++) {
    tick(core, in, out);
    const float expected = before_shortening + (1.8f - before_shortening) * i / 6.0f;
    assert(std::fabs(out.target_active - expected) < 0.0001f);
  }
  assert(std::fabs(out.target_active - 1.8f) < 0.0001f);
  in.target_night = 1.1f;
  tick(core, in, out, 2);
  const float before_lengthening = out.target_active;
  in.transition = 2;
  tick(core, in, out);
  assert(std::fabs(out.target_active - (before_lengthening - (before_lengthening - 1.1f) / 12)) < 0.0001f);
}

template<typename Controller> static void invalid_inputs_and_manual_protection() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  tick(core, in, out, 20);
  for (float invalid : {NAN, INFINITY, -INFINITY, -1.0f, 101.0f}) {
    in.rh = invalid;
    tick(core, in, out);
    assert(out.fan_output == 40.0f);
    assert(std::isnan(out.control_vpd));
  }
  in.rh = 60;
  in.t = -237.3f;  // Singularity of the vapour-pressure equation must not be evaluated.
  tick(core, in, out);
  assert(out.fan_output == 40.0f);
  in.auto_on = false;
  in.manual_speed = 5;
  in.t = 35;
  tick(core, in, out);
  assert(out.fan_output == 100.0f);
  float level;
  assert(core.manual_level(false, 5, level) && level == 100.0f);
  assert(core.control_off_level(5) == 100.0f);
  assert(core.control_off_level(NAN) == 100.0f);
  in.t = 25;
  tick(core, in, out);
  assert(out.fan_output == 5.0f);
  in.t = 35;
  in.safety_in_manual = false;
  tick(core, in, out);
  assert(out.fan_output == 5.0f);
  assert(core.control_off_level(5) == 5.0f);
}

template<typename Controller> static void external_source() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  in.ext.selected = true;
  in.ext.temperature = 25;
  in.ext.humidity = 50;
  in.ext.leaf_temperature = 23;
  tick(core, in, out, 20);
  float own_vpd = out.control_vpd;
  in.ext.sample_age_ms = 0;
  tick(core, in, out, 20);
  assert(out.control_vpd > own_vpd + 0.2f);
  in.ext.sample_age_ms = in.sensor_timeout_ms;
  tick(core, in, out, 20);
  assert(std::fabs(out.control_vpd - own_vpd) < 0.001f);
  in.ext.sample_age_ms = 0;
  in.ext.leaf_temperature = 60;  // Reject implausible external leaf readings too.
  tick(core, in, out, 20);
  assert(std::fabs(out.control_vpd - own_vpd) < 0.001f);
}

static void switching() {
  VpdControllerCore core;
  core.set_room_configured(true);
  Inputs in = climate();
  Outputs out;
  tick(core, in, out, 20);
  out.fan_curve_active = true;
  out.learned_airflow_50 = 77;
  in.room_rh = 150;
  tick(core, in, out, 11);
  assert(!out.fallback_active);
  tick(core, in, out);
  assert(out.fallback_active);
  assert(!out.fan_curve_active && std::isnan(out.learned_airflow_50));
  assert(std::isnan(out.moisture_load));
  in.room_rh = 50;
  tick(core, in, out, 5);
  assert(out.fallback_active);
  tick(core, in, out);
  assert(!out.fallback_active);
}

// Independent, exact first-order moisture plant: constant temperature/load.
// This is a baseline regression test, not evidence of performance in every tent.
static float svp(float t) { return 0.6107f * std::pow(10.0f, 7.5f * t / (237.3f + t)); }
static void closed_loop() {
  VpdKalmanCore core;
  Inputs in = climate();
  Outputs out;
  in.target_day = 1.0f;
  in.transition = 0;
  in.sacrifice = 0;
  const float room = svp(in.room_t) * in.room_rh / 100;
  float excess = 0.4f;
  float worst_final_error = 0;
  for (int i = 0; i < 720; i++) {
    float q = 0.2f + 0.8f * core.fan_level / 100;
    float load = i < 360 ? 0.18f : 0.24f;
    excess = load / q + (excess - load / q) * std::exp(-10 * q / 21);
    in.rh = 100 * (room + excess) / svp(in.t);
    tick(core, in, out);
    if (i > 660) worst_final_error = std::max(worst_final_error, std::fabs(out.control_vpd - in.target_day));
  }
  assert(worst_final_error < 0.06f);
  assert(out.fan_output < 90.0f);
  std::printf("Ideal plant after load step: max final error %.4f kPa, fan %.1f%%\n",
              worst_final_error, out.fan_output);
}

static float unreachable_target(float sacrifice) {
  VpdKalmanCore core;
  Inputs in = climate();
  Outputs out;
  in.room_rh = 70;
  in.target_day = 1.4f;
  in.transition = 0;
  in.sacrifice = sacrifice;
  const float room = svp(in.room_t) * in.room_rh / 100;
  const float load = 0.03f;
  float excess = 0.1f;
  for (int i = 0; i < 720; i++) {
    float q = 0.2f + 0.8f * core.fan_level / 100;
    excess = load / q + (excess - load / q) * std::exp(-10 * q / 21);
    in.rh = 100 * (room + excess) / svp(in.t);
    tick(core, in, out);
  }
  assert(out.control_vpd < in.target_day - 0.1f);
  return out.fan_output;
}

template<typename Controller> static void conflicting_runtime_limits() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  in.fan_min = 80;
  in.fan_max = 20;
  tick(core, in, out, 20);
  assert(out.fan_output <= 20);
}

template<typename Controller> static void protection_recovery() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  tick(core, in, out, 20);
  in.t = 35;
  tick(core, in, out);
  assert(out.fan_output == 100);  // No smoothing or rate limit on protection.
  in.t = 25;
  tick(core, in, out);
  assert(std::fabs(out.fan_output - (100.0f - 20.0f / 6.0f)) < 0.01f);
  bool recovery_text = false;
  for (const auto &event : out.events)
    if (event.kind == Event::STATE_TEXT && event.a == "Protection recovery") recovery_text = true;
  assert(recovery_text);
  for (int i = 0; i < 8; i++) {
    float previous = out.fan_output;
    tick(core, in, out);
    assert(previous - out.fan_output <= 20.0f / 6.0f + 0.01f);
  }
  in.t = 35;
  tick(core, in, out);
  assert(out.fan_output == 100);
  in.t = 25;
  in.auto_on = false;
  in.manual_speed = 5;
  tick(core, in, out);
  assert(out.fan_output == 5);  // An explicit manual command cancels the tail.
  in.auto_on = true;
  in.t = 35;
  tick(core, in, out);
  in.t = NAN;
  tick(core, in, out);
  assert(out.fan_output == 40);  // Sensor failure has its own policy.
}

static void protection_handover() {
  VpdControllerCore core;
  core.set_room_configured(true);
  Inputs in = climate();
  Outputs out;
  tick(core, in, out, 20);
  in.t = 35;
  tick(core, in, out);
  assert(out.fan_output == 100);
  in.t = 25;
  for (int i = 0; i < 12; i++) {
    in.force_fallback = i % 2 == 0;
    tick(core, in, out);
    // Handover must preserve the decay, never turn protective output into
    // the new normal controller baseline.
    assert(std::fabs(out.fan_output - (100.0f - (i + 1) * 20.0f / 6.0f)) < 0.02f);
  }
}

template<typename Controller> static void reject_spike_and_follow_real_change() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  in.target_day = 0.909f;  // At the initial measured VPD.
  tick(core, in, out, 30);
  const float before = out.control_vpd;
  const float fan_before = out.fan_output;
  in.rh = 70;  // Isolated plausible humidity spike, below the safety threshold.
  tick(core, in, out);
  assert(out.measured_vpd < before - 0.2f);
  assert(std::fabs(out.control_vpd - before) < 0.001f);
  assert(std::fabs(out.fan_output - fan_before) < 0.001f);
  in.rh = 60;
  tick(core, in, out);
  assert(std::fabs(out.control_vpd - before) < 0.001f);
  in.rh = 70;
  tick(core, in, out, 12);  // A sustained change must not be rejected indefinitely.
  assert(std::fabs(out.control_vpd - out.measured_vpd) < 0.01f);
  in.rh = NAN;
  tick(core, in, out);
  in.rh = 50;
  tick(core, in, out);
  assert(std::fabs(out.control_vpd - out.measured_vpd) < 0.001f);  // No history across a data gap.
}

template<typename Controller> static void anticipating_rise() {
  Controller anticipating, reactive;
  Inputs in = climate(), no_trend = in;
  Outputs out, reference;
  in.sacrifice = no_trend.sacrifice = 0;
  in.target_day = no_trend.target_day = 1.2f;
  no_trend.tuning.trend_horizon = 0;
  for (int i = 0; i < 10; i++) {
    const float vpd = 0.75f + i * 0.035f;
    in.rh = no_trend.rh = 100 * (svp(23) - vpd) / svp(25);
    tick(anticipating, in, out);
    tick(reactive, no_trend, reference);
  }
  // The fallback PI already reduces its demand on a rising signal; its extra
  // reduction is smaller than the Kalman controller's, but must be measurable.
  assert(out.fan_output + 0.01f < reference.fan_output);
}

template<typename Controller> static void rise_limit() {
  Controller core;
  Inputs in = climate();
  Outputs out;
  in.sacrifice = 0;
  in.target_day = 2.4f;
  in.tuning.fan_increase_rate = 6;
  tick(core, in, out);
  float previous = out.controller_output;
  for (int i = 0; i < 10; i++) {
    tick(core, in, out);
    assert(out.controller_output - previous <= 1.001f);
    previous = out.controller_output;
  }
  in.rh = 95;
  tick(core, in, out);
  assert(out.fan_output == 100);
}

#ifdef USE_VPD_FAN_CURVE
static void learning_persistence() {
  FanCurveBank bank;
  bank.set_config(0.2f, 21);
  FanCurveBank::Stored stored{};
  bank.save(stored);
  assert(bank.load(stored));
  bank.set_config(0.2f, 30);
  assert(!bank.load(stored));
  bank.set_config(0.2f, 21);
  stored.hist[0][0] = NAN;
  assert(!bank.load(stored));
  VpdKalmanCore core;
  core.set_fan_curve_learning(true);
  Inputs in = climate();
  Outputs out;
  tick(core, in, out, 100);
  assert(std::isfinite(out.learned_airflow_50));
  assert(std::isfinite(out.learned_sensible_max));
}
#endif

class FallbackProbe : public VpdFallbackCore {
 public:
  bool testing() const { return test; }
};

static void economic_confirmation() {
  ConfirmedLimit limit;
  assert(limit.update(50) == 50);
  for (int i = 0; i < 20; i++) assert(limit.update(i % 2 ? 49 : 51) == 50);
  assert(limit.update(55) == 50);
  assert(limit.update(55) == 50);
  assert(limit.update(55) == 55);
  VpdKalmanCore core;
  Inputs in = climate();
  Outputs out;
  in.sacrifice = 0.1f;
  tick(core, in, out, 100);
  assert(out.sensible_max < 100);
  in.sacrifice = 0;
  tick(core, in, out);
  assert(out.sensible_max == 100);  // Explicit opt-out does not wait for a ramp.
}

static void fallback_test_quality() {
  FallbackProbe stable, drifting;
  Inputs in = climate();
  in.target_day = 2;
  in.rh_max = 95;
  in.transition = 0;
  in.tuning.fallback_time_constant = 0.5f;
  in.tuning.fallback_smoothing = 10;
  Outputs out;
  bool started = false;
  for (int i = 0; i < 200; i++) {
    tick(stable, in, out);
    if (stable.testing()) { started = true; break; }
  }
  assert(started);
  const float test_output = out.fan_output;
  tick(stable, in, out);
  assert(out.fan_output == test_output);  // PI cannot change the test stimulus.
  in.rh -= 5;  // A raw VPD jump must cancel the experiment, not be learned.
  tick(stable, in, out);
  assert(!stable.testing());
  bool aborted = false;
  for (const auto &event : out.events)
    if (event.a.find("Climate disturbance") != std::string::npos) aborted = true;
  assert(aborted);
  for (int i = 0; i < 200; i++) {
    const float vpd = 0.6f + 0.008f * (i % 60);
    in.rh = 100 * (svp(23) - vpd) / svp(25);
    tick(drifting, in, out);
    assert(!drifting.testing());  // No experiments while the background drifts.
  }
}

int main() {
  economic_confirmation();
  fallback_test_quality();
  freshness();
  target_ramp<VpdKalmanCore>();
  target_ramp<VpdFallbackCore>();
  invalid_inputs_and_manual_protection<VpdKalmanCore>();
  invalid_inputs_and_manual_protection<VpdFallbackCore>();
  external_source<VpdKalmanCore>();
  external_source<VpdFallbackCore>();
  switching();
  closed_loop();
  conflicting_runtime_limits<VpdKalmanCore>();
  conflicting_runtime_limits<VpdFallbackCore>();
  protection_recovery<VpdKalmanCore>();
  protection_recovery<VpdFallbackCore>();
  protection_handover();
  reject_spike_and_follow_real_change<VpdKalmanCore>();
  reject_spike_and_follow_real_change<VpdFallbackCore>();
  anticipating_rise<VpdKalmanCore>();
  anticipating_rise<VpdFallbackCore>();
  rise_limit<VpdKalmanCore>();
  rise_limit<VpdFallbackCore>();
  float unlimited = unreachable_target(0);
  float economical = unreachable_target(0.03f);
  assert(unlimited > 99);
  assert(economical < 50);
  std::printf("Unreachable target: fan %.1f%% without sacrifice, %.1f%% with 0.03 kPa sacrifice\n",
              unlimited, economical);
#ifdef USE_VPD_FAN_CURVE
  learning_persistence();
#else
  static_assert(sizeof(VpdControllerCore) < 4096, "Disabled learning must not allocate the filter bank");
#endif
  std::printf("All core tests passed. Controller size: %zu bytes\n", sizeof(VpdControllerCore));
}
