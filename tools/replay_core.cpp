// Stream adapter for the production core. CSV/timestamp handling lives in replay.py.
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include "components/vpd_kalman/vpd_controller_core.h"

using namespace esphome::vpd_kalman;

int main() {
  try {
    VpdControllerCore core;
    Inputs in;
    Outputs out;
    std::unordered_map<std::string, float *> fields = {
        {"temperature", &in.t}, {"humidity", &in.rh}, {"leaf_temperature", &in.leaf},
        {"room_temperature", &in.room_t}, {"room_humidity", &in.room_rh},
        {"target_day", &in.target_day}, {"target_night", &in.target_night},
        {"manual_speed", &in.manual_speed}, {"deadband", &in.deadband},
        {"sacrifice", &in.sacrifice}, {"speed", &in.speed}, {"transition", &in.transition},
        {"fan_min", &in.fan_min}, {"fan_max", &in.fan_max}, {"emergency", &in.emergency},
        {"temp_max", &in.temp_max}, {"temp_band", &in.temp_band},
        {"rh_max", &in.rh_max}, {"rh_band", &in.rh_band}, {"open_max", &in.open_max},
        {"leaf_offset_day", &in.leaf_offset_day}, {"leaf_offset_night", &in.leaf_offset_night},
        {"control_smoothing", &in.tuning.control_smoothing}, {"trend_horizon", &in.tuning.trend_horizon},
        {"fan_increase_rate", &in.tuning.fan_increase_rate},
        {"protection_release_rate", &in.tuning.protection_release_rate},
        {"fallback_time_constant", &in.tuning.fallback_time_constant},
        {"fallback_rate", &in.tuning.fallback_rate}, {"fallback_smoothing", &in.tuning.fallback_smoothing},
        {"limit_finder_cost", &in.tuning.limit_finder_cost}, {"limit_finder_step", &in.tuning.limit_finder_step},
        {"limit_test_margin", &in.tuning.limit_test_margin}, {"limit_test_max_pause", &in.tuning.limit_test_max_pause},
        {"sensor_noise", &in.tuning.sensor_noise}, {"load_change_per_hour", &in.tuning.load_change_per_hour},
        {"sensible_max_rate", &in.tuning.sensible_max_rate},
    };
    std::string line, state;
    bool header = true;
    std::cout << "fan_output\tcontroller_output\tmeasured_vpd\tcontrol_vpd\ttarget_active\tsensible_max"
                 "\tnext_step_benefit\tvpd_at_max\tfallback_active\tlimit_finder_drift"
                 "\tlimit_finder_vpd_change\tlimit_finder_cost_before\tlimit_finder_cost_after\tstate\n";
    std::cout << std::setprecision(9);
    while (std::getline(std::cin, line)) {
      std::istringstream tokens(line);
      std::string key, value;
      while (tokens >> key >> value) {
        const float v = std::stof(value);
        auto field = fields.find(key);
        if (field != fields.end()) *field->second = v;
        else if (key == "room_configured") core.set_room_configured(v != 0);
        else if (key == "initial_output") { core.active().saved_u = v; core.active().fan_level = v; }
        else if (key == "observed_fan") core.active().fan_level = v;
        else if (key == "auto_on") in.auto_on = v != 0;
        else if (key == "night") in.night = v != 0;
        else if (key == "night_has_state") in.night_has_state = v != 0;
        else if (key == "tent_open") in.tent_open = v != 0;
        else if (key == "force_fallback") in.force_fallback = v != 0;
        else if (key == "leaf_switch") in.leaf_switch = v != 0;
        else if (key == "safety_in_manual") in.safety_in_manual = v != 0;
        else throw std::runtime_error("Unsupported field: " + key);
      }
      if (header) { header = false; continue; }
      in.now_ms += 10000;
      core.step(in, out);
      for (const auto &event : out.events) {
        if (event.kind == Event::STATE_TEXT) state = event.a;
        if (event.kind == Event::TURN_OFF_OPEN) in.tent_open = false;
      }
      std::cout << out.fan_output << '\t' << out.controller_output << '\t' << out.measured_vpd << '\t'
                << out.control_vpd << '\t' << out.target_active << '\t' << out.sensible_max << '\t'
                << out.next_step_benefit << '\t' << out.vpd_at_max << '\t' << out.fallback_active << '\t'
                << out.limit_finder_drift << '\t' << out.limit_finder_vpd_change << '\t'
                << out.limit_finder_cost_before << '\t' << out.limit_finder_cost_after << '\t' << state << '\n';
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
