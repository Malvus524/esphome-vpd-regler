#pragma once

#include <functional>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "vpd_controller_core.h"

namespace esphome {
namespace vpd_kalman {

// Setting stored in flash, behaves like an optimistic template number with
// restore_value: true (same preference key, so values survive a migration
// from template numbers with the same name).
class VpdNumber : public number::Number, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_initial_value(float value) { this->initial_value_ = value; }

 protected:
  void control(float value) override;
  float initial_value_{NAN};
  ESPPreferenceObject pref_;
};

// Optimistic switch with restore mode, like a template switch.
class VpdSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE - 2.0f; }

 protected:
  void write_state(bool state) override { this->publish_state(state); }
};

class VpdKalman : public PollingComponent {
 public:
  VpdKalman() : PollingComponent(10000) {
    for (float &v : this->tuning_value_)
      v = NAN;
  }

  void setup() override;
  void update() override;
  void dump_config() override;
  // Same point in the boot sequence as "on_boot: priority: -100", i.e.
  // after all stored values have been restored.
  float get_setup_priority() const override { return -100.0f; }

  // ---------- Configuration ----------
  void set_output(output::FloatOutput *out) { this->output_ = out; }
  void set_night(binary_sensor::BinarySensor *s) { this->night_ = s; }
  void set_temperature(sensor::Sensor *s) { this->temperature_ = s; }
  void set_humidity(sensor::Sensor *s) { this->humidity_ = s; }
  void set_leaf_temperature(sensor::Sensor *s) { this->leaf_temperature_ = s; }
  // Both room sensors set = Kalman controller with fallback, none = fallback only
  void set_room_temperature(sensor::Sensor *s) {
    this->room_temperature_ = s;
    this->core_.set_room_configured(this->room_temperature_ != nullptr && this->room_humidity_ != nullptr);
  }
  void set_room_humidity(sensor::Sensor *s) {
    this->room_humidity_ = s;
    this->core_.set_room_configured(this->room_temperature_ != nullptr && this->room_humidity_ != nullptr);
  }
  void set_external_climate(std::function<ExternalClimate()> &&f) { this->external_climate_ = f; }
  void set_language(Language lang) { this->core_.set_language(lang); }
  void set_airflow_at_zero(float q0) { this->core_.set_airflow_at_zero(q0); }
  void set_time_constant(uint32_t ms) { this->core_.kalman.set_time_constant_s(ms / 1000.0f); }
  void set_leaf_max_deviation(float d) { this->core_.set_leaf_max_deviation(d); }
  // Tuning parameter as fixed value or as number entity
  void set_tuning(TuningKey key, float value) { this->tuning_value_[key] = value; }
  void set_tuning_number(TuningKey key, VpdNumber *n) { this->tuning_number_[key] = n; }
  void set_storage_keys(uint32_t output, uint32_t day, uint32_t night) {
    this->key_u_ = output;
    this->key_day_ = day;
    this->key_night_ = night;
  }

  // Settings
  void set_manual_speed_number(VpdNumber *n);
  void set_target_day_number(VpdNumber *n) { this->target_day_ = n; }
  void set_target_night_number(VpdNumber *n) { this->target_night_ = n; }
  void set_deadband_number(VpdNumber *n) { this->deadband_ = n; }
  void set_vpd_sacrifice_number(VpdNumber *n) { this->sacrifice_ = n; }
  void set_speed_number(VpdNumber *n) { this->speed_ = n; }
  void set_transition_number(VpdNumber *n) { this->transition_ = n; }
  void set_tent_open_max_number(VpdNumber *n) { this->open_max_ = n; }
  void set_fan_min_number(VpdNumber *n) { this->fan_min_ = n; }
  void set_fan_max_number(VpdNumber *n) { this->fan_max_ = n; }
  void set_fan_emergency_number(VpdNumber *n) { this->emergency_ = n; }
  void set_temperature_max_number(VpdNumber *n) { this->temp_max_ = n; }
  void set_temperature_band_number(VpdNumber *n) { this->temp_band_ = n; }
  void set_humidity_max_number(VpdNumber *n) { this->rh_max_ = n; }
  void set_humidity_band_number(VpdNumber *n) { this->rh_band_ = n; }
  void set_leaf_offset_day_number(VpdNumber *n) { this->leaf_offset_day_ = n; }
  void set_leaf_offset_night_number(VpdNumber *n) { this->leaf_offset_night_ = n; }

  // Switches
  void set_control_switch(VpdSwitch *s);
  void set_tent_open_switch(VpdSwitch *s) { this->tent_open_ = s; }
  void set_leaf_sensor_switch(VpdSwitch *s) { this->leaf_switch_ = s; }
  void set_force_fallback_switch(VpdSwitch *s) { this->force_fallback_ = s; }

  // Diagnostics (all optional)
  void set_control_vpd_sensor(sensor::Sensor *s) { this->s_control_vpd_ = s; }
  void set_target_active_sensor(sensor::Sensor *s) { this->s_target_active_ = s; }
  void set_controller_output_sensor(sensor::Sensor *s) { this->s_controller_output_ = s; }
  void set_sensible_max_sensor(sensor::Sensor *s) { this->s_sensible_max_ = s; }
  void set_fan_output_sensor(sensor::Sensor *s) { this->s_fan_output_ = s; }
  void set_excess_sensor(sensor::Sensor *s) { this->s_excess_ = s; }
  void set_excess_target_sensor(sensor::Sensor *s) { this->s_excess_target_ = s; }
  void set_moisture_load_sensor(sensor::Sensor *s) { this->s_moisture_load_ = s; }
  void set_next_step_benefit_sensor(sensor::Sensor *s) { this->s_next_step_benefit_ = s; }
  void set_vpd_at_max_sensor(sensor::Sensor *s) { this->s_vpd_at_max_ = s; }
  void set_limit_finder_drift_sensor(sensor::Sensor *s) { this->s_limit_finder_drift_ = s; }
  void set_limit_finder_vpd_change_sensor(sensor::Sensor *s) { this->s_limit_finder_vpd_change_ = s; }
  void set_limit_finder_cost_before_sensor(sensor::Sensor *s) { this->s_limit_finder_cost_before_ = s; }
  void set_limit_finder_cost_after_sensor(sensor::Sensor *s) { this->s_limit_finder_cost_after_ = s; }
  void set_state_text_sensor(text_sensor::TextSensor *s) { this->t_state_ = s; }
  void set_temperature_protection_binary_sensor(binary_sensor::BinarySensor *s) { this->b_temp_prot_ = s; }
  void set_humidity_protection_binary_sensor(binary_sensor::BinarySensor *s) { this->b_rh_prot_ = s; }
  void set_fallback_active_binary_sensor(binary_sensor::BinarySensor *s) { this->b_fallback_ = s; }

  template<typename F> void add_on_message_callback(F &&callback) {
    this->message_callback_.add(std::forward<F>(callback));
  }

  /// Level currently applied to the fan output in %.
  float get_fan_level() const { return this->core_.active().fan_level; }
  /// True while the fallback controller without room sensor drives the fan.
  bool is_fallback_active() const { return this->core_.fallback_active(); }

 protected:
  void apply_level_(float level);
  static void publish_binary_(binary_sensor::BinarySensor *b, bool state);
  static float state_of_(number::Number *n) { return n == nullptr ? NAN : n->state; }
  static float state_of_(sensor::Sensor *s) { return s == nullptr ? NAN : s->state; }

  VpdControllerCore core_;
  Inputs in_;
  Outputs out_;

  output::FloatOutput *output_{nullptr};
  binary_sensor::BinarySensor *night_{nullptr};
  sensor::Sensor *temperature_{nullptr}, *humidity_{nullptr}, *leaf_temperature_{nullptr};
  sensor::Sensor *room_temperature_{nullptr}, *room_humidity_{nullptr};
  std::function<ExternalClimate()> external_climate_;

  VpdNumber *manual_speed_{nullptr}, *target_day_{nullptr}, *target_night_{nullptr}, *deadband_{nullptr},
      *sacrifice_{nullptr}, *speed_{nullptr}, *transition_{nullptr}, *open_max_{nullptr}, *fan_min_{nullptr},
      *fan_max_{nullptr}, *emergency_{nullptr}, *temp_max_{nullptr}, *temp_band_{nullptr}, *rh_max_{nullptr},
      *rh_band_{nullptr}, *leaf_offset_day_{nullptr}, *leaf_offset_night_{nullptr};
  float tuning_value_[TUNING_COUNT];         // NAN = not configured
  VpdNumber *tuning_number_[TUNING_COUNT] = {};
  VpdSwitch *control_{nullptr}, *tent_open_{nullptr}, *leaf_switch_{nullptr}, *force_fallback_{nullptr};

  sensor::Sensor *s_control_vpd_{nullptr}, *s_target_active_{nullptr}, *s_controller_output_{nullptr},
      *s_sensible_max_{nullptr}, *s_fan_output_{nullptr}, *s_excess_{nullptr}, *s_excess_target_{nullptr},
      *s_moisture_load_{nullptr}, *s_next_step_benefit_{nullptr}, *s_vpd_at_max_{nullptr};
  sensor::Sensor *s_limit_finder_drift_{nullptr}, *s_limit_finder_vpd_change_{nullptr},
      *s_limit_finder_cost_before_{nullptr}, *s_limit_finder_cost_after_{nullptr};
  text_sensor::TextSensor *t_state_{nullptr};
  binary_sensor::BinarySensor *b_temp_prot_{nullptr}, *b_rh_prot_{nullptr}, *b_fallback_{nullptr};

  CallbackManager<void(std::string, std::string)> message_callback_;

  // Stored controller values (same layout as restoring float globals)
  uint32_t key_u_{0}, key_day_{0}, key_night_{0};
  ESPPreferenceObject pref_u_, pref_day_, pref_night_;
  float stored_u_{NAN}, stored_day_{NAN}, stored_night_{NAN};
};

}  // namespace vpd_kalman
}  // namespace esphome
