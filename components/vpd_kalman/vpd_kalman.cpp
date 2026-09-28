#include "vpd_kalman.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace vpd_kalman {

static const char *const TAG = "vpd";

// ---------- VpdNumber ----------
void VpdNumber::setup() {
  float value;
  this->pref_ = this->make_entity_preference<float>();
  if (!this->pref_.load(&value)) {
    value = std::isnan(this->initial_value_) ? this->traits.get_min_value() : this->initial_value_;
  }
  this->publish_state(value);
}

void VpdNumber::control(float value) {
  this->publish_state(value);
  this->pref_.save(&value);
}

// ---------- VpdSwitch ----------
void VpdSwitch::setup() {
  optional<bool> initial_state = this->get_initial_state_with_restore_mode();
  if (initial_state.has_value()) {
    if (initial_state.value()) {
      this->turn_on();
    } else {
      this->turn_off();
    }
  }
}

// ---------- VpdKalman ----------
// Registered at code generation time (before any setup), so they also see
// the first state that is restored during boot - exactly like on_value /
// on_turn_off automations would.
void VpdKalman::set_manual_speed_number(VpdNumber *n) {
  this->manual_speed_ = n;
  n->add_on_state_callback([this](float x) {
    // In automatic the controller drives the fan - only apply in manual mode.
    float level;
    if (this->core_.active().manual_level(this->control_ != nullptr && this->control_->state, x, level))
      this->apply_level_(level);
  });
}

void VpdKalman::set_control_switch(VpdSwitch *s) {
  this->control_ = s;
  s->add_on_state_callback([this](bool state) {
    if (!state)
      this->apply_level_(this->core_.active().control_off_level(state_of_(this->manual_speed_)));
  });
}

void VpdKalman::apply_level_(float level) {
  if (this->output_ != nullptr)
    this->output_->set_level(level / 100.0f);
}

void VpdKalman::publish_binary_(binary_sensor::BinarySensor *b, bool state) {
  if (b != nullptr && (!b->has_state() || b->state != state))
    b->publish_state(state);
}

void VpdKalman::setup() {
  // Stored controller values (same keys as restoring float globals)
  this->pref_u_ = global_preferences->make_preference<float>(this->key_u_);
  this->pref_day_ = global_preferences->make_preference<float>(this->key_day_);
  this->pref_night_ = global_preferences->make_preference<float>(this->key_night_);
  CoreBase &fan = this->core_.active();
  this->pref_u_.load(&fan.saved_u);
  this->pref_day_.load(&fan.saved_u_day);
  this->pref_night_.load(&fan.saved_u_night);
  this->stored_u_ = fan.saved_u;
  this->stored_day_ = fan.saved_u_day;
  this->stored_night_ = fan.saved_u_night;

  // Set the fan right away, not only at the first control tick.
  // Automatic: last stored controller value. Manual: manual speed.
  this->apply_level_(fan.boot_level(this->control_->state, state_of_(this->manual_speed_)));
}

void VpdKalman::update() {
  Inputs &in = this->in_;
  in.now_ms = millis();
  if (this->night_ != nullptr) {
    in.night_has_state = this->night_->has_state();
    in.night = this->night_->state;
  } else {
    in.night_has_state = true;  // always day: no boot wait, no light change
    in.night = false;
  }
  in.ext = this->external_climate_ ? this->external_climate_() : ExternalClimate{};
  in.t = state_of_(this->temperature_);
  in.rh = state_of_(this->humidity_);
  in.leaf = state_of_(this->leaf_temperature_);
  in.leaf_switch = this->leaf_switch_->state;
  in.room_t = state_of_(this->room_temperature_);
  in.room_rh = state_of_(this->room_humidity_);
  in.auto_on = this->control_->state;
  in.tent_open = this->tent_open_ != nullptr && this->tent_open_->state;
  in.force_fallback = this->force_fallback_ != nullptr && this->force_fallback_->state;
  in.manual_speed = state_of_(this->manual_speed_);
  if (this->target_ != nullptr) {
    in.target_day = in.target_night = this->target_->state;
  } else {
    in.target_day = state_of_(this->target_day_);
    in.target_night = state_of_(this->target_night_);
  }
  in.deadband = state_of_(this->deadband_);
  in.sacrifice = state_of_(this->sacrifice_);
  in.speed = state_of_(this->speed_);
  in.transition = state_of_(this->transition_);
  in.open_max = state_of_(this->open_max_);
  in.fan_min = state_of_(this->fan_min_);
  in.fan_max = state_of_(this->fan_max_);
  in.emergency = state_of_(this->emergency_);
  in.temp_max = state_of_(this->temp_max_);
  in.temp_band = state_of_(this->temp_band_);
  in.rh_max = state_of_(this->rh_max_);
  in.rh_band = state_of_(this->rh_band_);
  if (this->leaf_offset_ != nullptr) {
    in.leaf_offset_day = in.leaf_offset_night = this->leaf_offset_->state;
  } else {
    in.leaf_offset_day = state_of_(this->leaf_offset_day_);
    in.leaf_offset_night = state_of_(this->leaf_offset_night_);
  }
  for (int i = 0; i < TUNING_COUNT; i++) {
    in.tuning.*TUNING_FIELDS[i] =
        this->tuning_number_[i] != nullptr ? this->tuning_number_[i]->state : this->tuning_value_[i];
  }

  this->core_.step(in, this->out_);

  // Side effects in the order the controller produced them
  for (auto &e : this->out_.events) {
    switch (e.kind) {
      case Event::TURN_OFF_OPEN:
        if (this->tent_open_ != nullptr)
          this->tent_open_->turn_off();
        break;
      case Event::MESSAGE:
        this->message_callback_.call(e.a, e.b);
        break;
      case Event::SET_LEVEL:
        if (this->output_ != nullptr)
          this->output_->set_level(e.level);
        break;
      case Event::TEMP_PROTECTION:
        publish_binary_(this->b_temp_prot_, e.flag);
        break;
      case Event::RH_PROTECTION:
        publish_binary_(this->b_rh_prot_, e.flag);
        break;
      case Event::STATE_TEXT:
        if (this->t_state_ != nullptr)
          this->t_state_->publish_state(e.a);
        break;
      case Event::LOG:
        ESP_LOGI(TAG, "%s", e.a.c_str());
        break;
    }
  }

  // Store rarely changing controller values when they changed
  const CoreBase &fan = this->core_.active();
  if (memcmp(&this->stored_u_, &fan.saved_u, sizeof(float)) != 0) {
    this->stored_u_ = fan.saved_u;
    this->pref_u_.save(&this->stored_u_);
  }
  if (memcmp(&this->stored_day_, &fan.saved_u_day, sizeof(float)) != 0) {
    this->stored_day_ = fan.saved_u_day;
    this->pref_day_.save(&this->stored_day_);
  }
  if (memcmp(&this->stored_night_, &fan.saved_u_night, sizeof(float)) != 0) {
    this->stored_night_ = fan.saved_u_night;
    this->pref_night_.save(&this->stored_night_);
  }

  // Diagnostics
  const Outputs &o = this->out_;
  publish_binary_(this->b_fallback_, o.fallback_active);
  if (this->s_control_vpd_ != nullptr)
    this->s_control_vpd_->publish_state(o.control_vpd);
  if (this->s_target_active_ != nullptr)
    this->s_target_active_->publish_state(o.target_active);
  if (this->s_controller_output_ != nullptr)
    this->s_controller_output_->publish_state(o.controller_output);
  if (this->s_sensible_max_ != nullptr)
    this->s_sensible_max_->publish_state(o.sensible_max);
  if (this->s_fan_output_ != nullptr)
    this->s_fan_output_->publish_state(o.fan_output);
  if (this->s_excess_ != nullptr)
    this->s_excess_->publish_state(o.excess);
  if (this->s_excess_target_ != nullptr)
    this->s_excess_target_->publish_state(o.excess_target);
  if (this->s_moisture_load_ != nullptr)
    this->s_moisture_load_->publish_state(o.moisture_load);
  if (this->s_next_step_benefit_ != nullptr)
    this->s_next_step_benefit_->publish_state(o.next_step_benefit);
  if (this->s_vpd_at_max_ != nullptr)
    this->s_vpd_at_max_->publish_state(o.vpd_at_max);
  if (this->s_limit_finder_drift_ != nullptr)
    this->s_limit_finder_drift_->publish_state(o.limit_finder_drift);
  if (this->s_limit_finder_vpd_change_ != nullptr)
    this->s_limit_finder_vpd_change_->publish_state(o.limit_finder_vpd_change);
  if (this->s_limit_finder_cost_before_ != nullptr)
    this->s_limit_finder_cost_before_->publish_state(o.limit_finder_cost_before);
  if (this->s_limit_finder_cost_after_ != nullptr)
    this->s_limit_finder_cost_after_->publish_state(o.limit_finder_cost_after);
}

void VpdKalman::dump_config() {
  ESP_LOGCONFIG(TAG, "VPD Kalman controller:");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->room_temperature_ == nullptr
                                         ? "fallback controller only (no room sensor)"
                                         : "Kalman filter, fallback controller if the room sensor fails");
  ESP_LOGCONFIG(TAG, "  Day/night sensor: %s", this->night_ != nullptr ? "yes" : "no (always day)");
  ESP_LOGCONFIG(TAG, "  VPD target: %s", this->target_ != nullptr ? "single" : "day/night");
  ESP_LOGCONFIG(TAG, "  External tent climate: %s", this->external_climate_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Leaf temperature sensor: %s", this->leaf_temperature_ != nullptr ? "yes" : "no");
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace vpd_kalman
}  // namespace esphome
