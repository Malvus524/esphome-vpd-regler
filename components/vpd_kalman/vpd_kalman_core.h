#pragma once
// ==================================================================
// VPD controller core - excess-humidity controller with Kalman filter
// ------------------------------------------------------------------
// Pure C++ without any ESPHome dependency, so the exact same code runs in
// the firmware and in host-side tests. The ESPHome glue (vpd_kalman.h/.cpp)
// only copies entity states into Inputs, calls step() every 10 s and
// replays the resulting events.
//
// Model: the tent trails the room air. Everything is computed with vapour
// pressure (kPa), which stays the same when the intake air warms up.
//   e_tent = e_room + E            E  = vapour pressure excess (kPa)
//   E      = L / q(u)              L  = moisture load, q = rel. airflow
//   q(u)   = q0 + (1 - q0) * u     q0 = airflow at 0 %
// Room changes pass 1:1 into the tent, the fan divides E by q. The tent
// follows with the time constant tau/q (tau at 100 % fan). Direction is
// fixed: more exhaust = less moisture = higher VPD.
//
// Steps per tick (10 s):
//  1. Measure: tent (external source or own sensors), room, leaf, light
//  2. Convert: e_tent, e_room, E_meas = e_tent - e_room
//  3. Events
//     Light change: fan to the remembered value of this light phase,
//     raise the load uncertainty in the Kalman filter, start the setpoint
//     transition. Remembering is delayed: every 5 min the value from 5 min
//     ago is taken over, never in the first 20 min of a phase (the light
//     signal usually arrives a few minutes late).
//     Tent open: hold the fan, pause the Kalman filter and remembering,
//     safety stays active. Switches itself off after "tent open max.
//     duration". Tent closed: reset Kalman E to the measurement, start the
//     setpoint transition.
//  4. Kalman filter with the airflow that was actually applied, states:
//     true excess E, load L:
//       E_new  = f * E + (1 - f) / q * L    f = exp(-dt/tau * q)
//       E_meas = E
//     Outliers only count up to 4 sigma.
//  5. Sensible maximum from L: the fan setting at which full fan would only
//     gain the allowed VPD sacrifice X:
//       1/q_limit = X / L + 1/q_max
//     Changes by at most 5 %/min. A bank of filters learns the real fan
//     curve, the tent sensor offset and lag alongside (vpd_fan_curve.h).
//     Once it predicts clearly better than the configured straight curve,
//     the sensible maximum uses its curve and load instead.
//  6. Setpoint transition: after light change and "tent open" the
//     effective target starts at the measured VPD and moves towards the
//     target with |target - start| / transition time. If the VPD is
//     already further towards the target on its own, the effective target
//     follows it. Band edges from the leaf temperature:
//       E_upper (VPD = target - band) and E_lower (VPD = target + band)
//  7. Control law (soft deadband):
//       VPD < target - band:  ln q += k * dt * ln(E / E_upper)   -> more fan
//       VPD > target + band:  ln q += k * dt * ln(E / E_lower)   -> less
//       in between:  nothing
//     The error ln(...) is limited to +/-1. k = speed * q / tau.
//  8. Limit to [fan min, min(fan max, sensible maximum)]
//  9. Safety: output = max(u, temp P, humidity P), each with hysteresis
//     (0.5 K / 3 %RH). One notification per episode, all clear after
//     60 min without triggering.
// 10. Output, remember fan per light phase, store rarely, diagnostics
//
// Note: the code of step() is intentionally kept 1:1 with the original
// YAML lambda it was extracted from (German variable names included), so
// that equivalence can be verified bit by bit.
//
// Without room sensor VpdFallbackCore (vpd_fallback_core.h) runs instead,
// VpdControllerCore (vpd_controller_core.h) switches between the two. Both
// derive from CoreBase, which holds everything that is handed over on a
// switch (fan level, stored values, event memory, safety episodes).
//
// Tuning parameters (struct Tuning) replace constants of step(). NAN = not
// configured, then the original constant is used unchanged.
// ==================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "vpd_input.h"
#include "vpd_control_support.h"
#ifdef USE_VPD_FAN_CURVE
#include "vpd_fan_curve.h"
#endif

namespace esphome {
namespace vpd_kalman {

enum Language : uint8_t { LANGUAGE_EN = 0, LANGUAGE_DE = 1 };

// All user-visible texts. Index with the Language value.
struct Texts {
  // State text sensor
  const char *manual, *waiting, *tent_sensor_error, *room_missing, *tent_open_fmt, *safety_fmt, *safety_temp,
      *safety_rh, *in_band_transition, *in_band, *at_max, *unreachable, *at_min, *regulating_transition,
      *regulating, *protection_recovery;
  // Log lines ("reason" of the last event)
  const char *log_state_fmt, *log_reason_fmt, *r_restart, *r_source_ext, *r_source_ext_missing, *r_source_own,
      *r_control_on, *r_control_off, *r_open, *r_closed, *r_lights_off, *r_lights_on;
  // Notifications
  const char *title, *open_ended_fmt, *prot_temp, *prot_rh, *unit_temp, *unit_rh, *prot_active_fmt,
      *prot_clear_fmt;
};

static const Texts TEXTS[2] = {
    {
        // ---------------- English ----------------
        "Manual", "Waiting for sensors", "Tent sensor error", "Room sensor missing - fan held",
        "Tent open - paused (%d min)", "Safety (%s)", "temperature", "humidity", "In band (transition)", "In band",
        "At sensible maximum", "Target unreachable", "At fan minimum", "Regulating (transition)", "Regulating", "Protection recovery",
        //
        "State: %s", "Reason: %s", "Restart", "Tent sensor: external", "External tent sensor missing - using own",
        "Tent sensor: own", "Control switched on", "Control switched off", "Tent open - controller paused",
        "Tent closed - setpoint transition", "Lights off - fan to night value", "Lights on - fan to day value",
        //
        "VPD controller", "Tent open pause ended automatically after %.0f min, the controller continues.",
        "Temperature protection", "Humidity protection", "°C", "%RH",
        "Active: %.1f %s (limit %.1f %s), fan %.0f %%.",
        "All clear: %d trigger(s) in %lu min, highest value %.1f %s. Quiet for 60 min.",
    },
    {
        // ---------------- Deutsch ----------------
        "Hand", "Warte auf Sensoren", "Sensorfehler Zelt", "Raumsensor fehlt - Lüfter gehalten",
        "Zelt offen - pausiert (%d min)", "Sicherheit (%s)", "Temperatur", "Feuchte", "Im Band (Übergang)", "Im Band",
        "An sinnvollem Maximum", "Ziel unerreichbar", "An Lüfter min", "Regeln (Übergang)", "Regeln", "Rueckkehr nach Schutz",
        //
        "Zustand: %s", "Grund: %s", "Neustart", "Regelsensor: unten", "Sensor unten fehlt - Ersatz oben",
        "Regelsensor: oben", "Regelung eingeschaltet", "Regelung ausgeschaltet", "Zelt offen - Regler pausiert",
        "Zelt zu - Sollwert-Übergang", "Licht aus - Lüfter auf Nachtwert", "Licht an - Lüfter auf Tagwert",
        //
        "VPD-Regler", "Zelt-offen-Pause nach %.0f min automatisch beendet, der Regler läuft weiter.",
        "Temperaturschutz", "Feuchteschutz", "°C", "%rF", "Aktiv: %.1f %s (Grenze %.1f %s), Lüfter %.0f %%.",
        "Entwarnung: %d Auslösung(en) in %lu min, höchster Wert %.1f %s. Seit 60 min ruhig.",
    },
};

// Tent climate from an external source (e.g. a second sensor via radio).
// NAN in any field = not available, the controller then uses its own
// sensors. selected enables this source; sample_age_ms is the age of the
// oldest value in the packet, supplied by the receiver (not by this loop).
struct ExternalClimate {
  bool selected{false};
  float temperature{NAN};
  float humidity{NAN};
  float leaf_temperature{NAN};
  uint32_t sample_age_ms{UINT32_MAX};
};

// Optional tuning parameters, in the unit of their entity. NAN = default.
struct Tuning {
  // Switching Kalman <-> fallback when a configured room sensor fails
  float room_fallback_delay{NAN};    // min without room value until the fallback takes over
  float room_return_delay{NAN};      // min with valid room values until the Kalman controller is back
  // Remembering the fan per light phase (Kalman)
  float light_memory_after{NAN};     // min after a light change before remembering starts
  float light_memory_delay{NAN};     // min delay of the remembered value
  // Start
  float boot_wait_tent{NAN};         // s holding the fan until the first tent value
  float boot_wait_night{NAN};        // s holding the fan until day/night is known
  // Kalman filter and sensible maximum
  float sensible_max_rate{NAN};      // %/min
  float sensor_noise{NAN};           // kPa, standard deviation of the measured excess
  float load_change_per_hour{NAN};   // %/h
  // Safety
  float temperature_hysteresis{NAN}; // K
  float humidity_hysteresis{NAN};    // %RH
  float all_clear_after{NAN};        // min without triggering until the all clear
  // Fallback controller
  float fallback_time_constant{NAN}; // min
  float fallback_rate{NAN};          // %/min
  float limit_finder_cost{NAN};
  float limit_finder_step{NAN};      // %
  float fallback_smoothing{NAN};     // s
  float limit_test_margin{NAN};      // kPa
  float limit_test_max_pause{NAN};   // min
  float control_smoothing{NAN};      // s, Kalman control signal low-pass time constant
  float trend_horizon{NAN};          // s, maximum lookahead for an improving VPD
  float fan_increase_rate{NAN};      // percentage points/min, regular feedback corrections
  float protection_release_rate{NAN}; // percentage points/min, automatic recovery
};

enum TuningKey : uint8_t {
  TUNING_ROOM_FALLBACK_DELAY,
  TUNING_ROOM_RETURN_DELAY,
  TUNING_LIGHT_MEMORY_AFTER,
  TUNING_LIGHT_MEMORY_DELAY,
  TUNING_BOOT_WAIT_TENT,
  TUNING_BOOT_WAIT_NIGHT,
  TUNING_SENSIBLE_MAX_RATE,
  TUNING_SENSOR_NOISE,
  TUNING_LOAD_CHANGE_PER_HOUR,
  TUNING_TEMPERATURE_HYSTERESIS,
  TUNING_HUMIDITY_HYSTERESIS,
  TUNING_ALL_CLEAR_AFTER,
  TUNING_FALLBACK_TIME_CONSTANT,
  TUNING_FALLBACK_RATE,
  TUNING_LIMIT_FINDER_COST,
  TUNING_LIMIT_FINDER_STEP,
  TUNING_FALLBACK_SMOOTHING,
  TUNING_LIMIT_TEST_MARGIN,
  TUNING_LIMIT_TEST_MAX_PAUSE,
  TUNING_CONTROL_SMOOTHING,
  TUNING_TREND_HORIZON,
  TUNING_FAN_INCREASE_RATE,
  TUNING_PROTECTION_RELEASE_RATE,
  TUNING_COUNT,
};

// Field of each TuningKey
static float Tuning::*const TUNING_FIELDS[TUNING_COUNT] = {
    &Tuning::room_fallback_delay,    &Tuning::room_return_delay,      &Tuning::light_memory_after,
    &Tuning::light_memory_delay,     &Tuning::boot_wait_tent,         &Tuning::boot_wait_night,
    &Tuning::sensible_max_rate,      &Tuning::sensor_noise,           &Tuning::load_change_per_hour,
    &Tuning::temperature_hysteresis, &Tuning::humidity_hysteresis,    &Tuning::all_clear_after,
    &Tuning::fallback_time_constant, &Tuning::fallback_rate,          &Tuning::limit_finder_cost,
    &Tuning::limit_finder_step,      &Tuning::fallback_smoothing,     &Tuning::limit_test_margin,
    &Tuning::limit_test_max_pause, &Tuning::control_smoothing, &Tuning::trend_horizon,
    &Tuning::fan_increase_rate, &Tuning::protection_release_rate,
};

// Tuning value or default
inline float tuning_or(float v, float def) { return !std::isfinite(v) ? def : v; }
// Tuning value converted to ticks (ticks_per_unit), at least lo, or default
inline int tuning_ticks(float v, float ticks_per_unit, int def, int lo = 1) {
  return !std::isfinite(v) ? def : std::max(lo, (int) lroundf(v * ticks_per_unit));
}

struct Inputs {
  // Night phase (binary sensor, ON = night)
  bool night_has_state{false};
  bool night{false};
  // External tent climate (optional)
  ExternalClimate ext;
  // Own tent sensors, leaf sensor (NAN if none) and its switch
  float t{NAN}, rh{NAN}, leaf{NAN};
  bool leaf_switch{false};
  // Room
  float room_t{NAN}, room_rh{NAN};
  // Switches
  bool auto_on{false};
  bool safety_in_manual{true};
  uint32_t sensor_timeout_ms{120000};
  bool tent_open{false};
  bool force_fallback{false};   // switch "use fallback controller" (VpdControllerCore)
  // Settings (number entities)
  float manual_speed{NAN};
  float target_day{NAN}, target_night{NAN}, deadband{NAN}, sacrifice{NAN}, speed{NAN}, transition{NAN},
      open_max{NAN}, fan_min{NAN}, fan_max{NAN}, emergency{NAN}, temp_max{NAN}, temp_band{NAN}, rh_max{NAN},
      rh_band{NAN}, leaf_offset_day{NAN}, leaf_offset_night{NAN};
  Tuning tuning;
  uint32_t now_ms{0};
};

// Side effects of one tick, in the order they happened.
struct Event {
  enum Kind : uint8_t { TURN_OFF_OPEN, MESSAGE, SET_LEVEL, TEMP_PROTECTION, RH_PROTECTION, STATE_TEXT, LOG };
  Kind kind;
  float level{0};    // SET_LEVEL: 0..1
  bool flag{false};  // TEMP/RH_PROTECTION
  std::string a, b;  // MESSAGE: title, text; STATE_TEXT/LOG: text
};

struct Outputs {
  std::vector<Event> events;
  // Diagnostics, published every tick (NAN = unknown)
  float measured_vpd{NAN};  // unfiltered, for comparison with the actual control signal
  float control_vpd{NAN}, target_active{NAN}, controller_output{NAN}, sensible_max{NAN}, fan_output{NAN},
      excess{NAN}, excess_target{NAN}, moisture_load{NAN}, next_step_benefit{NAN}, vpd_at_max{NAN};
  // Fan curve learning (Kalman only): airflow at 50 % fan in % of full,
  // tent sensor offset against the room sensor (%RH), tent sensor lag (s)
  float learned_airflow_50{NAN}, learned_offset{NAN}, learned_lag{NAN};
  // Sensible maximum with the learned curve, always (also while not used)
  float learned_sensible_max{NAN};
  bool fan_curve_active{false};
  // Fallback controller only
  float limit_finder_drift{NAN}, limit_finder_vpd_change{NAN}, limit_finder_cost_before{NAN},
      limit_finder_cost_after{NAN};
  bool fallback_active{false};
};

// State shared by the Kalman controller and the fallback controller
// (vpd_fallback_core.h). On a switch it is copied to the controller that
// takes over, so the fan, remembered events and safety episodes carry on.
class CoreBase {
 public:
  // ---------- Persistent values (loaded/saved by the caller) ----------
  float saved_u{30.0f};       // controller output in automatic
  float saved_u_day{-1.0f};   // remembered fan for the day phase
  float saved_u_night{-1.0f}; // remembered fan for the night phase
  // Level currently applied to the fan output in %
  float fan_level{50.0f};

  // ---------- Fan output outside the control loop ----------
  // Boot: last controller value in automatic, manual speed otherwise.
  float boot_level(bool auto_on, float manual) {
    float level = auto_on ? this->saved_u : manual;
    if (std::isnan(level) || level < 1.0f) level = 1.0f;
    if (level > 100.0f) level = 100.0f;
    this->fan_level = level;
    return level;
  }
  // Manual speed changed. Returns false in automatic (nothing to do).
  bool manual_level(bool auto_on, float x, float &level) {
    if (auto_on) return false;
    this->protection_release_.reset();
    level = std::max(std::isfinite(x) ? x : 50.0f, this->manual_safety_floor_);
    // Failsafe: never switch off, catch broken values
    if (std::isnan(level) || level < 1.0f) level = 1.0f;
    if (level > 100.0f) level = 100.0f;
    this->fan_level = level;
    return true;
  }
  // Control switched off: back to the manual speed.
  float control_off_level(float manual) {
    this->protection_release_.reset();
    float level = std::max(std::isfinite(manual) ? manual : 50.0f, this->manual_safety_floor_);
    if (std::isnan(level) || level < 1.0f) level = 1.0f;
    if (level > 100.0f) level = 100.0f;
    this->fan_level = level;
    return level;
  }

 protected:
  float manual_safety_floor_{0.0f};
  Median3 vpd_median_;
  VpdResponse vpd_response_;
  ProtectionRelease protection_release_;
  float normal_output_{NAN};
  float handover_level_() const {
    return std::isfinite(this->normal_output_) && this->protection_release_.above(this->normal_output_)
               ? this->normal_output_ : this->fan_level;
  }
  // ---------- Event memory (was "static" in the lambdas) ----------
  bool auto_vorher = false, nacht_vorher = false;
  bool nacht_bekannt = false;       // Nachtphase hat schon einen Wert
  bool zelt_je_ok = false;          // seit dem Booten schon ein gueltiger Zeltwert
  int t_boot = 0;                   // Takte seit dem Booten (zaehlt bis BOOT_WARTEN(_NACHT))
  int quelle_vorher = -1;           // Zeltsensor im letzten Takt: 1 extern, 0 eigener, -1 noch keiner
  bool offen_vorher = false;        // Schalter "Zelt offen" im letzten Takt
  int t_offen = 0;                  // Takte seit "Zelt offen"
  bool sich_t = false, sich_rh = false;   // Sicherheitsstufe aktiv (Hysterese)
  // Schutz-Episoden, [0] Temperatur, [1] Feuchte
  bool schutz_vorher[2] = {false, false}, ep[2] = {false, false};
  int ep_runden[2] = {0, 0}, ep_ruhe[2] = {0, 0};
  uint32_t ep_start[2] = {0, 0}, ep_ende[2] = {0, 0};
  float ep_max[2] = {NAN, NAN};
};

class VpdKalmanCore : public CoreBase {
 public:
  // ---------- Configuration ----------
  void set_language(Language lang) {
    this->lang_ = lang;
    this->grund = TEXTS[lang].r_restart;
  }
  void set_airflow_at_zero(float q0) {
    this->q0_ = q0;
#ifdef USE_VPD_FAN_CURVE
    this->bank_.set_config(this->q0_, this->tau_s_);
#endif
  }
  void set_time_constant_s(float s) {
    this->tau_s_ = s;
#ifdef USE_VPD_FAN_CURVE
    this->bank_.set_config(this->q0_, this->tau_s_);
#endif
  }
#ifdef USE_VPD_FAN_CURVE
  /// Learn the fan curve (vpd_fan_curve.h). Off by default so that host
  /// tests can compare the controller bit by bit, the ESPHome glue turns it on.
  void set_fan_curve_learning(bool on) { this->learn_curve_ = on; }
  FanCurveBank &fan_curve() { return this->bank_; }
#endif
  void set_leaf_max_deviation(float d) { this->leaf_max_dev_ = d; }

  /// Current sensible maximum in % (NAN = not known yet).
  float sensible_max() const { return this->u_max_s; }

  // ---------- Taking over from the fallback controller ----------
  // Called right before step(), after CoreBase was copied from the
  // fallback. Takes over the fan without a jump and re-synchronises the
  // filter like after "tent open": E to the measurement, the load stays with
  // more uncertainty. light_changed: there was a light change meanwhile, so
  // remembering starts over.
  void take_over(const Inputs &in, bool light_changed, const std::string &reason) {
    auto par = [](float v, float def) { return !std::isfinite(v) ? def : v; };
    float u_max = std::min(100.0f, std::max(1.0f, par(in.fan_max, 100.0f)));
    float u_min = std::min(u_max, std::max(1.0f, par(in.fan_min, 5.0f)));
    if (!gestartet) {
      gestartet = true;
      u_phase[0] = this->saved_u_day;
      u_phase[1] = this->saved_u_night;
    }
    u = std::min(std::max(this->handover_level_(), u_min), u_max);
    if (!std::isnan(u_max_s)) u_max_s = std::max(u_max_s, u);
    if (light_changed) t_phase = 0;
    u_kand = -1.0f; t_kand = 0;
    kalman_neu = true; e_neu = false;
    this->vpd_median_.reset();
    this->vpd_response_.reset();
    this->economic_limit_.reset();
    ziel_eff = NAN; fuehr_starten = false; ramp_start_ = NAN;
    zustand_vorher.clear();
    grund = reason;
    previous_target_ = par(in.night_has_state && in.night ? in.target_night : in.target_day, 1.0f);
#ifdef USE_VPD_FAN_CURVE
    this->bank_.resume();
#endif
  }

  // ---------- One control tick (10 s) ----------
  void step(const Inputs &in, Outputs &out) {
    const Texts &tx = TEXTS[this->lang_];
    out.events.clear();

    // ---------- Konstanten ----------
    const float DT = 10.0f / 60.0f;          // Taktzeit in Minuten
    const float SCHRITT = 1.15f;             // Luefterschritt: +15 % Luftstrom
    const float E_KLEIN = 0.005f;            // kleinster Ueberschuss fuer ln(), kPa
    const float F_MAX = 1.0f;                // Obergrenze fuer |ln(E / Bandrand)|
    const Tuning &tn = in.tuning;            // Einstellparameter, NAN = Standard
    const int PHASE_MERKEN = tuning_ticks(tn.light_memory_after, 6.0f, 120, 0);  // Takte (20 min) bis zum Merken je Lichtphase
    const int MERK_TAKTE = tuning_ticks(tn.light_memory_delay, 6.0f, 30);        // Takte (5 min) Verzoegerung beim Merken
    const int BOOT_WARTEN = tuning_ticks(tn.boot_wait_tent, 0.1f, 6);            // Takte (60 s) Ausgang halten bis zum ersten Zeltwert
    const int BOOT_WARTEN_NACHT = tuning_ticks(tn.boot_wait_night, 0.1f, 18);    // Takte (3 min) Ausgang halten bis Tag/Nacht bekannt
    const int BOOT_ZAEHLEN = std::max(BOOT_WARTEN, BOOT_WARTEN_NACHT);
    const float MAX_RATE = tuning_or(tn.sensible_max_rate, 5.0f);   // Aenderung des sinnvollen Maximums, %/min
    const float R_MESS = std::isnan(tn.sensor_noise) ? 4.0e-5f       // Messrauschen von E, kPa^2 (~0,006 kPa)
                                                     : tn.sensor_noise * tn.sensor_noise;
    const float Q_E = 1.0e-5f;               // Modellfehler der Dynamik je Takt, kPa^2 (~0,003 kPa)
    const float L_REL_H = std::isnan(tn.load_change_per_hour) ? 0.10f   // Last darf sich ~10 % pro Stunde aendern
                                                              : tn.load_change_per_hour / 100.0f;
    const float L_TYP = 0.03f;               // Untergrenze fuer das Lastrauschen, kPa
    const float L_MIN = 0.001f;              // kleinste Last, kPa
    const float NIS_MAX = 16.0f;             // Ausreisser ab 4 Sigma
    const float T_HYST = tuning_or(tn.temperature_hysteresis, 0.5f);   // Hysterese Sicherheit Temperatur, K
    const float RH_HYST = tuning_or(tn.humidity_hysteresis, 3.0f);     // Hysterese Sicherheit Feuchte, %rF
    const int SCHUTZ_RUHE = tuning_ticks(tn.all_clear_after, 6.0f, 360);   // Takte (60 min) ohne Ausloesung = Episode vorbei
    const float q0 = this->q0_;                        // Luftstrom bei 0 %
    const float tau_voll = this->tau_s_ / 60.0f;       // min, Zeitkonstante bei 100 %

    // ---------- Einstellwerte ----------
    auto par = [](float v, float def) { return !std::isfinite(v) ? def : v; };
    auto begrenze = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };
    bool nacht = in.night_has_state ? in.night : nacht_vorher;
    float ziel    = par(nacht ? in.target_night : in.target_day, 1.0f);
    float band    = par(in.deadband, 0.05f);
    float verzicht = par(in.sacrifice, 0.03f);
    float tempo   = begrenze(par(in.speed, 0.25f), 0.05f, 1.0f);
    float uebergang = begrenze(par(in.transition, 45.0f), 0.0f, 180.0f);
    float offen_max = begrenze(par(in.open_max, 30.0f), 5.0f, 240.0f);
    float u_max   = begrenze(par(in.fan_max, 100.0f), 1.0f, 100.0f);
    float u_min   = begrenze(par(in.fan_min, 5.0f), 1.0f, u_max);
    float notlauf = begrenze(par(in.emergency, 40.0f), 1.0f, 100.0f);
    float t_max   = par(in.temp_max, 28.0f);
    float t_band  = std::max(par(in.temp_band, 3.0f), 0.5f);
    float rh_max  = par(in.rh_max, 75.0f);
    float rh_band = std::max(par(in.rh_band, 10.0f), 1.0f);

    // ---------- Hilfsfunktionen ----------
    // Saettigungsdampfdruck (kPa, Magnus)
    auto svp = [](float t) { return 0.6107f * powf(10.0f, 7.5f * t / (237.3f + t)); };
    auto q_von = [&](float uu) { return q0 + (1.0f - q0) * uu / 100.0f; };
    auto u_von = [&](float qq) { return (qq - q0) / (1.0f - q0) * 100.0f; };
    auto melden = [&](const char *titel, const std::string &text) {
      Event e{Event::MESSAGE};
      e.a = titel;
      e.b = text;
      out.events.push_back(std::move(e));
    };
    char buf[240];

    // ---------- 1. Messen ----------
    // Externe Quelle (z. B. Sensor unten per Funk) mit eigener
    // Blatttemperatur, wenn gewaehlt und vollstaendig. Sonst eigener
    // Sensor: Blatttemperatur vom Blattsensor (Schalter AN, plausibel),
    // sonst Zelttemperatur + Blatt-Offset Tag/Nacht.
    bool unten_gewaehlt = in.ext.selected;
    bool unten = unten_gewaehlt && valid_climate(in.ext.temperature, in.ext.humidity) &&
                 valid_temperature(in.ext.leaf_temperature) &&
                 fabsf(in.ext.leaf_temperature - in.ext.temperature) <= this->leaf_max_dev_ &&
                 in.ext.sample_age_ms < in.sensor_timeout_ms;
    float T, RH, T_blatt;
    if (unten) {
      T = in.ext.temperature; RH = in.ext.humidity; T_blatt = in.ext.leaf_temperature;
    } else {
      T = in.t; RH = in.rh;
      float offset = par(nacht ? in.leaf_offset_night : in.leaf_offset_day, -2.0f);
      float mlx = in.leaf;
      T_blatt = (in.leaf_switch && valid_temperature(mlx) &&
                 fabsf(mlx - T) <= this->leaf_max_dev_) ? mlx : T + offset;
    }
    float Tr = in.room_t, RHr = in.room_rh;
    bool zelt_ok = valid_climate(T, RH) && valid_temperature(T_blatt);
    // Regelgroesse zur Anzeige (Regelung rechnet unten gleich mit E)
    float vpd_regel_anz = zelt_ok ? svp(T_blatt) - svp(T) * RH / 100.0f : NAN;
    bool raum_ok = valid_climate(Tr, RHr);
    const bool signal_reset = !zelt_ok || in.tent_open ||
        (quelle_vorher >= 0 && quelle_vorher != (int) unten) ||
        (nacht_bekannt && nacht != nacht_vorher) || (offen_vorher && !in.tent_open);
    if (signal_reset) {
      this->vpd_median_.reset(); this->vpd_response_.reset();
      this->economic_limit_.reset();
    }
    const float filtered_vpd = zelt_ok
        ? this->vpd_response_.update(this->vpd_median_.update(vpd_regel_anz), 10.0f,
                                     tuning_or(tn.control_smoothing, 10.0f)) : NAN;
    bool auto_an = in.auto_on;
    float aus_vorher = this->fan_level;
    if (zelt_ok) zelt_je_ok = true;
    if (t_boot < BOOT_ZAEHLEN) t_boot++;
    // Wechsel der Quelle (Schalter oder Ausfall, nach der Startphase): Die
    // Sensoren sitzen an verschiedenen Stellen, der VPD springt. Kalman-E
    // neu auf den Messwert setzen (sonst haelt er den Sprung fuer eine
    // Laständerung) und den Sollwert-Übergang starten.
    if (t_boot >= BOOT_WARTEN_NACHT && quelle_vorher >= 0 && quelle_vorher != (int) unten) {
      e_neu = true;
      fuehr_starten = true;
      grund = unten ? tx.r_source_ext : (unten_gewaehlt ? tx.r_source_ext_missing : tx.r_source_own);
    }
    quelle_vorher = unten ? 1 : 0;

    // ---------- Sicherheit (vorab, wird in Schritt 9 angewendet) ----------
    // Ueber der Schwelle sofort mindestens Notlauf, dann linear ueber
    // das P-Band bis 100 %. Aus erst wieder unter Schwelle - Hysterese,
    // sonst flattert der Ausgang, wenn der Messwert um die Schwelle pendelt.
    float u_temp = protection_output(valid_temperature(T) ? T : NAN, t_max, t_band, T_HYST, notlauf, sich_t);
    float u_rh = protection_output(valid_humidity(RH) ? RH : NAN, rh_max, rh_band, RH_HYST, notlauf, sich_rh);
    float u_sicher = std::max(u_temp, u_rh);
    this->manual_safety_floor_ = in.safety_in_manual
        ? std::max(u_sicher, zelt_ok ? 0.0f : notlauf) : 0.0f;

    // ---------- Start und Hand/Automatik ----------
    if (!gestartet) {
      gestartet = true;
      u = begrenze(par(this->saved_u, 30.0f), u_min, u_max);
      u_phase[0] = this->saved_u_day;
      u_phase[1] = this->saved_u_night;
      auto_vorher = auto_an;
    }
    // Der erste Wert der Nachtphase nach dem Booten ist kein Lichtwechsel
    if (!nacht_bekannt && in.night_has_state) {
      nacht_bekannt = true;
      nacht_vorher = nacht;
    }
    if (auto_an != auto_vorher) {
      auto_vorher = auto_an;
      if (auto_an) {
        u = begrenze(aus_vorher, u_min, u_max);     // ohne Sprung uebernehmen
        // Liegt die Handdrehzahl ueber dem sinnvollen Maximum, sinkt
        // die Grenze von dort mit ihrer Rate, statt u sofort zu kappen
        if (!std::isnan(u_max_s)) u_max_s = std::max(u_max_s, u);
        u_kand = -1.0f; t_kand = 0;
        grund = tx.r_control_on;
      } else {
        grund = tx.r_control_off;
      }
    }
    // ---------- 3. Ereignisse ----------
    // --- Zelt offen / wieder zu ---
    bool offen = in.tent_open;
    if (offen) {
      if (!offen_vorher) {
        t_offen = 0;
        u_kand = -1.0f; t_kand = 0;           // nichts aus der Pause merken
        grund = tx.r_open;
      }
      if (++t_offen >= (int) roundf(offen_max / DT)) {
        out.events.push_back(Event{Event::TURN_OFF_OPEN});
        offen = false;
        snprintf(buf, sizeof(buf), tx.open_ended_fmt, offen_max);
        melden(tx.title, buf);
      }
    }
    if (offen_vorher && !offen) {
      fuehr_starten = true;
      kalman_neu = true;
      grund = tx.r_closed;
    }
    offen_vorher = offen;

    // --- Lichtwechsel ---
    if (t_phase < PHASE_MERKEN) t_phase++;
    bool licht_jetzt = false;               // fuer die Kennlinien-Bank
    if (nacht_bekannt && nacht != nacht_vorher) {
      licht_jetzt = true;
      nacht_vorher = nacht;
      t_phase = 0;
      u_kand = -1.0f; t_kand = 0;             // Kandidat gehoert zur alten Phase
      fuehr_starten = true;
      // Bei offenem Zelt bleibt der Luefter, wo er ist
      ramp_start_ = std::isfinite(ziel_eff) ? ziel_eff : this->previous_target_;
      if (auto_an && !offen && u_phase[nacht ? 1 : 0] > 0.0f) {
        u = begrenze(u_phase[nacht ? 1 : 0], u_min, u_max);
      }
      // Last kann gesprungen sein. Mindestens L_TYP, sonst bleibt eine
      // nachts fast auf 0 geschaetzte Last stundenlang dort haengen.
      if (!std::isnan(kl)) {
        float lb = std::max(kl, L_TYP);
        p11 += 4.0f * lb * lb;
      }
      grund = nacht ? tx.r_lights_off : tx.r_lights_on;
    }

    if (std::isfinite(this->previous_target_) && fabsf(ziel - this->previous_target_) > 0.0005f) {
      fuehr_starten = true;
      ramp_start_ = std::isfinite(ziel_eff) ? ziel_eff : this->previous_target_;
    }
    this->previous_target_ = ziel;

    // ---------- 2. Umrechnen und 4. Kalman-Filter ----------
    float e_zelt = NAN, e_raum = NAN, e_mess = NAN;
    if (zelt_ok && raum_ok) {
      e_zelt = svp(T) * RH / 100.0f;
      e_raum = svp(Tr) * RHr / 100.0f;
      e_mess = e_zelt - e_raum;
    }
    // Kennlinien-Bank: lernt immer mit dem eigenen Zeltsensor (die Kennlinie
    // gehoert zum Luefter, Versatz und Traegheit zu diesem Sensor)
#ifdef USE_VPD_FAN_CURVE
    if (this->learn_curve_) {
      bool eigen_ok = valid_climate(in.t, in.rh) && raum_ok;
      float c_eigen = eigen_ok ? svp(in.t) / 100.0f : NAN;
      this->bank_.step(eigen_ok, eigen_ok ? c_eigen * in.rh : NAN, eigen_ok ? svp(Tr) * RHr / 100.0f : NAN,
                       c_eigen, aus_vorher, licht_jetzt, offen, L_REL_H, R_MESS);
    }
#endif
    // Bei offenem Zelt gilt E = L/q nicht (Luft geht nicht nur durch den
    // Luefter) - Filter und sinnvolles Maximum pausieren.
    if (!std::isnan(e_mess) && !offen) {
      float q_ist = q_von(aus_vorher);
      if (std::isnan(kl)) {
        // Start: Gleichgewicht mit dem Luftstrom, der gerade anliegt
        ke = e_mess;
        kl = std::max(std::max(ke, E_KLEIN) * q_ist, L_MIN);
        float lb = std::max(kl, L_TYP);
        p00 = R_MESS; p01 = 0; p11 = lb * lb;
      } else if (kalman_neu) {
        // Zelt war offen: E ist jetzt wirklich niedrig, die Last kann
        // sich (Giessen, Umstellen) geaendert haben
        float lb = std::max(kl, L_TYP);
        ke = e_mess;
        p00 = R_MESS; p01 = 0; p11 += 4.0f * lb * lb;
      } else if (e_neu) {
        // Anderer Zeltsensor: Der Dampfdruck ist im Zelt fast gleich,
        // die Last bleibt - nur E auf den neuen Messwert setzen
        ke = e_mess;
        p00 = R_MESS; p01 = 0;
      }
      kalman_neu = false;
      e_neu = false;
      // Vorhersage ueber einen Takt (exakte Loesung bei konstantem q):
      // x = F x, P = F P F' + Q  mit F = [[f, b], [0, 1]]
      float f = expf(-DT / tau_voll * q_ist);
      float b = (1.0f - f) / q_ist;
      ke = f * ke + b * kl;
      float n00 = f * f * p00 + 2.0f * f * b * p01 + b * b * p11;
      float n01 = f * p01 + b * p11;
      float sl = L_REL_H * std::max(kl, L_TYP);
      p00 = n00 + Q_E; p01 = n01;
      p11 += sl * sl * DT / 60.0f;
      // Korrektur mit H = [1, 0]. Ausreisser gehen nur mit 4 Sigma ein:
      // s wird so vergroessert, dass die Korrektur begrenzt bleibt,
      // echte Spruenge aber in wenigen Takten folgen.
      float y = e_mess - ke;
      float s = p00 + R_MESS;
      float nis = y * y / s;
      if (nis > NIS_MAX) s *= sqrtf(nis / NIS_MAX);
      float g0 = p00 / s, g1 = p01 / s;
      ke += g0 * y; kl += g1 * y;
      float o00 = p00, o01 = p01;
      p00 = o00 - g0 * o00; p01 = o01 - g0 * o01; p11 -= g1 * o01;
      kl = std::max(kl, L_MIN);

      // ---------- 5. Sinnvolles Maximum (laufend, in jeder Betriebsart) ----------
      // Mit gelernter Kennlinie, sobald die Bank klar besser vorhersagt
      // Do not classify uncertain moisture removal as a negligible benefit.
      const float load_upper = kl + sqrtf(std::max(0.0f, p11));
      float ziel_max = verzicht > 0.0f
          ? u_von(1.0f / (verzicht / load_upper + 1.0f / q_von(u_max))) : u_max;
#ifdef USE_VPD_FAN_CURVE
      if (this->learn_curve_ && this->bank_.active())
        ziel_max = std::max(ziel_max, this->bank_.sensible_max(verzicht, u_max));
#endif
      ziel_max = begrenze(ziel_max, u_min, u_max);
      if (verzicht != this->previous_sacrifice_) {
        this->economic_limit_.reset();
        this->previous_sacrifice_ = verzicht;
      }
      ziel_max = this->economic_limit_.update(ziel_max);
      if (verzicht <= 0.0f) u_max_s = u_max;
      if (std::isnan(u_max_s)) u_max_s = ziel_max;   // erster Wert direkt
      u_max_s += begrenze(ziel_max - u_max_s, -MAX_RATE * DT, MAX_RATE * DT);
    }
    if (!std::isnan(u_max_s)) u_max_s = begrenze(u_max_s, u_min, u_max);
    float grenze = std::isnan(u_max_s) ? u_max : std::min(u_max, u_max_s);

    float ausgang;
    float ziel_w = ziel;                     // wirksames Ziel (Übergang)
    float e_wahr = NAN, e_soll = NAN, nutzen = NAN, vpd_max = NAN;
    char zustand[48];
    if (!auto_an) {
      // ================= HAND =================
      ausgang = std::max(par(in.manual_speed, 50.0f), this->manual_safety_floor_);
      fuehr_starten = false; ziel_eff = NAN; ramp_start_ = NAN;  // kein alter Übergang beim Einschalten
      if (in.safety_in_manual && !zelt_ok)
        snprintf(zustand, sizeof(zustand), "%s", tx.tent_sensor_error);
      else if (in.safety_in_manual && u_sicher > par(in.manual_speed, 50.0f))
        snprintf(zustand, sizeof(zustand), tx.safety_fmt, u_temp >= u_rh ? tx.safety_temp : tx.safety_rh);
      else
        snprintf(zustand, sizeof(zustand), "%s", tx.manual);

    } else if ((!zelt_ok && !zelt_je_ok && t_boot < BOOT_WARTEN) ||
               (zelt_ok && !nacht_bekannt && t_boot < BOOT_WARTEN_NACHT)) {
      // == BOOT: erste Zeltwerte und Tag/Nacht abwarten, Luefter halten ==
      ausgang = std::max(aus_vorher, u_sicher);
      snprintf(zustand, sizeof(zustand), "%s", tx.waiting);

    } else if (!zelt_ok) {
      // ================= SENSORFEHLER =================
      ausgang = std::max(notlauf, u_sicher);
      snprintf(zustand, sizeof(zustand), "%s", tx.tent_sensor_error);

    } else if (!raum_ok) {
      // ============ OHNE RAUMSENSOR: Luefter halten ============
      ausgang = std::max(u, u_sicher);
      snprintf(zustand, sizeof(zustand), "%s", tx.room_missing);

    } else if (offen) {
      // ============ ZELT OFFEN: Luefter halten ============
      ausgang = std::max(u, u_sicher);
      snprintf(zustand, sizeof(zustand), tx.tent_open_fmt, t_offen / 6);

    } else {
      float q = q_von(u);
      bool sicher = u_sicher > u + 1.0f || this->protection_release_.above(u);
      float e_blatt = svp(T_blatt);
      float vpd_regel = filtered_vpd;

      // ---------- 6. Soll mit Sollwert-Übergang ----------
      if (fuehr_starten) {
        fuehr_starten = false;
        target_transition_ = std::isfinite(ramp_start_);
        const float start = target_transition_ ? ramp_start_ : vpd_regel;
        ramp_start_ = NAN;
        float abstand = fabsf(ziel - start);
        if (uebergang > 0.0f && abstand > (target_transition_ ? 0.0005f : band)) {
          ziel_eff = start;
          fuehr_rate = abstand / (uebergang / DT);
        } else {
          ziel_eff = NAN;
        }
      }
      if (uebergang <= 0.0f) ziel_eff = NAN;
      if (std::isfinite(ziel_eff) && uebergang != previous_transition_)
        fuehr_rate = fabsf(ziel - ziel_eff) / (uebergang / DT);
      previous_transition_ = uebergang;
      if (!std::isnan(ziel_eff)) {
        float richtung = ziel > ziel_eff ? 1.0f : -1.0f;
        ziel_eff += richtung * fuehr_rate;
        // VPD ist von selbst schon weiter Richtung Ziel -> mitgehen
        if (!target_transition_ && (vpd_regel - ziel_eff) * richtung > 0.0f) ziel_eff = vpd_regel;
        // Ziel erreicht oder ueberschritten -> Übergang vorbei
        if ((ziel - ziel_eff) * richtung <= 0.0f) ziel_eff = NAN;
      }
      bool uebergang_aktiv = !std::isnan(ziel_eff);
      if (uebergang_aktiv) ziel_w = ziel_eff;

      // Ueberschuss, bei dem mit der Blatttemperatur ein bestimmter VPD herauskommt
      auto e_fuer = [&](float v) { return e_blatt - v - e_raum; };
      e_wahr = e_zelt - e_raum;
      const float e_regel = e_blatt - vpd_regel - e_raum;
      e_soll = e_fuer(ziel_w);
      float e_oben = e_fuer(ziel_w - band);    // feuchter darf es nicht werden
      float e_unten = e_fuer(ziel_w + band);   // trockener darf es nicht werden
      float q_voll = q_von(u_max);
      nutzen = kl / q - kl / std::min(SCHRITT * q, q_voll);
      vpd_max = e_blatt - (e_raum + kl / q_voll);
      bool erreichbar = vpd_max >= ziel_w - band;   // bei Lüfter max, gleiche Temperatur

      // ---------- 7. Regelgesetz (weiches Totband) ----------
      // Richtung aus dem VPD selbst (ungeklemmt), Groesse aus ln(E / Bandrand).
      // Geschrieben als ln(1 + Abstand / Nenner): ueber E_KLEIN exakt
      // ln(E / Bandrand), darunter stetig weiter statt eines Sprungs.
      float fehler = 0.0f;
      if (vpd_regel < ziel_w - band)          // zu feucht -> mehr Luefter
        fehler = std::max(log1pf((e_regel - e_oben) / std::max(e_oben, E_KLEIN)), 0.0f);
      else if (vpd_regel > ziel_w + band)     // zu trocken -> weniger
        fehler = std::min(-log1pf((e_unten - e_regel) / std::max(e_regel, E_KLEIN)), 0.0f);
      fehler = begrenze(fehler, -F_MAX, F_MAX);
      bool im_band = fehler == 0.0f;
      if (!sicher && !im_band) {
        float k = tempo * q / tau_voll;      // Tempo folgt dem Luftwechsel
        const float requested = u_von(q * expf(k * DT * fehler));
        u = limit_fan_increase(u, requested,
            this->vpd_response_.increase_factor(ziel_w - band, tuning_or(tn.trend_horizon, 30.0f)),
            tuning_or(tn.fan_increase_rate, 30.0f), 10.0f);
      }

      // ---------- 8. Begrenzen ----------
      u = begrenze(u, u_min, grenze);
      // Verzoegert merken: gespeichert wird der Kandidat von vor 5 min,
      // nie der aktuelle Wert (siehe Schritt 3 im Kopf)
      if (nacht_bekannt && t_phase >= PHASE_MERKEN && ++t_kand >= MERK_TAKTE) {
        t_kand = 0;
        if (u_kand > 0.0f) u_phase[nacht ? 1 : 0] = u_kand;
        u_kand = u;
      }

      // ---------- 9. Sicherheit ----------
      ausgang = std::max(u, u_sicher);

      if (sicher)
        snprintf(zustand, sizeof(zustand), tx.safety_fmt, u_temp >= u_rh ? tx.safety_temp : tx.safety_rh);
      else if (im_band)
        snprintf(zustand, sizeof(zustand), "%s", uebergang_aktiv ? tx.in_band_transition : tx.in_band);
      else if (fehler > 0.0f && u >= grenze - 0.05f)
        snprintf(zustand, sizeof(zustand), "%s", erreichbar ? tx.at_max : tx.unreachable);
      else if (fehler < 0.0f && u <= u_min + 0.05f)
        snprintf(zustand, sizeof(zustand), "%s", tx.at_min);
      else
        snprintf(zustand, sizeof(zustand), "%s", uebergang_aktiv ? tx.regulating_transition : tx.regulating);
    }

    // Release only the extra protection output gradually. Sensor failure and
    // explicit manual operation retain their own immediate output policies.
    this->normal_output_ = u;
    ausgang = this->protection_release_.apply(ausgang, u_sicher, auto_an && zelt_ok,
        tuning_or(tn.protection_release_rate, 20.0f), 10.0f);
    if (auto_an && zelt_ok && raum_ok && !offen && ausgang > std::max(u, u_sicher) + 0.01f)
      snprintf(zustand, sizeof(zustand), "%s", tx.protection_recovery);

    // ---------- 10. Ausgeben ----------
    ausgang = begrenze(par(ausgang, notlauf), 1.0f, 100.0f);
    if (!(fabsf(ausgang - aus_vorher) <= 0.01f)) {
      Event e{Event::SET_LEVEL};
      e.level = ausgang / 100.0f;
      out.events.push_back(std::move(e));
    }
    this->fan_level = ausgang;

    // ---------- Schutz melden: eine Meldung je Episode ----------
    // Protection can also override manual operation.
    bool protection_enabled = auto_an || in.safety_in_manual;
    bool schutz[2] = {sich_t && protection_enabled, sich_rh && protection_enabled};
    float s_wert[2] = {T, RH}, s_grenze[2] = {t_max, rh_max};
    const char *s_name[2] = {tx.prot_temp, tx.prot_rh};
    const char *s_einheit[2] = {tx.unit_temp, tx.unit_rh};
    for (int i = 0; i < 2; i++) {
      if (schutz[i]) {
        ep_ruhe[i] = 0;
        if (!(s_wert[i] <= ep_max[i])) ep_max[i] = s_wert[i];   // faengt NAN ab
        if (!schutz_vorher[i]) {
          if (!ep[i]) {
            ep[i] = true; ep_runden[i] = 1; ep_start[i] = in.now_ms; ep_max[i] = s_wert[i];
            snprintf(buf, sizeof(buf), tx.prot_active_fmt,
                     s_wert[i], s_einheit[i], s_grenze[i], s_einheit[i], ausgang);
            melden(s_name[i], buf);
          } else {
            ep_runden[i]++;
          }
        }
      } else if (ep[i]) {
        if (schutz_vorher[i]) ep_ende[i] = in.now_ms;
        if (++ep_ruhe[i] >= SCHUTZ_RUHE) {
          ep[i] = false;
          snprintf(buf, sizeof(buf), tx.prot_clear_fmt, ep_runden[i],
                   (unsigned long) ((ep_ende[i] - ep_start[i]) / 60000UL), ep_max[i], s_einheit[i]);
          melden(s_name[i], buf);
        }
      }
      schutz_vorher[i] = schutz[i];
    }
    {
      Event e{Event::TEMP_PROTECTION};
      e.flag = schutz[0];
      out.events.push_back(std::move(e));
      Event r{Event::RH_PROTECTION};
      r.flag = schutz[1];
      out.events.push_back(std::move(r));
    }

    // Selten sichern (Flash schonen): Stellwerte nur in Automatik
    if (++t_sichern >= 90) {
      t_sichern = 0;
      if (auto_an) {
        if (fabsf(this->saved_u - u) >= 1.0f) this->saved_u = u;
        if (fabsf(this->saved_u_day - u_phase[0]) >= 1.0f) this->saved_u_day = u_phase[0];
        if (fabsf(this->saved_u_night - u_phase[1]) >= 1.0f) this->saved_u_night = u_phase[1];
      }
    }

    // Diagnose
    if (zustand_vorher != zustand) {
      zustand_vorher = zustand;
      Event e{Event::STATE_TEXT};
      e.a = zustand_vorher;
      out.events.push_back(std::move(e));
      snprintf(buf, sizeof(buf), tx.log_state_fmt, zustand);
      Event l{Event::LOG};
      l.a = buf;
      out.events.push_back(std::move(l));
    }
    if (grund_vorher != grund) {
      grund_vorher = grund;
      snprintf(buf, sizeof(buf), tx.log_reason_fmt, grund.c_str());
      Event l{Event::LOG};
      l.a = buf;
      out.events.push_back(std::move(l));
    }
    out.measured_vpd = vpd_regel_anz;
    out.control_vpd = filtered_vpd;
    out.target_active = ziel_w;
    out.controller_output = u;
    out.sensible_max = u_max_s;
    out.fan_output = ausgang;
    out.excess = e_wahr;
    out.excess_target = e_soll;
    out.moisture_load = kl;
    out.next_step_benefit = nutzen;
    out.vpd_at_max = vpd_max;
#ifdef USE_VPD_FAN_CURVE
    if (this->learn_curve_) {
      out.learned_airflow_50 = 100.0f * this->bank_.airflow(50.0f);
      out.learned_offset = this->bank_.offset();
      out.learned_lag = this->bank_.lag();
      out.learned_sensible_max = begrenze(this->bank_.sensible_max(verzicht, u_max), u_min, u_max);
      out.fan_curve_active = this->bank_.active();
    }
#endif
  }

 protected:
  Language lang_{LANGUAGE_EN};
  float q0_{0.2f};
  float tau_s_{21.0f};
  float leaf_max_dev_{6.0f};
#ifdef USE_VPD_FAN_CURVE
  bool learn_curve_{false};
  FanCurveBank bank_;
#endif
  float previous_target_{NAN};
  float ramp_start_{NAN};
  float previous_transition_{NAN};
  bool target_transition_{false};

  // ---------- Speicher (war "static" im Lambda, gemeinsamer Teil in CoreBase) ----------
  bool gestartet = false;
  int t_phase = 0;                  // Takte seit dem letzten Lichtwechsel (bis PHASE_MERKEN)
  float u_kand = -1.0f;             // Kandidat zum Merken (Wert von vor MERK_TAKTE)
  int t_kand = 0;                   // Takte bis zum naechsten Merken
  float u = 30.0f;                  // Stellwert des Reglers in %
  float u_max_s = NAN;              // sinnvolles Maximum in %
  float u_phase[2] = {-1.0f, -1.0f};   // gemerkter Luefter [0] Tag, [1] Nacht
  float ke = 0.0f;                  // Kalman: wahrer Ueberschuss E (kPa)
  float kl = NAN;                   // Kalman: Last L (kPa)
  float p00 = 0, p01 = 0, p11 = 0;  // Kovarianz
  ConfirmedLimit economic_limit_;
  float previous_sacrifice_{NAN};
  bool kalman_neu = false;          // nach "Zelt offen": E neu auf den Messwert
  bool e_neu = false;               // nach Sensorwechsel: nur E neu auf den Messwert
  float ziel_eff = NAN;             // wirksames Ziel im Sollwert-Übergang (NAN = keiner)
  float fuehr_rate = 0.0f;          // Mindestschritt des wirksamen Ziels je Takt, kPa
  bool fuehr_starten = false;       // Übergang beim naechsten Regeltakt starten
  std::string grund = "Restart";    // letztes Ereignis, nur fuers Log
  std::string zustand_vorher, grund_vorher;
  int t_sichern = 0;
};

}  // namespace vpd_kalman
}  // namespace esphome
