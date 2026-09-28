#pragma once
// ==================================================================
// Switching between the Kalman controller and the fallback controller
// ------------------------------------------------------------------
// Pure C++ like the two controllers, used by the ESPHome glue.
//
// No room sensor configured: only the fallback controller runs.
// Room sensor configured: the Kalman controller runs. If the room sensor
// has no valid value for "room fallback delay" (2 min), the fallback
// controller takes over. Once the room sensor has delivered valid values
// for "room return delay" (1 min), the Kalman controller takes over again.
// Shorter dropouts behave as before (the Kalman controller holds the fan).
//
// On a switch the shared state (CoreBase: fan level, stored values, event
// memory, safety episodes) is copied to the controller that takes over, so
// the fan does not jump and no event is reported twice. The fallback
// starts its upper limit at the last sensible maximum of the Kalman
// controller. Each automatic switch sends a notification. While the
// fallback runs, the state text gets a suffix "(no room sensor)".
//
// Switch "use fallback controller" (optional, Inputs::force_fallback): ON
// hands over to the fallback right away, e.g. to compare both controllers
// with the room sensor still recording. OFF hands back to the Kalman
// controller right away if the room sensor has a value, otherwise the
// normal return delay applies. No notification, suffix "(fallback)".
// ==================================================================

#include "vpd_fallback_core.h"
#include "vpd_kalman_core.h"

namespace esphome {
namespace vpd_kalman {

class VpdControllerCore {
 public:
  VpdKalmanCore kalman;
  VpdFallbackCore fallback;

  // ---------- Configuration ----------
  void set_room_configured(bool configured) { this->room_configured_ = configured; }
  void set_language(Language lang) {
    this->lang_ = lang;
    this->kalman.set_language(lang);
    this->fallback.set_language(lang);
  }
  void set_airflow_at_zero(float q0) {
    this->kalman.set_airflow_at_zero(q0);
    this->fallback.set_airflow_at_zero(q0);
  }
  void set_leaf_max_deviation(float d) {
    this->kalman.set_leaf_max_deviation(d);
    this->fallback.set_leaf_max_deviation(d);
  }

  /// True while the fallback controller drives the fan.
  bool fallback_active() const { return !this->room_configured_ || this->fallback_active_; }
  /// Shared state of the active controller (fan level, stored values).
  CoreBase &active() { return this->fallback_active() ? static_cast<CoreBase &>(this->fallback) : this->kalman; }
  const CoreBase &active() const {
    return this->fallback_active() ? static_cast<const CoreBase &>(this->fallback) : this->kalman;
  }

  // ---------- One control tick (10 s) ----------
  void step(const Inputs &in, Outputs &out) {
    out.learned_airflow_50 = out.learned_offset = out.learned_lag = out.learned_sensible_max = NAN;
    out.fan_curve_active = false;
    if (!this->room_configured_) {
      this->fallback.step(in, out);
      out.fallback_active = true;
      return;
    }
    const FallbackTexts &fx = FALLBACK_TEXTS[this->lang_];
    const Texts &tx = TEXTS[this->lang_];
    const Tuning &tn = in.tuning;
    const float weg_min = tuning_or(tn.room_fallback_delay, 2.0f);
    const int WEG_TAKTE = tuning_ticks(tn.room_fallback_delay, 6.0f, 12);      // 2 min ohne Raumwert
    const int ZURUECK_TAKTE = tuning_ticks(tn.room_return_delay, 6.0f, 6);     // 1 min mit Raumwerten

    bool raum_ok = valid_climate(in.room_t, in.room_rh);
    bool erzwungen_vorher = this->erzwungen_;
    bool melden = false;
    char buf[200];
    auto zum_ersatzregler = [&](const char *grund) {
      this->fallback_active_ = true;
      this->t_raum_ = 0;
      this->nacht_abgabe_ = in.night_has_state ? in.night : this->nacht_abgabe_;
      static_cast<CoreBase &>(this->fallback) = static_cast<const CoreBase &>(this->kalman);
      this->fallback.take_over(in, this->kalman.sensible_max(), grund);
    };
    auto zum_kalman = [&](const char *grund) {
      this->fallback_active_ = false;
      this->erzwungen_ = false;
      this->t_raum_ = 0;
      bool nacht = in.night_has_state ? in.night : this->nacht_abgabe_;
      static_cast<CoreBase &>(this->kalman) = static_cast<const CoreBase &>(this->fallback);
      this->kalman.take_over(in, nacht != this->nacht_abgabe_, grund);
    };
    if (!this->fallback_active_) {
      this->t_raum_ = raum_ok ? 0 : this->t_raum_ + 1;
      if (in.force_fallback) {
        // Schalter: sofort zum Ersatzregler
        this->erzwungen_ = true;
        zum_ersatzregler(fx.r_forced_on);
      } else if (this->t_raum_ >= WEG_TAKTE) {
        // Raumsensor fehlt: Kalman -> Ersatzregler
        zum_ersatzregler(fx.r_to_fallback);
        snprintf(buf, sizeof(buf), fx.to_fallback_fmt, weg_min);
        melden = true;
      }
    } else if (in.force_fallback) {
      // Schalter AN haelt den Ersatzregler, auch wenn der Raumsensor fehlt
      this->erzwungen_ = true;
      this->t_raum_ = 0;
    } else if (this->erzwungen_ && raum_ok) {
      // Schalter AUS und Raumwert da: sofort zurueck
      zum_kalman(fx.r_forced_off);
    } else {
      // Automatischer Ersatzregler (oder Schalter AUS ohne Raumwert):
      // zurueck erst nach der Rueckkehr-Verzoegerung
      if (this->erzwungen_) {
        this->erzwungen_ = false;
        this->t_raum_ = 0;
      }
      this->t_raum_ = raum_ok ? this->t_raum_ + 1 : 0;
      if (this->t_raum_ >= ZURUECK_TAKTE) {
        zum_kalman(fx.r_to_kalman);
        snprintf(buf, sizeof(buf), "%s", fx.to_kalman);
        melden = true;
      }
    }

    // Anderer Zusatz im Zustandstext, wenn der Schalter den laufenden
    // Ersatzregler uebernimmt oder freigibt
    if (this->fallback_active_ && this->erzwungen_ != erzwungen_vorher) this->fallback.republish_state();

    // Die Kalman-Diagnose gibt es nur im Kalman-Regler und umgekehrt
    if (this->fallback_active_) {
      out.excess = out.excess_target = out.moisture_load = out.next_step_benefit = out.vpd_at_max = NAN;
      this->fallback.step(in, out);
      for (auto &e : out.events)
        if (e.kind == Event::STATE_TEXT) e.a += this->erzwungen_ ? fx.state_suffix_forced : fx.state_suffix;
    } else {
      out.limit_finder_drift = out.limit_finder_vpd_change = NAN;
      out.limit_finder_cost_before = out.limit_finder_cost_after = NAN;
      this->kalman.step(in, out);
    }
    out.fallback_active = this->fallback_active_;
    if (melden) {
      Event e{Event::MESSAGE};
      e.a = tx.title;
      e.b = buf;
      out.events.insert(out.events.begin(), std::move(e));
    }
  }

 protected:
  Language lang_{LANGUAGE_EN};
  bool room_configured_{false};
  bool fallback_active_{false};
  bool erzwungen_{false};      // Ersatzregler laeuft wegen des Schalters
  int t_raum_{0};              // Takte ohne (Kalman) bzw. mit (Ersatzregler) gueltigem Raumwert
  bool nacht_abgabe_{false};   // Tag/Nacht bei der Abgabe an den Ersatzregler
};

}  // namespace vpd_kalman
}  // namespace esphome
