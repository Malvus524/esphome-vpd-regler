#pragma once
// ==================================================================
// Fallback controller without room sensor
// ------------------------------------------------------------------
// Runs instead of the Kalman controller when no room_temperature /
// room_humidity is configured, or when a configured room sensor fails
// (VpdControllerCore switches). It only needs the tent climate (and the
// leaf temperature), so it cannot estimate the moisture load. Instead it
// finds the sensible upper limit of the fan by trying it out.
// Pure C++ like vpd_kalman_core.h, same Inputs/Outputs/Event.
//
// Ported from a YAML lambda (three levels, tick 10 s):
//
// LEVEL 1 - BASE CONTROLLER (PI on a logarithmic fan scale)
//   x = ln(u + u0) instead of u, with u0 = 100 * q0 / (1 - q0), so
//   x = ln q + const: equal steps in x are equal RELATIVE airflow changes,
//   5 -> 10 % is a big step, 40 -> 45 % a small one.
//     e   = VPD - effective target
//     e_t = e without deadband (0 inside, else distance to the band edge)
//     x  += -Ki * ( tau * (e_t - e_t_prev)  +  e_t * 10 s )
//           P part: reacts to changes     I part: speed ~ distance
//     Ki  = ln(1 + rate/100) / 0.1 kPa
//   x is limited to [fan min, upper limit], so there is no windup.
//
// LEVEL 2 - LIMIT FINDER (only while the target is not reached)
//   Active when the VPD is below the band AND the base controller has sat
//   at the upper limit for 3 tau (fan steady), not during a setpoint
//   transition.
//     1. Regression line over the last 3 tau: drift d and level V0
//     2. Move the upper limit by one test step (down first, after a
//        discarded step up), the fan follows immediately
//     3. Wait 3 tau, V1 = mean of the last tau
//     4. dV = V1 - (V0 + d * t)       -> effect without the drift
//     5. J = (e / 0.1 kPa)^2 + cost * (u / 100 %)^2
//        before: J(e0, u_old)   after: J(e0 + dV - margin, u_new)
//        The step is kept if J after is smaller. The margin (0.01 kPa)
//        means the step has to be clearly better.
//     6. Kept -> next test in the same direction after 3 tau.
//        Discarded -> back, next test in the other direction, the waiting
//        time doubles (max. 60 min).
//   Abort: VPD in band (limit stays), safety, sensor error, tent open or
//   tent sensor change (limit back). Upper limit = fan max after a light
//   change, target change, tent sensor change, restart and switching the
//   control on.
//
// LEVEL 3 - SAFETY
//   output = max(u, temperature P, humidity P), the same as in the Kalman
//   controller (hysteresis, one notification per episode). While the
//   safety overrides, x stays where it is.
//
// Same as in the Kalman controller: tent sensor selection (external source
// or own sensors, leaf sensor or offset), holding the fan after boot,
// manual mode, "tent open" (hold the fan, max. duration), setpoint
// transition after a light change, "tent closed" and a tent sensor change,
// storing the controller output.
// The controlled VPD is the mean of the last 3 ticks (30 s), like the
// template sensor with sliding_window_moving_average of the lambda setup.
// Constants marked "Tuning" can be changed via struct Tuning.
//
// German variable names follow the original lambda, like in
// vpd_kalman_core.h.
// ==================================================================

#include "vpd_kalman_core.h"

namespace esphome {
namespace vpd_kalman {

// Texts only used by the fallback controller. Index with the Language value.
struct FallbackTexts {
  // State text sensor
  const char *limit_test_fmt, *at_limit_fmt, *down, *up;
  // Reasons
  const char *r_lights_off, *r_lights_on, *r_target, *r_abort_fmt, *r_test_fmt, *r_step_fmt, *kept, *discarded;
  // Why a limit test was aborted
  const char *a_control, *a_light, *a_target, *a_sensor, *a_safety, *a_band, *a_source, *a_open;
  // Log lines
  const char *log_step_fmt, *log_result_fmt;
  // Switching between Kalman and fallback controller (VpdControllerCore)
  const char *to_fallback_fmt, *to_kalman, *r_to_fallback, *r_to_kalman, *state_suffix;
  // Switch "use fallback controller"
  const char *r_forced_on, *r_forced_off, *state_suffix_forced;
};

static const FallbackTexts FALLBACK_TEXTS[2] = {
    {
        // ---------------- English ----------------
        "Limit test %s", "At upper limit %.0f %%", "down", "up",
        //
        "Lights off - upper limit reset", "Lights on - upper limit reset", "Target changed - upper limit reset",
        "Limit test aborted: %s", "Limit test: step %s", "Step %s %s (J %.2f -> %.2f)", "kept", "discarded",
        //
        "control switched", "light change", "target changed", "sensor error", "safety override", "VPD in band",
        "tent sensor changed", "tent open",
        //
        "Limit finder: step %s %.1f -> %.1f %%, drift %.4f kPa/min",
        "Limit finder: %s, dV %.3f kPa, upper limit %.1f %%",
        //
        "No room sensor values for %.0f min. The fallback controller without room sensor takes over.",
        "The room sensor delivers values again. The Kalman controller takes over again.",
        "Room sensor missing - fallback controller", "Room sensor back - Kalman controller", " (no room sensor)",
        //
        "Fallback controller switched on", "Fallback controller switched off - Kalman controller", " (fallback)",
    },
    {
        // ---------------- Deutsch ----------------
        "Grenztest %s", "An Obergrenze %.0f %%", "runter", "rauf",
        //
        "Licht aus - Obergrenze zurückgesetzt", "Licht an - Obergrenze zurückgesetzt",
        "Ziel geändert - Obergrenze zurückgesetzt", "Grenztest abgebrochen: %s", "Grenztest: Schritt %s",
        "Schritt %s %s (J %.2f -> %.2f)", "behalten", "verworfen",
        //
        "Regelung umgeschaltet", "Lichtwechsel", "Ziel geändert", "Sensorfehler", "Sicherheit übersteuert",
        "VPD im Band", "Zeltsensor gewechselt", "Zelt offen",
        //
        "Grenzfinder: Schritt %s %.1f -> %.1f %%, Drift %.4f kPa/min",
        "Grenzfinder: %s, dV %.3f kPa, Obergrenze %.1f %%",
        //
        "Raumsensor liefert seit %.0f min keine Werte. Der Ersatzregler ohne Raumsensor übernimmt.",
        "Der Raumsensor liefert wieder Werte. Der Kalman-Regler übernimmt wieder.",
        "Raumsensor fehlt - Ersatzregler", "Raumsensor wieder da - Kalman-Regler", " (ohne Raumsensor)",
        //
        "Ersatzregler per Schalter eingeschaltet", "Ersatzregler per Schalter aus - Kalman-Regler",
        " (Ersatzregler)",
    },
};

class VpdFallbackCore : public CoreBase {
 public:
  // ---------- Configuration ----------
  void set_language(Language lang) {
    this->lang_ = lang;
    this->grund = TEXTS[lang].r_restart;
  }
  void set_airflow_at_zero(float q0) { this->q0_ = q0; }
  void set_leaf_max_deviation(float d) { this->leaf_max_dev_ = d; }

  // ---------- Taking over from the Kalman controller ----------
  // Called right before step(), after CoreBase was copied from the Kalman
  // controller. Takes over the fan without a jump. The upper limit starts at
  // upper_limit (the last sensible maximum of the Kalman controller, NAN =
  // fan max), but never below the current fan.
  void take_over(const Inputs &in, float upper_limit, const std::string &reason) {
    auto par = [](float v, float def) { return std::isnan(v) ? def : v; };
    auto begrenze = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };
    float u_min = par(in.fan_min, 5.0f);
    float u_max = std::max(par(in.fan_max, 100.0f), u_min);
    bool nacht = in.night_has_state ? in.night : nacht_vorher;
    gestartet = true;
    u = begrenze(this->fan_level, u_min, u_max);
    u_grenze = std::isnan(upper_limit) ? u_max : begrenze(std::max(upper_limit, u), u_min, u_max);
    ziel_vorher = par(nacht ? in.target_night : in.target_day, 1.0f);
    // Verlauf und Grenzfinder fangen neu an, der alte Stand ist veraltet
    test = false; t_stat = 0; t_pause = 0; pause_soll = 0; richtung = -1;
    et_gueltig = false; gn = 0; vn = 0;
    drift = dv = j0 = j1 = NAN;
    ziel_eff = NAN; fuehr_starten = false;
    zustand_vorher.clear();
    grund = reason;
  }

  /// Publish the state text again in the next tick (its suffix changed).
  void republish_state() { zustand_vorher.clear(); }

  // ---------- One control tick (10 s) ----------
  void step(const Inputs &in, Outputs &out) {
    const Texts &tx = TEXTS[this->lang_];
    const FallbackTexts &fx = FALLBACK_TEXTS[this->lang_];
    out.events.clear();

    // ---------- Konstanten ----------
    const Tuning &tn = in.tuning;                      // Einstellparameter, NAN = Standard
    const float DT = 10.0f / 60.0f;                    // Taktzeit in Minuten
    const float U0 = 100.0f * this->q0_ / (1.0f - this->q0_);   // Offset der log-Skala in %
    const float MARGE = tuning_or(tn.limit_test_margin, 0.01f);            // Messreserve im Grenztest, kPa
    const int GLAETTEN = std::min(tuning_ticks(tn.fallback_smoothing, 0.1f, 3), (int) GLAETTEN_MAX);  // Takte (30 s)
    const int BOOT_WARTEN = tuning_ticks(tn.boot_wait_tent, 0.1f, 6);          // Takte (60 s) bis zum ersten Zeltwert
    const int BOOT_WARTEN_NACHT = tuning_ticks(tn.boot_wait_night, 0.1f, 18);  // Takte (3 min) bis Tag/Nacht bekannt
    const int BOOT_ZAEHLEN = std::max(BOOT_WARTEN, BOOT_WARTEN_NACHT);
    const float T_HYST = tuning_or(tn.temperature_hysteresis, 0.5f);   // Hysterese Sicherheit Temperatur, K
    const float RH_HYST = tuning_or(tn.humidity_hysteresis, 3.0f);     // Hysterese Sicherheit Feuchte, %rF
    const int SCHUTZ_RUHE = tuning_ticks(tn.all_clear_after, 6.0f, 360);   // Takte (60 min) = Episode vorbei

    // ---------- Einstellwerte ----------
    auto par = [](float v, float def) { return std::isnan(v) ? def : v; };
    auto begrenze = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };
    bool nacht = in.night_has_state ? in.night : nacht_vorher;
    float ziel    = par(nacht ? in.target_night : in.target_day, 1.0f);
    float band    = par(in.deadband, 0.05f);
    float uebergang = begrenze(par(in.transition, 45.0f), 0.0f, 180.0f);
    float offen_max = begrenze(par(in.open_max, 30.0f), 5.0f, 240.0f);
    float tau     = std::max(par(tn.fallback_time_constant, 2.0f), 0.5f);
    float tempo   = std::max(par(tn.fallback_rate, 10.0f), 1.0f);
    float lam     = par(tn.limit_finder_cost, 4.0f);
    float schritt = par(tn.limit_finder_step, 15.0f);
    float u_min   = par(in.fan_min, 5.0f);
    float u_max   = std::max(par(in.fan_max, 100.0f), u_min);
    float notlauf = par(in.emergency, 40.0f);
    float t_max   = par(in.temp_max, 28.0f);
    float t_band  = std::max(par(in.temp_band, 3.0f), 0.5f);
    float rh_max  = par(in.rh_max, 75.0f);
    float rh_band = std::max(par(in.rh_band, 10.0f), 1.0f);

    float ki = logf(1.0f + tempo / 100.0f) / 0.1f;          // ln-Einheiten je kPa und min
    int n_tau = std::max(3, (int) lroundf(tau / DT));      // Takte je Tau
    int n_mess = std::min((int) NBUF, 3 * n_tau);          // 3 Tau
    int pause_max = tuning_ticks(tn.limit_test_max_pause, 6.0f, (int) lroundf(60.0f / DT));   // 60 min
    if (pause_soll <= 0) pause_soll = n_mess;

    // ---------- Hilfsfunktionen ----------
    // Saettigungsdampfdruck (kPa, Magnus)
    auto svp = [](float t) { return 0.6107f * powf(10.0f, 7.5f * t / (237.3f + t)); };
    auto melden = [&](const char *titel, const std::string &text) {
      Event e{Event::MESSAGE};
      e.a = titel;
      e.b = text;
      out.events.push_back(std::move(e));
    };
    auto loggen = [&](const char *text) {
      Event l{Event::LOG};
      l.a = text;
      out.events.push_back(std::move(l));
    };
    char buf[240];

    // ---------- Messen (wie im Kalman-Regler) ----------
    // Externe Quelle mit eigener Blatttemperatur, wenn gewaehlt und
    // vollstaendig. Sonst eigener Sensor: Blatttemperatur vom Blattsensor
    // (Schalter AN, plausibel), sonst Zelttemperatur + Blatt-Offset Tag/Nacht.
    bool unten_gewaehlt = in.ext.selected;
    bool unten = unten_gewaehlt && !std::isnan(in.ext.temperature) &&
                 !std::isnan(in.ext.humidity) && !std::isnan(in.ext.leaf_temperature);
    float T, RH, T_blatt;
    if (unten) {
      T = in.ext.temperature; RH = in.ext.humidity; T_blatt = in.ext.leaf_temperature;
    } else {
      T = in.t; RH = in.rh;
      float offset = par(nacht ? in.leaf_offset_night : in.leaf_offset_day, -2.0f);
      float mlx = in.leaf;
      T_blatt = (in.leaf_switch && !std::isnan(mlx) &&
                 fabsf(mlx - T) <= this->leaf_max_dev_) ? mlx : T + offset;
    }
    bool mess_ok = !std::isnan(T) && !std::isnan(RH) && !std::isnan(T_blatt);
    bool auto_an = in.auto_on;
    float aus_vorher = this->fan_level;
    if (mess_ok) zelt_je_ok = true;
    if (t_boot < BOOT_ZAEHLEN) t_boot++;
    // Wechsel der Quelle nach der Startphase: der VPD springt, Glaettung
    // und Verlauf gehoeren zum alten Sensor
    bool quelle_neu = t_boot >= BOOT_WARTEN_NACHT && quelle_vorher >= 0 && quelle_vorher != (int) unten;
    quelle_vorher = unten ? 1 : 0;
    if (quelle_neu) { gn = 0; vn = 0; }

    // Regelgroesse: Mittel der letzten GLAETTEN Messungen (30 s)
    float vpd = NAN;
    if (mess_ok) {
      gbuf[gh] = svp(T_blatt) - svp(T) * RH / 100.0f;
      gh = (gh + 1) % GLAETTEN_MAX; if (gn < GLAETTEN_MAX) gn++;
      int m = std::min(gn, GLAETTEN);
      float s = 0;
      for (int i = 1; i <= m; i++) s += gbuf[(gh - i + GLAETTEN_MAX) % GLAETTEN_MAX];
      vpd = s / m;
      vbuf[vh] = vpd; vh = (vh + 1) % NBUF; if (vn < NBUF) vn++;
    }

    // Ausgleichsgerade ueber die letzten m VPD-Werte: Steigung in
    // kPa/min und Wert der Geraden beim neuesten Messwert
    auto gerade = [&](int m, float &steigung, float &ende) -> bool {
      if (m < 3 || m > vn) return false;
      float sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (int i = 0; i < m; i++) {
        float x = i * DT, y = vbuf[(vh - m + i + NBUF) % NBUF];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
      }
      float nenner = m * sxx - sx * sx;
      if (nenner <= 0) return false;
      steigung = (m * sxy - sx * sy) / nenner;
      ende = sy / m + steigung * ((m - 1) * DT - sx / m);
      return true;
    };
    // Mittel der letzten m VPD-Werte
    auto mittel = [&](int m) -> float {
      float s = 0;
      for (int i = 1; i <= m; i++) s += vbuf[(vh - i + NBUF) % NBUF];
      return s / m;
    };
    // Kostenfunktion des Grenzfinders
    auto kosten = [&](float e, float uu) {
      float a = e / 0.1f, b = uu / 100.0f;
      return a * a + lam * b * b;
    };
    auto test_abbrechen = [&](const char *warum, bool zurueck) {
      if (!test) return;
      if (zurueck) u_grenze = g_alt;
      test = false; t_stat = 0; t_pause = 0;
      snprintf(buf, sizeof(buf), fx.r_abort_fmt, warum);
      grund = buf;
    };

    // ---------- Sicherheit (wie im Kalman-Regler) ----------
    // Ueber der Schwelle sofort mindestens Notlauf, dann linear ueber
    // das P-Band bis 100 %. Aus erst wieder unter Schwelle - Hysterese.
    auto sicherheit = [&](float wert, float schwelle, float p_band, float hyst, bool &aktiv) -> float {
      if (std::isnan(wert)) { aktiv = false; return 0.0f; }
      if (wert > schwelle) aktiv = true;
      else if (wert < schwelle - hyst) aktiv = false;
      if (!aktiv) return 0.0f;
      return begrenze(notlauf + (100.0f - notlauf) * std::max(wert - schwelle, 0.0f) / p_band,
                      0.0f, 100.0f);
    };
    float u_temp = sicherheit(T, t_max, t_band, T_HYST, sich_t);
    float u_rh = sicherheit(RH, rh_max, rh_band, RH_HYST, sich_rh);
    float u_sicher = std::max(u_temp, u_rh);

    // ---------- Start und Ereignisse ----------
    if (!gestartet) {
      gestartet = true;
      u = begrenze(par(this->saved_u, 30.0f), u_min, u_max);
      u_grenze = u_max;
      auto_vorher = auto_an; nacht_vorher = nacht; ziel_vorher = ziel;
    }
    // Der erste Wert der Nachtphase nach dem Booten ist kein Lichtwechsel
    if (!nacht_bekannt && in.night_has_state) {
      nacht_bekannt = true;
      nacht_vorher = nacht; ziel_vorher = ziel;
    }
    if (auto_an != auto_vorher) {
      auto_vorher = auto_an;
      test_abbrechen(fx.a_control, true);
      if (auto_an) {
        u = begrenze(aus_vorher, u_min, u_max);     // ohne Sprung uebernehmen
        u_grenze = u_max; et_gueltig = false; t_stat = 0;
        grund = tx.r_control_on;
      } else {
        grund = tx.r_control_off;
      }
    }
    // --- Zelt offen / wieder zu ---
    bool offen = in.tent_open;
    if (offen) {
      if (!offen_vorher) {
        t_offen = 0;
        test_abbrechen(fx.a_open, true);
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
      et_gueltig = false;
      grund = tx.r_closed;
    }
    offen_vorher = offen;
    // --- Lichtwechsel ---
    if (nacht_bekannt && nacht != nacht_vorher) {
      nacht_vorher = nacht;
      test_abbrechen(fx.a_light, false);
      u_grenze = u_max; t_stat = 0;
      ziel_vorher = ziel;                           // Zielwechsel gehoert zum Lichtwechsel
      fuehr_starten = true;
      grund = nacht ? fx.r_lights_off : fx.r_lights_on;
    }
    if (std::isnan(ziel_vorher) || fabsf(ziel - ziel_vorher) > 0.0005f) {
      ziel_vorher = ziel;
      test_abbrechen(fx.a_target, false);
      u_grenze = u_max; t_stat = 0;
      grund = fx.r_target;
    }
    if (quelle_neu) {
      test_abbrechen(fx.a_source, true);
      u_grenze = u_max; t_stat = 0; et_gueltig = false;
      fuehr_starten = true;
      grund = unten ? tx.r_source_ext : (unten_gewaehlt ? tx.r_source_ext_missing : tx.r_source_own);
    }
    u_grenze = begrenze(u_grenze, u_min, u_max);

    float ausgang;
    float ziel_w = ziel;                     // wirksames Ziel (Übergang)
    char zustand[48];
    if (!auto_an) {
      // ================= HAND =================
      ausgang = par(in.manual_speed, 50.0f);
      fuehr_starten = false; ziel_eff = NAN;  // kein alter Übergang beim Einschalten
      snprintf(zustand, sizeof(zustand), "%s", tx.manual);

    } else if ((!mess_ok && !zelt_je_ok && t_boot < BOOT_WARTEN) ||
               (mess_ok && !nacht_bekannt && t_boot < BOOT_WARTEN_NACHT)) {
      // == BOOT: erste Zeltwerte und Tag/Nacht abwarten, Luefter halten ==
      ausgang = std::max(aus_vorher, u_sicher);
      snprintf(zustand, sizeof(zustand), "%s", tx.waiting);

    } else if (!mess_ok) {
      // ================= SENSORFEHLER =================
      test_abbrechen(fx.a_sensor, true);
      et_gueltig = false; t_stat = 0;
      ausgang = std::max(notlauf, u_sicher);
      snprintf(zustand, sizeof(zustand), "%s", tx.tent_sensor_error);

    } else if (offen) {
      // ============ ZELT OFFEN: Luefter halten ============
      et_gueltig = false; t_stat = 0;
      ausgang = std::max(u, u_sicher);
      snprintf(zustand, sizeof(zustand), tx.tent_open_fmt, t_offen / 6);

    } else {
      // ---------- Soll mit Sollwert-Übergang (wie im Kalman-Regler) ----------
      if (fuehr_starten) {
        fuehr_starten = false;
        float abstand = fabsf(ziel - vpd);
        if (uebergang > 0.0f && abstand > band) {
          ziel_eff = vpd;
          fuehr_rate = abstand / (uebergang / DT);
        } else {
          ziel_eff = NAN;
        }
      }
      if (!std::isnan(ziel_eff)) {
        float richtung_z = ziel > ziel_eff ? 1.0f : -1.0f;
        ziel_eff += richtung_z * fuehr_rate;
        // VPD ist von selbst schon weiter Richtung Ziel -> mitgehen
        if ((vpd - ziel_eff) * richtung_z > 0.0f) ziel_eff = vpd;
        // Ziel erreicht oder ueberschritten -> Übergang vorbei
        if ((ziel - ziel_eff) * richtung_z <= 0.0f) ziel_eff = NAN;
      }
      bool uebergang_aktiv = !std::isnan(ziel_eff);
      if (uebergang_aktiv) ziel_w = ziel_eff;

      float e = vpd - ziel_w;
      float et = e > band ? e - band : (e < -band ? e + band : 0.0f);
      if (!et_gueltig) { et_vorher = et; et_gueltig = true; }
      bool sicher = u_sicher > u + 1.0f;
      t_pause++;

      // ================= EBENE 2: GRENZFINDER =================
      if (test) {
        t_test++;
        if (sicher) {
          test_abbrechen(fx.a_safety, true);
        } else if (e >= -band) {
          test_abbrechen(fx.a_band, false);
        } else if (t_test >= n_mess) {
          // Auswerten: V1 ist das Mittel der letzten Tau, sein
          // Zeitpunkt liegt eine halbe Tau vor jetzt
          float v1 = mittel(n_tau);
          float t_v1 = (t_test - (n_tau - 1) * 0.5f) * DT;
          dv = v1 - (v0 + drift * t_v1);
          j0 = kosten(e0, u_alt);
          j1 = kosten(e0 + dv - MARGE, u);
          const char *wohin = test_dir < 0 ? fx.down : fx.up;
          char erg[96];
          if (j1 < j0) {
            richtung = test_dir;
            pause_soll = n_mess;
            snprintf(erg, sizeof(erg), fx.r_step_fmt, wohin, fx.kept, j0, j1);
          } else {
            u_grenze = g_alt;
            richtung = -test_dir;
            pause_soll = std::min(2 * pause_soll, pause_max);
            snprintf(erg, sizeof(erg), fx.r_step_fmt, wohin, fx.discarded, j0, j1);
          }
          snprintf(buf, sizeof(buf), fx.log_result_fmt, erg, dv, u_grenze);
          loggen(buf);
          grund = erg;
          test = false; t_stat = 0; t_pause = 0;
        }
      } else if (e < -band && !sicher && !uebergang_aktiv && t_stat >= n_mess && t_pause >= pause_soll) {
        int dir = richtung;
        if (dir > 0 && u_grenze >= u_max - 0.05f) dir = -1;
        if (dir < 0 && u_grenze <= u_min + 0.05f) dir = (u_grenze < u_max - 0.05f) ? 1 : 0;
        float d, ende;
        if (dir != 0 && gerade(n_mess, d, ende)) {
          drift = d; v0 = ende; e0 = ende - ziel;
          g_alt = u_grenze; u_alt = u; test_dir = dir; t_test = 0; test = true;
          float xs = logf(u_grenze + U0) + dir * logf(1.0f + schritt / 100.0f);
          u_grenze = begrenze(expf(xs) - U0, u_min, u_max);
          u = u_grenze;                              // Luefter folgt sofort
          const char *wohin = dir < 0 ? fx.down : fx.up;
          snprintf(buf, sizeof(buf), fx.r_test_fmt, wohin);
          grund = buf;
          snprintf(buf, sizeof(buf), fx.log_step_fmt, wohin, g_alt, u_grenze, drift);
          loggen(buf);
        }
      }

      // ================= EBENE 1: GRUNDREGLER =================
      if (!sicher) {
        float x = logf(u + U0) - ki * (tau * (et - et_vorher) + et * DT);
        x = begrenze(x, logf(u_min + U0), logf(u_grenze + U0));
        u = expf(x) - U0;
      }
      et_vorher = et;
      u = begrenze(u, u_min, u_grenze);

      // ================= EBENE 3: SICHERHEIT =================
      ausgang = std::max(u, u_sicher);

      bool an_grenze = u >= u_grenze - 0.05f;
      t_stat = (!test && !sicher && an_grenze) ? t_stat + 1 : 0;

      if (sicher)
        snprintf(zustand, sizeof(zustand), tx.safety_fmt, u_temp >= u_rh ? tx.safety_temp : tx.safety_rh);
      else if (test)
        snprintf(zustand, sizeof(zustand), fx.limit_test_fmt, test_dir < 0 ? fx.down : fx.up);
      else if (an_grenze && e < -band)
        snprintf(zustand, sizeof(zustand), fx.at_limit_fmt, u_grenze);
      else
        snprintf(zustand, sizeof(zustand), "%s", uebergang_aktiv ? tx.regulating_transition : tx.regulating);
    }

    // ---------- Ausgeben ----------
    ausgang = begrenze(par(ausgang, notlauf), 1.0f, 100.0f);
    if (!(fabsf(ausgang - aus_vorher) <= 0.01f)) {
      Event e{Event::SET_LEVEL};
      e.level = ausgang / 100.0f;
      out.events.push_back(std::move(e));
    }
    this->fan_level = ausgang;

    // ---------- Schutz melden: eine Meldung je Episode (wie im Kalman-Regler) ----------
    bool schutz[2] = {sich_t && auto_an, sich_rh && auto_an};
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

    // Stellwert selten sichern (Flash schonen)
    if (auto_an && ++t_sichern >= 90) {
      t_sichern = 0;
      if (fabsf(this->saved_u - u) >= 1.0f) this->saved_u = u;
    }

    // ---------- Diagnose ----------
    if (zustand_vorher != zustand) {
      zustand_vorher = zustand;
      Event e{Event::STATE_TEXT};
      e.a = zustand_vorher;
      out.events.push_back(std::move(e));
      snprintf(buf, sizeof(buf), tx.log_state_fmt, zustand);
      loggen(buf);
    }
    if (grund_vorher != grund) {
      grund_vorher = grund;
      snprintf(buf, sizeof(buf), tx.log_reason_fmt, grund.c_str());
      loggen(buf);
    }
    out.control_vpd = vpd;
    out.target_active = ziel_w;
    out.controller_output = u;
    out.sensible_max = u_grenze;
    out.fan_output = ausgang;
    out.limit_finder_drift = drift;
    out.limit_finder_vpd_change = dv;
    out.limit_finder_cost_before = j0;
    out.limit_finder_cost_after = j1;
  }

 protected:
  static constexpr int NBUF = 180;          // VPD-Verlauf: 30 min
  static constexpr int GLAETTEN_MAX = 30;   // Takte (5 min) laengste Glaettung der Regelgroesse

  Language lang_{LANGUAGE_EN};
  float q0_{0.2f};
  float leaf_max_dev_{6.0f};

  // ---------- Speicher (war "static" im Lambda, gemeinsamer Teil in CoreBase) ----------
  bool gestartet = false;
  float ziel_vorher = NAN;
  float u = 30.0f;                   // Stellwert des Grundreglers in %
  float u_grenze = 100.0f;           // Obergrenze aus dem Grenzfinder in %
  float et_vorher = 0.0f;            // e_t des letzten Takts
  bool et_gueltig = false;           // false = P-Anteil neu ansetzen
  float gbuf[GLAETTEN_MAX] = {0}; int gn = 0, gh = 0;   // Glaettung der Regelgroesse
  float vbuf[NBUF] = {0}; int vn = 0, vh = 0;           // VPD-Verlauf
  float ziel_eff = NAN;              // wirksames Ziel im Sollwert-Übergang (NAN = keiner)
  float fuehr_rate = 0.0f;           // Mindestschritt des wirksamen Ziels je Takt, kPa
  bool fuehr_starten = false;        // Übergang beim naechsten Regeltakt starten

  // Grenzfinder
  int t_stat = 0;                    // Takte stationaer an der Obergrenze
  int t_pause = 0;                   // Takte seit dem letzten Test
  int pause_soll = 0;                // Wartezeit bis zum naechsten Test
  int richtung = -1;                 // naechster Test: -1 runter, +1 rauf
  bool test = false;
  int t_test = 0, test_dir = 0;
  float g_alt = 0, u_alt = 0, v0 = 0, e0 = 0;
  float drift = NAN, dv = NAN, j0 = NAN, j1 = NAN;

  std::string grund = "Restart";     // letztes Ereignis, nur fuers Log
  std::string zustand_vorher, grund_vorher;
  int t_sichern = 0;
};

}  // namespace vpd_kalman
}  // namespace esphome
