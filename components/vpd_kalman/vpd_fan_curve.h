#pragma once
// ==================================================================
// Fan curve learning - a bank of small Kalman filters
// ------------------------------------------------------------------
// Pure C++ like vpd_kalman_core.h. The Kalman controller assumes a straight
// fan curve q(u) = q0 + (1 - q0) * u (airflow_at_zero). Real fans are often
// "saturated" early, and tent and room sensor usually disagree by a few %RH.
// Both shift the sensible maximum. They cannot be told apart from the humidity
// level alone, only together and through the dynamics after fan changes.
//
// Every candidate is one complete assumption about the setup:
//   fan curve   q(u) = q0 + (1 - q0) * s(u),  s(u) = (1 - exp(-u/k)) / (1 - exp(-100/k))
//               (k = 0: straight), q0 from {0.1, 0.2, 0.3, 0.4} with the
//               configured airflow_at_zero replacing the closest value
//   offset      v of the tent sensor against the room sensor, -4 ... +4 %RH
//   sensor lag  tau_s of the tent sensor, 0 / 15 / 30 / 60 s
// Each runs its own filter (tent vapour pressure, sensor reading, load) on
// the own tent sensor and scores how well it predicts the next reading - but
// only in the 10 min after a fan change. Only then do the readings tell the
// curves apart; in quiet phases the candidates would differ by their noise
// assumptions alone, which points to wrong curves. Scores fade with a half
// life of 48 h, so the bank follows a changed setup within a few days.
//
// The learned curve is only used when it predicts clearly better than what
// the Kalman filter assumes (configured curve, no offset): margin >= 30 (log
// likelihood), at least 20 min of clean informative data, and the learned curve
// has been stable for 48 h (within 0.05 at 25/50/75 %). Released below 15.
// Scores are booked in blocks of 10 informative minutes and only if even the
// best fitting candidate had no 3-sigma miss in the block: the fan reacts to
// whatever the model does not know (tent opened without the switch,
// watering), and such blocks would teach wrong curves. Until then nothing changes. When
// active, a weighted mix of all candidates (weight exp(0.03 * score))
// gives curve and load for the sensible maximum.
// ==================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace vpd_kalman {

class FanCurveBank {
 public:
  static constexpr int N_Q0 = 4, N_K = 5, N_V = 9, N_TS = 4;
  static constexpr int N_CURVE = N_Q0 * N_K;              // 20
  static constexpr int N_DYN = N_CURVE * N_TS;            // 80 (curve x lag share the dynamics)
  static constexpr int N = N_DYN * N_V;                   // 720 candidates
  static constexpr int HIST = 48;                         // hours the curve must be stable
  static constexpr uint32_t STORE_VERSION = 0x46430004;

  // Persistent part (scores), saved rarely by the caller
  struct Stored {
    uint32_t version;
    float q0_cfg;
    float tau_s;
    float count;          // informative ticks, fading like the scores
    float score[N];
    float hist[HIST][3];  // learned airflow at 25/50/75 % once per hour
    int32_t hist_n;
    int32_t active;
  };

  void set_config(float q0_cfg, float tau_s) {
    this->tau_ = tau_s;
    static const float GRID[N_Q0] = {0.1f, 0.2f, 0.3f, 0.4f};
    int nearest = 0;
    for (int i = 0; i < N_Q0; i++) {
      this->q0_[i] = GRID[i];
      if (fabsf(GRID[i] - q0_cfg) < fabsf(GRID[nearest] - q0_cfg)) nearest = i;
    }
    this->q0_[nearest] = q0_cfg;
    this->q0_cfg_ = q0_cfg;
    this->cfg_curve_ = nearest * N_K + 0;   // k = 0: straight
  }

  /// Restore scores (false = nothing usable stored).
  bool load(const Stored &s) {
    if (s.version != STORE_VERSION || s.q0_cfg != this->q0_cfg_ || s.tau_s != this->tau_ ||
        !std::isfinite(s.count) || s.count < 0.0f) return false;
    for (int i = 0; i < N; i++)
      if (!std::isfinite(s.score[i])) return false;
    for (int h = 0; h < HIST; h++)
      for (int j = 0; j < 3; j++)
        if (!std::isfinite(s.hist[h][j]) || s.hist[h][j] < 0.0f || s.hist[h][j] > 1.0f) return false;
    this->count_ = s.count;
    for (int i = 0; i < N; i++) this->score_[i] = s.score[i];
    this->hist_n_ = std::min(std::max((int) s.hist_n, 0), HIST);
    for (int h = 0; h < HIST; h++)
      for (int j = 0; j < 3; j++) this->hist_[h][j] = s.hist[h][j];
    this->active_ = s.active != 0;
    this->decide_();
    return true;
  }
  void save(Stored &s) const {
    s.version = STORE_VERSION;
    s.q0_cfg = this->q0_cfg_;
    s.tau_s = this->tau_;
    s.count = this->count_;
    for (int i = 0; i < N; i++) s.score[i] = this->score_[i];
    for (int h = 0; h < HIST; h++)
      for (int j = 0; j < 3; j++) s.hist[h][j] = this->hist_[h][j];
    s.hist_n = this->hist_n_;
    s.active = this->active_ ? 1 : 0;
  }

  // A fallback interval is not a single 10 s model step. Discard the
  // unfinished scoring block and reinitialise the dynamic state on return.
  void resume() {
    this->reinit_ = true;
    this->u_prev_ = NAN;
    this->since_change_ = 1 << 20;
    this->block_n_ = this->block_age_ = 0;
    for (float &value : this->block_) value = 0.0f;
    this->block_clean_ = false;
  }

  // ---------- One tick (10 s) ----------
  // e_tent: vapour pressure from the OWN tent sensor, c = svp(T_tent)/100,
  // u: fan level applied during the last tick (%). valid = tent and room ok.
  void step(bool valid, float e_tent, float e_room, float c, float u, bool light_change, bool tent_open,
            float load_change_h, float sensor_noise_var) {
    this->count_ *= LAMBDA;
    for (int i = 0; i < N; i++) this->score_[i] *= LAMBDA;
    if (light_change) {
      // The load jumps: no scoring for 30 min
      this->light_pending_ = true;
      this->quiet_ = std::max(this->quiet_, SETTLE_LIGHT);
      this->block_clean_ = false;
    }
    if (tent_open) {
      this->reinit_ = true;
      this->quiet_ = std::max(this->quiet_, SETTLE);
      this->block_clean_ = false;
      return;
    }
    if (!valid) {
      if (++this->gap_ > 6) this->reinit_ = true;   // over 1 min without values
      this->block_clean_ = false;
      return;
    }
    this->gap_ = 0;
    // Window after a fan change
    if (!std::isnan(this->u_prev_) && fabsf(u - this->u_prev_) > 1.0f) this->since_change_ = 0;
    else if (this->since_change_ < WINDOW) this->since_change_++;
    this->u_prev_ = u;

    const float r = sensor_noise_var * 0.5f, r_room = sensor_noise_var * 0.5f;
    if (this->reinit_) {
      for (int d = 0; d < N_DYN; d++) {
        float q = this->q_of(d / N_TS, u);
        for (int v = 0; v < N_V; v++) this->init_(d * N_V + v, e_tent, e_room, c, q, r, !this->started_);
      }
      this->started_ = true;
      this->reinit_ = false;
      this->light_pending_ = false;
      this->quiet_ = std::max(this->quiet_, SETTLE);
      this->block_clean_ = false;
      return;
    }
    if (fabsf(u - this->u_cache_) > 0.01f) {
      this->u_cache_ = u;
      for (int d = 0; d < N_DYN; d++) this->transition_(d, u);
    }
    bool scored = this->since_change_ < WINDOW && this->quiet_ == 0;
    if (this->quiet_ > 0) this->quiet_--;
    if (scored && this->block_n_ == 0) this->block_clean_ = true;   // a new block starts clean
    const float dt_h = 10.0f / 3600.0f;
    float min_nis = INFINITY;
    for (int d = 0; d < N_DYN; d++) {
      const float *F = this->phi_[d];
      const float bz = this->b_[d][0], bs = this->b_[d][1];
      float f = F[0];   // e_tent row: [f, 0, b]
      for (int v = 0; v < N_V; v++) {
        int i = d * N_V + v;
        float *x = this->x_[i], *P = this->p_[i];   // P: 00 01 02 11 12 22
        if (this->light_pending_) {
          float lb = std::max(x[2], L_TYP);
          P[5] += 4.0f * lb * lb;
        }
        // Prediction x = F x + b e_room, P = F P F' + Q (F rows: z, s, L)
        float xz = F[0] * x[0] + F[1] * x[1] + F[2] * x[2] + bz * e_room;
        float xs = F[3] * x[0] + F[4] * x[1] + F[5] * x[2] + bs * e_room;
        x[0] = xz;
        x[1] = xs;
        float Pm[3][3] = {{P[0], P[1], P[2]}, {P[1], P[3], P[4]}, {P[2], P[4], P[5]}};
        float T[3][3];
        for (int a = 0; a < 3; a++)
          for (int b = 0; b < 3; b++) T[a][b] = F[a * 3 + 0] * Pm[0][b] + F[a * 3 + 1] * Pm[1][b] + F[a * 3 + 2] * Pm[2][b];
        float N00 = T[0][0] * F[0] + T[0][1] * F[1] + T[0][2] * F[2];
        float N01 = T[0][0] * F[3] + T[0][1] * F[4] + T[0][2] * F[5];
        float N02 = T[0][2];
        float N11 = T[1][0] * F[3] + T[1][1] * F[4] + T[1][2] * F[5];
        float N12 = T[1][2];
        float N22 = T[2][2];
        float lb = std::max(x[2], L_TYP);
        N00 += Q_E + (1.0f - f) * (1.0f - f) * r_room;
        N22 += (load_change_h * lb) * (load_change_h * lb) * dt_h;
        // Correction with the reading of the tent sensor (offset v)
        float y = e_tent - (x[1] + c * OFFSET(v));
        float s = N11 + r;
        float nis = y * y / s;
        if (scored) this->block_[i] -= 0.5f * (logf(s) + std::min(nis, NIS_MAX));
        min_nis = std::min(min_nis, nis);
        if (nis > NIS_MAX) s *= sqrtf(nis / NIS_MAX);
        float g0 = N01 / s, g1 = N11 / s, g2 = N12 / s;
        x[0] += g0 * y;
        x[1] += g1 * y;
        x[2] = std::max(x[2] + g2 * y, L_MIN);
        P[0] = N00 - g0 * N01;
        P[1] = N01 - g0 * N11;
        P[2] = N02 - g0 * N12;
        P[3] = N11 - g1 * N11;
        P[4] = N12 - g1 * N12;
        P[5] = N22 - g2 * N12;
      }
    }
    this->light_pending_ = false;
    // Even the best fitting candidate misses: something the model does not
    // know happened (tent opened without the switch, watering, ...). The fan
    // reacts to it, so this block must not count.
    if (min_nis > NIS_BLOCK) this->block_clean_ = false;
    // Scores are collected in blocks of 10 min of informative ticks and only
    // booked if nothing unexpected happened within the block
    if (scored) this->block_n_++;
    if (this->block_n_ > 0) this->block_age_++;
    if (this->block_n_ >= BLOCK || this->block_age_ >= BLOCK_MAX_AGE) {
      if (this->block_clean_ && this->block_n_ >= BLOCK / 3) {
        for (int i = 0; i < N; i++) this->score_[i] += this->block_[i];
        this->count_ += (float) this->block_n_;
      }
      for (int i = 0; i < N; i++) this->block_[i] = 0.0f;
      this->block_n_ = 0;
      this->block_age_ = 0;
      this->block_clean_ = true;
    }
    this->decide_();
    // Once per hour: remember the learned curve (for the stability check)
    if (++this->hour_ticks_ >= 360) {
      this->hour_ticks_ = 0;
      for (int h = HIST - 1; h > 0; h--)
        for (int j = 0; j < 3; j++) this->hist_[h][j] = this->hist_[h - 1][j];
      this->hist_[0][0] = this->airflow(25.0f);
      this->hist_[0][1] = this->airflow(50.0f);
      this->hist_[0][2] = this->airflow(75.0f);
      if (this->hist_n_ < HIST) this->hist_n_++;
    }
  }

  // ---------- Results ----------
  /// True while the learned curve is used for the sensible maximum.
  bool active() const { return this->active_; }
  /// Score margin of the best candidate over the best with the configured curve.
  float margin() const { return this->margin_; }
  /// Clean informative ticks (fading), and whether the curve was stable for 48 h.
  float informative() const { return this->count_; }
  bool stable() const { return this->stable_(); }
  /// Mixed airflow at fan level u (0..1), load (kPa), offset (%RH), lag (s).
  float airflow(float u) const {
    float q = 0.0f;
    for (int c = 0; c < N_CURVE; c++)
      if (this->w_curve_[c] > 0.0f) q += this->w_curve_[c] * this->q_of(c, u);
    return q;
  }
  float load() const {
    const float *w = this->w_;
    float l = 0.0f;
    for (int i = 0; i < N; i++) l += w[i] * this->x_[i][2];
    return l;
  }
  float offset() const {
    const float *w = this->w_;
    float o = 0.0f;
    for (int i = 0; i < N; i++) o += w[i] * OFFSET(i % N_V);
    return o;
  }
  float lag() const {
    const float *w = this->w_;
    float t = 0.0f;
    for (int i = 0; i < N; i++) t += w[i] * LAG[(i / N_V) % N_TS];
    return t;
  }
  /// Sensible maximum with the learned curve: the fan setting at which full
  /// fan (u_max) would only gain the allowed VPD sacrifice.
  float sensible_max(float sacrifice, float u_max) const {
    if (!(sacrifice > 0.0f)) return u_max;
    float l = std::max(this->load(), L_MIN);
    float q_lim = 1.0f / (sacrifice / l + 1.0f / this->airflow(u_max));
    float lo = 0.0f, hi = u_max;
    for (int it = 0; it < 30; it++) {
      float m = 0.5f * (lo + hi);
      if (this->airflow(m) < q_lim) lo = m; else hi = m;
    }
    return 0.5f * (lo + hi);
  }

 protected:
  static constexpr float WINDOW = 60;          // ticks (10 min) scored after a fan change
  static constexpr int SETTLE = 30;            // ticks (5 min) not scored after (re)start
  static constexpr int SETTLE_LIGHT = 180;     // ticks (30 min) not scored after a light change
  static constexpr int BLOCK = 60;             // informative ticks per block
  static constexpr int BLOCK_MAX_AGE = 360;    // ticks (1 h): a block closes at the latest
  static constexpr float NIS_BLOCK = 9.0f;     // 3 sigma: the block is discarded
  static constexpr float NIS_MAX = 16.0f;
  static constexpr float Q_E = 1.0e-5f;
  static constexpr float L_TYP = 0.03f, L_MIN = 0.001f;
  static constexpr float BETA = 0.03f;
  static constexpr float MARGIN_ON = 30.0f, MARGIN_OFF = 15.0f;
  static constexpr float COUNT_ON = 120.0f, COUNT_OFF = 30.0f;     // 20 / 5 min of clean informative ticks
  static constexpr float STABLE = 0.05f;       // learned airflow may move at most this much in 48 h

  /// The learned curve has not moved by more than STABLE at 25/50/75 % over the last 48 h.
  bool stable_() const {
    if (this->hist_n_ < HIST) return false;
    for (int j = 0; j < 3; j++) {
      float lo = this->hist_[0][j], hi = lo;
      for (int h = 1; h < HIST; h++) {
        lo = std::min(lo, this->hist_[h][j]);
        hi = std::max(hi, this->hist_[h][j]);
      }
      if (hi - lo > STABLE) return false;
    }
    return true;
  }
  static constexpr float LAMBDA = 0.99995989f;  // 0.5^(10 s / 48 h)
  static constexpr float K_VAL[N_K] = {0.0f, 60.0f, 30.0f, 20.0f, 12.0f};
  static constexpr float LAG[N_TS] = {0.0f, 15.0f, 30.0f, 60.0f};
  static float OFFSET(int v) { return (float) (v - N_V / 2); }

  float q_of(int curve, float u) const {
    float q0 = this->q0_[curve / N_K], k = K_VAL[curve % N_K];
    u = std::min(std::max(u, 0.0f), 100.0f);
    float s = k <= 0.0f ? u / 100.0f : (1.0f - expf(-u / k)) / (1.0f - expf(-100.0f / k));
    return q0 + (1.0f - q0) * s;
  }

  void init_(int i, float e_tent, float e_room, float c, float q, float r, bool first) {
    float *x = this->x_[i], *P = this->p_[i];
    float z = e_tent - c * OFFSET(i % N_V);
    float l = first ? std::max(std::max(z - e_room, 0.005f) * q, L_MIN) : x[2];
    float lb = std::max(l, L_TYP);
    x[0] = x[1] = z;
    x[2] = l;
    P[0] = P[1] = P[3] = r;
    P[2] = P[4] = 0.0f;
    P[5] = first ? lb * lb : P[5] + 4.0f * lb * lb;
  }

  // Transition over one tick (20 steps of 0.5 s) for curve x lag d at fan u
  void transition_(int d, float u) {
    float q = this->q_of(d / N_TS, u), ts = LAG[d % N_TS];
    const int STEPS = 20;
    const float dts = 10.0f / STEPS;
    float a = q / this->tau_ * dts;
    float F[3][3] = {{1.0f - a, 0.0f, dts / this->tau_}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    float b[3] = {a, 0.0f, 0.0f};
    if (ts > 0.0f) {
      F[1][0] = dts / ts;
      F[1][1] = 1.0f - dts / ts;
    } else {
      for (int j = 0; j < 3; j++) F[1][j] = F[0][j];   // sensor = tent
      b[1] = b[0];
    }
    float M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, B[3] = {0, 0, 0};
    for (int s = 0; s < STEPS; s++) {
      float Mn[3][3], Bn[3];
      for (int i = 0; i < 3; i++) {
        Bn[i] = b[i];
        for (int j = 0; j < 3; j++) {
          Mn[i][j] = F[i][0] * M[0][j] + F[i][1] * M[1][j] + F[i][2] * M[2][j];
          Bn[i] += F[i][j] * B[j];
        }
      }
      for (int i = 0; i < 3; i++) {
        B[i] = Bn[i];
        for (int j = 0; j < 3; j++) M[i][j] = Mn[i][j];
      }
    }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) this->phi_[d][i * 3 + j] = M[i][j];
    this->b_[d][0] = B[0];
    this->b_[d][1] = B[1];
  }

  void decide_() {
    float best = -INFINITY, best_cfg = -INFINITY;
    for (int i = 0; i < N; i++) {
      best = std::max(best, this->score_[i]);
      // Reference: what the Kalman filter assumes - configured curve, no offset
      // (any lag, the lag hardly moves the sensible maximum)
      if ((i / N_V) / N_TS == this->cfg_curve_ && i % N_V == N_V / 2) best_cfg = std::max(best_cfg, this->score_[i]);
    }
    this->best_ = best;
    this->margin_ = best - best_cfg;
    if (!this->active_ && this->count_ >= COUNT_ON && this->margin_ >= MARGIN_ON && this->stable_()) this->active_ = true;
    else if (this->active_ && (this->count_ < COUNT_OFF || this->margin_ < MARGIN_OFF)) this->active_ = false;
    // Weights of the mix, and summed per curve
    float sum = 0.0f;
    for (int i = 0; i < N; i++) {
      this->w_[i] = expf(BETA * (this->score_[i] - best));
      sum += this->w_[i];
    }
    for (int c = 0; c < N_CURVE; c++) this->w_curve_[c] = 0.0f;
    for (int i = 0; i < N; i++) {
      this->w_[i] /= sum;
      this->w_curve_[(i / N_V) / N_TS] += this->w_[i];
    }
  }

  float tau_{21.0f}, q0_cfg_{0.2f};
  float q0_[N_Q0] = {0.1f, 0.2f, 0.3f, 0.4f};
  int cfg_curve_{5};
  float x_[N][3] = {};
  float p_[N][6] = {};
  float score_[N] = {};
  float w_[N] = {};
  float w_curve_[N_CURVE] = {};
  float phi_[N_DYN][9] = {};
  float b_[N_DYN][2] = {};
  float u_cache_{-1.0f}, u_prev_{NAN};
  float count_{0.0f}, margin_{0.0f}, best_{0.0f};
  int since_change_{1 << 20}, quiet_{0}, gap_{0};
  float block_[N] = {};
  int block_n_{0}, block_age_{0};
  float hist_[HIST][3] = {};
  int hist_n_{0}, hour_ticks_{0};
  bool block_clean_{true};
  bool started_{false}, reinit_{true}, light_pending_{false}, active_{false};
};

}  // namespace vpd_kalman
}  // namespace esphome
