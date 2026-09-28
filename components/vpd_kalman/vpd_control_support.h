#pragma once

#include <algorithm>
#include <cmath>

namespace esphome {
namespace vpd_kalman {

// Reject isolated jumps >0.1 kPa relative to a three-sample median. Small
// changes pass immediately, avoiding the dead time of a permanent median filter.
// A sustained large change is accepted after two samples.
// No heap allocation; reset whenever samples no longer describe the same climate.
class Median3 {
 public:
  void reset() { this->count_ = 0; this->index_ = 0; }
  float update(float sample) {
    if (this->count_ == 0) {
      this->values_[0] = this->values_[1] = this->values_[2] = sample;
      this->count_ = 1;
    }
    this->values_[this->index_] = sample;
    this->index_ = (this->index_ + 1) % 3;
    const float a = this->values_[0], b = this->values_[1], c = this->values_[2];
    const float median = std::max(std::min(a, b), std::min(std::max(a, b), c));
    return std::fabs(sample - median) > 0.1f ? median : sample;
  }

 protected:
  float values_[3]{};
  unsigned index_{0}, count_{0};
};

// Light low-pass filter and a short trend estimate. Trend can only reduce an
// upward controller move; it never commands a move or changes the actual target.
class VpdResponse {
 public:
  void reset() { this->value_ = NAN; this->trend_ = 0.0f; this->rising_ = 0; }
  float update(float sample, float dt_s, float smoothing_s) {
    if (!std::isfinite(this->value_)) {
      this->value_ = sample;
      return sample;
    }
    const float alpha = smoothing_s > 0.0f ? 1.0f - std::exp(-dt_s / smoothing_s) : 1.0f;
    const float previous = this->value_;
    this->value_ += alpha * (sample - this->value_);
    const float slope = (this->value_ - previous) / dt_s;
    // A reversal cancels anticipation immediately; require two rising samples.
    if (slope <= 0.0f) {
      this->trend_ = 0.0f;
      this->rising_ = 0;
    } else {
      this->trend_ += 0.25f * (slope - this->trend_);
      this->rising_ = std::min(this->rising_ + 1, 3U);
    }
    return this->value_;
  }
  float increase_factor(float lower_band_edge, float horizon_s) const {
    const float gap = lower_band_edge - this->value_;
    if (this->rising_ < 2 || gap <= 0.0f || horizon_s <= 0.0f) return 1.0f;
    const float expected_rise = this->trend_ * horizon_s;
    return std::max(0.0f, 1.0f - expected_rise / gap);
  }

 protected:
  float value_{NAN}, trend_{0.0f};
  unsigned rising_{0};
};

inline float limit_fan_increase(float previous, float requested, float factor, float rate_per_min, float dt_s) {
  if (requested <= previous) return requested;
  return previous + std::min((requested - previous) * factor, rate_per_min * dt_s / 60.0f);
}

// A decaying output floor provides a bumpless automatic return after protection.
// A new protection request raises the floor immediately, with no upward limit.
class ProtectionRelease {
 public:
  void reset() { this->floor_ = NAN; }
  bool above(float requested) const { return std::isfinite(this->floor_) && this->floor_ > requested + 0.01f; }
  float apply(float requested, float protection, bool enabled, float rate_per_min, float dt_s) {
    if (!enabled) {
      this->reset();
      return requested;
    }
    if (std::isfinite(this->floor_)) this->floor_ -= rate_per_min * dt_s / 60.0f;
    if (protection > 0.0f && protection >= requested - 0.01f)
      this->floor_ = std::isfinite(this->floor_) ? std::max(this->floor_, protection) : protection;
    if (!this->above(requested)) {
      // Keep a floor while protection is actually holding the output.
      if (protection <= 0.0f || protection < requested - 0.01f) this->reset();
      return requested;
    }
    return this->floor_;
  }

 protected:
  float floor_{NAN};
};

}  // namespace vpd_kalman
}  // namespace esphome
