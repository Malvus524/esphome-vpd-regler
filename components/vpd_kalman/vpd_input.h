#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace esphome {
namespace vpd_kalman {

// Broad physical limits, not the configurable temperature protection threshold.
inline bool valid_temperature(float value) { return std::isfinite(value) && value >= -40.0f && value <= 85.0f; }
inline bool valid_humidity(float value) { return std::isfinite(value) && value >= 0.0f && value <= 100.0f; }
inline bool valid_climate(float temperature, float humidity) {
  return valid_temperature(temperature) && valid_humidity(humidity);
}

// Tracks publications, not value changes: a stable climate is still fresh.
class SampleClock {
 public:
  void received(uint32_t now_ms) {
    this->received_ = true;
    this->last_ms_ = now_ms;
  }
  bool fresh(uint32_t now_ms, uint32_t timeout_ms) {
    // Latch expiration so a long-dead source cannot revive on the next millis wrap.
    if (this->received_ && uint32_t(now_ms - this->last_ms_) >= timeout_ms)
      this->received_ = false;
    return this->received_;
  }

 protected:
  bool received_{false};
  uint32_t last_ms_{0};
};

inline float protection_output(float value, float threshold, float band, float hysteresis,
                               float emergency, bool &active) {
  if (!std::isfinite(value)) { active = false; return 0.0f; }
  if (value > threshold) active = true;
  else if (value < threshold - hysteresis) active = false;
  if (!active) return 0.0f;
  return std::min(100.0f, std::max(0.0f,
      emergency + (100.0f - emergency) * std::max(value - threshold, 0.0f) / band));
}

}  // namespace vpd_kalman
}  // namespace esphome
