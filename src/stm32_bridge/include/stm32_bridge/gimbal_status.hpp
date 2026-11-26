#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace stm32_bridge {

inline double applyYawFeedforwardPx(double center_x, double fx, double yaw_rate,
                                    double prediction_sec, double max_px, unsigned width) {
  if (!width) return 0.0;
  const double offset = std::max(-max_px, std::min(max_px, fx * yaw_rate * prediction_sec));
  return std::max(0.0, std::min(static_cast<double>(width - 1), center_x + offset));
}

struct GimbalStatus {
  bool enabled, ready, locked, searching;
  double pan_deg, tilt_deg;
};

struct GimbalTiltConfig {
  double min_tilt_deg{-25.0};
  double max_tilt_deg{0.0};
  double default_tilt_deg{-10.0};
  double relevel_tolerance_deg{1.5};
  double relevel_trigger_deg{2.5};
  double relevel_delay_sec{0.8};
  double min_target_down_deg{-4.0};
};

struct GimbalTargetCommand {
  bool found{false};
  uint16_t x{0xFFFF};
  uint16_t y{0xFFFF};
};

inline bool printableGimbalLine(const std::string& line) {
  return std::all_of(line.begin(), line.end(),
                     [](unsigned char ch) { return ch >= 0x20 && ch <= 0x7e; });
}

class GimbalStatusParser {
 public:
  explicit GimbalStatusParser(bool require_checksum = false)
      : require_checksum_(require_checksum) {}

  bool parse(const std::string& line, GimbalStatus* status) {
    if (!printableGimbalLine(line) || line.compare(0, 5, "STAT,") != 0 ||
        line.find("STAT,", 5) != std::string::npos) return false;
    const auto star = line.find('*');
    if (star == std::string::npos) {
      if (require_checksum_) return false;
    } else {
      if (line.size() - star != 3) return false;
      const int high = hexDigit(line[star + 1]), low = hexDigit(line[star + 2]);
      if (high < 0 || low < 0) return false;
      unsigned char checksum = 0;
      for (std::size_t i = 0; i < star; ++i) checksum ^= line[i];
      if (checksum != (high * 16 + low)) return false;
    }

    const std::string payload = line.substr(0, star);
    long mode, ready, lock, search, bearing, tilt;
    if (!integerField(payload, "mode", 0, 1, &mode) ||
        !integerField(payload, "ready", 0, 1, &ready) ||
        !integerField(payload, "lock", 0, 1, &lock) ||
        !integerField(payload, "search", 0, 1, &search) ||
        !integerField(payload, "bearing10", -1800, 1800, &bearing) ||
        !integerField(payload, "tilt10", -900, 900, &tilt)) return false;
    *status = {mode == 1, ready == 1, lock == 1, search == 1, bearing / 10.0, tilt / 10.0};
    if (star != std::string::npos) require_checksum_ = true;
    return true;
  }

 private:
  static int hexDigit(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    return -1;
  }

  static bool integerField(const std::string& line, const char* key,
                           long min, long max, long* value) {
    const std::string token = std::string(",") + key + "=";
    const auto begin = line.find(token);
    if (begin == std::string::npos || line.find(token, begin + 1) != std::string::npos)
      return false;
    const char* text = line.c_str() + begin + token.size();
    const char* digit = (*text == '-' || *text == '+') ? text + 1 : text;
    if (*digit < '0' || *digit > '9') return false;
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || (*end != ',' && *end != '\0') || errno == ERANGE ||
        parsed < min || parsed > max) return false;
    *value = parsed;
    return true;
  }

  bool require_checksum_;
};

class GimbalStartup {
 public:
  enum class Action { None, Center, Enable };

  void reset(bool center_on_connect) {
    center_requested_ = center_on_connect;
    center_time_ = -1.0;
    last_command_ = -1.5;
  }

  void requestCenter() { center_requested_ = true; }

  Action next(double now, bool ready, bool auto_enable, bool enabled,
              double tilt_deg, double target_tilt_deg = 0.0) {
    if (!ready) return Action::None;
    if (center_requested_) {
      center_requested_ = false;
      center_time_ = last_command_ = now;
      return Action::Center;
    }
    if (center_time_ >= 0.0) {
      if (now - center_time_ < 0.8) return Action::None;
      center_time_ = -1.0;
      if (!auto_enable) return Action::None;
      last_command_ = now;
      return Action::Enable;
    }
    if (!auto_enable || enabled || now - last_command_ < 1.5) return Action::None;
    last_command_ = now;
    if (std::fabs(tilt_deg - target_tilt_deg) > 3.0) {
      center_time_ = now;
      return Action::Center;
    }
    return Action::Enable;
  }

 private:
  bool center_requested_{false};
  double center_time_{-1.0};
  double last_command_{-1.5};
};

// Pure ROS-side tilt limiter and default-pitch leveling controller compatible
// with the unmodified STM32 gimbal firmware (TILT_DIRECTION = -1.0, 40 ms
// step period, 12 px dead zone, ~2.87 Hz STAT feedback).
class GimbalTiltController {
 public:
  explicit GimbalTiltController(const GimbalTiltConfig& config = {}) {
    configure(config);
  }

  void configure(const GimbalTiltConfig& config) {
    config_ = config;
    if (config_.min_tilt_deg > config_.max_tilt_deg) {
      std::swap(config_.min_tilt_deg, config_.max_tilt_deg);
    }
    config_.default_tilt_deg =
        clamp(config_.default_tilt_deg, config_.min_tilt_deg, config_.max_tilt_deg);
    config_.relevel_tolerance_deg = std::max(0.5, config_.relevel_tolerance_deg);
    config_.relevel_trigger_deg =
        std::max(config_.relevel_tolerance_deg, config_.relevel_trigger_deg);
    config_.relevel_delay_sec = std::max(0.0, config_.relevel_delay_sec);
  }

  const GimbalTiltConfig& config() const { return config_; }

  void reset() {
    phase_ = Phase::Idle;
    level_requested_ = false;
    pulses_in_cycle_ = 0;
    cooldown_until_ = -1.0;
    last_valid_target_time_ = -1e9;
    has_estimate_ = false;
    last_cmd_ = {};
    last_compute_time_ = -1.0;
  }

  void requestLevel() {
    level_requested_ = true;
    phase_ = Phase::Idle;
    pulses_in_cycle_ = 0;
    cooldown_until_ = -1.0;
  }

  void onStatus(double tilt_deg) {
    last_tilt_deg_ = tilt_deg;
    estimated_tilt_deg_ = tilt_deg;
    has_estimate_ = true;
    ++status_seq_;
  }

  bool leveling() const { return phase_ != Phase::Idle || level_requested_; }

  GimbalTargetCommand compute(double now, bool enabled, bool ready, double tilt_deg,
                              bool has_observation, double scaled_x, double scaled_y,
                              double target_down_deg, bool autonomy_enabled) {
    if (!enabled || !ready || !std::isfinite(tilt_deg)) {
      phase_ = Phase::Idle;
      pulses_in_cycle_ = 0;
      last_cmd_ = {};
      last_compute_time_ = now;
      has_estimate_ = false;
      return {};
    }

    advanceEstimate(now, tilt_deg);

    const bool height_ok = !std::isfinite(target_down_deg) ||
                           target_down_deg >= config_.min_target_down_deg;
    const bool valid_target = has_observation && height_ok &&
                              std::isfinite(scaled_x) && std::isfinite(scaled_y);

    // During standby, finish an explicit center/init pitch leveling first so
    // pre-race tilt checks converge deterministically even with nearby clutter.
    const bool prioritize_standby_level = !autonomy_enabled && level_requested_;

    if (valid_target && !prioritize_standby_level) {
      phase_ = Phase::Idle;
      pulses_in_cycle_ = 0;
      level_requested_ = false;
      last_valid_target_time_ = now;
      return recordCommand(now, limitTrackedTarget(tilt_deg, scaled_x, scaled_y));
    }

    const double err_deg = tilt_deg - config_.default_tilt_deg;
    const bool out_of_bounds =
        tilt_deg > config_.max_tilt_deg + 0.5 || tilt_deg < config_.min_tilt_deg - 0.5;

    if (phase_ == Phase::Pulse) {
      if (now < pulse_end_time_) {
        return recordCommand(now, {true, kCenterX, pulse_y_});
      }
      phase_ = Phase::Settle;
      settle_start_time_ = now;
      settle_start_seq_ = status_seq_;
      return recordCommand(now, {true, kCenterX, kCenterY});
    }

    if (phase_ == Phase::Settle) {
      const bool got_fresh_status = status_seq_ > settle_start_seq_;
      const bool timed_out = (now - settle_start_time_) >= kSettleTimeoutSec;
      if (!got_fresh_status && !timed_out) {
        return recordCommand(now, {true, kCenterX, kCenterY});
      }
      const double settled_err = tilt_deg - config_.default_tilt_deg;
      const bool settled_out_of_bounds =
          tilt_deg > config_.max_tilt_deg + 0.5 || tilt_deg < config_.min_tilt_deg - 0.5;
      if (!settled_out_of_bounds && std::fabs(settled_err) <= config_.relevel_tolerance_deg) {
        phase_ = Phase::Idle;
        level_requested_ = false;
        pulses_in_cycle_ = 0;
        return recordCommand(now, {});
      }
      if (pulses_in_cycle_ >= kMaxPulsesPerCycle) {
        phase_ = Phase::Idle;
        level_requested_ = false;
        pulses_in_cycle_ = 0;
        cooldown_until_ = now + kCooldownSec;
        return recordCommand(now, {});
      }
      return recordCommand(now, startPulse(now, settled_err));
    }

    // Idle phase: decide whether to drive tilt back to default_tilt_deg.
    if (now < cooldown_until_ && !level_requested_ && !out_of_bounds) {
      return recordCommand(now, {});
    }

    if (level_requested_) {
      if (!out_of_bounds && std::fabs(err_deg) <= config_.relevel_tolerance_deg) {
        level_requested_ = false;
        return recordCommand(now, {});
      }
      pulses_in_cycle_ = 0;
      return recordCommand(now, startPulse(now, err_deg));
    }

    const bool lost_long_enough = (now - last_valid_target_time_) >= config_.relevel_delay_sec;
    if (out_of_bounds || (lost_long_enough && std::fabs(err_deg) > config_.relevel_trigger_deg)) {
      pulses_in_cycle_ = 0;
      return recordCommand(now, startPulse(now, err_deg));
    }

    return recordCommand(now, {});
  }

 private:
  enum class Phase { Idle, Pulse, Settle };

  static constexpr uint16_t kCenterX = 320;
  static constexpr uint16_t kCenterY = 240;
  static constexpr double kFirmwareKpY = 0.012;
  static constexpr double kFirmwareMaxStepDeg = 1.5;
  static constexpr double kFirmwareStepPeriodSec = 0.04;
  static constexpr double kFirmwareDeadZonePx = 12.0;
  static constexpr double kSettleTimeoutSec = 0.75;
  static constexpr int kMaxPulsesPerCycle = 6;
  static constexpr double kCooldownSec = 2.0;

  static double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
  }

  void advanceEstimate(double now, double reported_tilt_deg) {
    if (!has_estimate_) {
      estimated_tilt_deg_ = reported_tilt_deg;
      has_estimate_ = true;
      last_compute_time_ = now;
      return;
    }
    const double dt = clamp(now - last_compute_time_, 0.0, 0.5);
    if (dt > 0.0 && last_cmd_.found) {
      const double err_y = static_cast<double>(last_cmd_.y) - static_cast<double>(kCenterY);
      if (std::fabs(err_y) > kFirmwareDeadZonePx) {
        const double step_deg = clamp(-kFirmwareKpY * err_y, -kFirmwareMaxStepDeg,
                                      kFirmwareMaxStepDeg);
        estimated_tilt_deg_ =
            clamp(estimated_tilt_deg_ + step_deg * (dt / kFirmwareStepPeriodSec), -75.0, 75.0);
      }
    }
  }

  GimbalTargetCommand recordCommand(double now, const GimbalTargetCommand& cmd) {
    last_cmd_ = cmd;
    last_compute_time_ = now;
    return cmd;
  }

  GimbalTargetCommand startPulse(double now, double err_deg) {
    const double abs_err = std::fabs(err_deg);
    double offset_px = 20.0;
    if (abs_err >= 6.0) offset_px = 60.0;
    else if (abs_err >= 3.0) offset_px = 35.0;
    const double rate_deg_per_sec = (offset_px * kFirmwareKpY) / kFirmwareStepPeriodSec;
    const double duration = clamp(0.80 * abs_err / rate_deg_per_sec, 0.06, 0.42);
    // err_deg > 0 means current tilt is too high (needs pitching down -> y > 240).
    const double signed_y = err_deg > 0.0 ? (kCenterY + offset_px) : (kCenterY - offset_px);
    pulse_y_ = static_cast<uint16_t>(std::lround(clamp(signed_y, 0.0, 479.0)));
    pulse_end_time_ = now + duration;
    phase_ = Phase::Pulse;
    ++pulses_in_cycle_;
    return {true, kCenterX, pulse_y_};
  }

  GimbalTargetCommand limitTrackedTarget(double tilt_deg, double scaled_x, double scaled_y) const {
    double out_y = scaled_y;
    const double conservative_high = std::max(tilt_deg, estimated_tilt_deg_);
    const double conservative_low = std::min(tilt_deg, estimated_tilt_deg_);
    const double margin_up = config_.max_tilt_deg - conservative_high;
    const double margin_down = conservative_low - config_.min_tilt_deg;

    // Account for worst-case ~0.55 s STAT lag (~14 MCU 40 ms steps) so the
    // firmware cannot overshoot max_tilt_deg or min_tilt_deg between STAT frames.
    constexpr double kSafetySteps = 14.0;
    constexpr double kStepsGain = kSafetySteps * kFirmwareKpY;

    if (tilt_deg > config_.max_tilt_deg + 0.5) {
      // Already above upper bound (pitched up): force downward correction.
      const double corr_px = clamp((tilt_deg - config_.max_tilt_deg) * 10.0, 25.0, 60.0);
      out_y = kCenterY + corr_px;
    } else if (tilt_deg < config_.min_tilt_deg - 0.5) {
      // Already below lower bound: force upward correction back into range.
      const double corr_px = clamp((config_.min_tilt_deg - tilt_deg) * 10.0, 25.0, 60.0);
      out_y = kCenterY - corr_px;
    } else if (out_y < kCenterY) {
      // Target requests pitching up (increasing tilt_deg toward max_tilt_deg).
      const double max_up_px = (margin_up - 0.5) / kStepsGain;
      if (max_up_px <= kFirmwareDeadZonePx) {
        out_y = kCenterY;
      } else {
        out_y = std::max(out_y, kCenterY - max_up_px);
      }
    } else if (out_y > kCenterY) {
      // Target requests pitching down (decreasing tilt_deg toward min_tilt_deg).
      const double max_down_px = (margin_down - 0.5) / kStepsGain;
      if (max_down_px <= kFirmwareDeadZonePx) {
        out_y = kCenterY;
      } else {
        out_y = std::min(out_y, kCenterY + max_down_px);
      }
    }

    const uint16_t x = static_cast<uint16_t>(std::lround(clamp(scaled_x, 0.0, 639.0)));
    const uint16_t y = static_cast<uint16_t>(std::lround(clamp(out_y, 0.0, 479.0)));
    return {true, x, y};
  }

  GimbalTiltConfig config_;
  Phase phase_{Phase::Idle};
  bool level_requested_{false};
  uint16_t pulse_y_{kCenterY};
  double pulse_end_time_{0.0};
  double settle_start_time_{0.0};
  std::uint64_t settle_start_seq_{0};
  std::uint64_t status_seq_{0};
  double last_tilt_deg_{0.0};
  double estimated_tilt_deg_{0.0};
  bool has_estimate_{false};
  GimbalTargetCommand last_cmd_;
  double last_compute_time_{-1.0};
  int pulses_in_cycle_{0};
  double cooldown_until_{-1.0};
  double last_valid_target_time_{-1e9};
};

}  // namespace stm32_bridge
