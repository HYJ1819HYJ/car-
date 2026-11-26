#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <string>

namespace stm32_bridge {

struct GimbalStatus {
  bool enabled, ready, locked, searching;
  double pan_deg, tilt_deg;
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

  Action next(double now, bool ready, bool auto_enable, bool enabled, double tilt_deg) {
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
    if (std::fabs(tilt_deg) > 3.0) {
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

}  // namespace stm32_bridge
