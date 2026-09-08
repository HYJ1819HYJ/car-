#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace stm32_bridge {

struct SafetyPoint {
  double x{0.0};
  double y{0.0};
};

struct SafetyFootprint {
  double front{0.3483};
  double rear{-0.1517};
  double half_width{0.24};
  double padding{0.03};
};

inline SafetyPoint integrateTwist(const double v, const double w,
                                  const double time, double* yaw) {
  *yaw = w * time;
  if (std::fabs(w) < 1e-6) {
    return SafetyPoint{v * time, 0.0};
  }
  return SafetyPoint{(v / w) * std::sin(*yaw),
                     (v / w) * (1.0 - std::cos(*yaw))};
}

inline bool pointInsideFootprint(const SafetyPoint& point,
                                 const SafetyPoint& robot_position,
                                 const double robot_yaw,
                                 const SafetyFootprint& footprint) {
  const double dx = point.x - robot_position.x;
  const double dy = point.y - robot_position.y;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  // Transform the fixed obstacle from the current base frame into the
  // predicted vehicle frame.
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  return local_x >= footprint.rear - footprint.padding &&
         local_x <= footprint.front + footprint.padding &&
         std::fabs(local_y) <= footprint.half_width + footprint.padding;
}

// Check the complete asymmetric rectangular vehicle along a constant-twist
// trajectory.  This is intentionally independent of ROS so it can be tested.
inline bool trajectoryIntersects(const std::vector<SafetyPoint>& obstacles,
                                 const SafetyFootprint& footprint,
                                 const double v, const double w,
                                 const double horizon,
                                 const double requested_step) {
  if (obstacles.empty()) return false;
  const double duration = std::max(0.0, horizon);
  const double step = std::max(0.01, requested_step);
  const int samples = std::max(1, static_cast<int>(std::ceil(duration / step)));
  for (int sample = 0; sample <= samples; ++sample) {
    const double time = duration * static_cast<double>(sample) /
                        static_cast<double>(samples);
    double yaw = 0.0;
    const SafetyPoint pose = integrateTwist(v, w, time, &yaw);
    for (const SafetyPoint& obstacle : obstacles) {
      if (pointInsideFootprint(obstacle, pose, yaw, footprint)) return true;
    }
  }
  return false;
}

}  // namespace stm32_bridge
