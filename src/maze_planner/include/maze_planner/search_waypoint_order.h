#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace maze_planner {

// Prefer a nearby waypoint within 45 degrees of the initial heading. Keep the
// nearest waypoint as a fallback when the map has no candidate in that cone.
template <typename Point>
std::size_t searchStartIndex(const std::vector<Point>& points, double x, double y,
                             double yaw, bool prefer_forward) {
  std::size_t nearest = 0, forward = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  double forward_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double dx = points[i].x - x, dy = points[i].y - y;
    const double distance = std::hypot(dx, dy);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
    const double projection = std::cos(yaw) * dx + std::sin(yaw) * dy;
    if (distance > 1e-6 && projection >= 0.7071067811865476 * distance &&
        distance < forward_distance) {
      forward_distance = distance;
      forward = i;
    }
  }
  return prefer_forward && std::isfinite(forward_distance) ? forward : nearest;
}

}  // namespace maze_planner
