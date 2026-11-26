#pragma once

#include <algorithm>
#include <cmath>

namespace maze_planner {

// ROS positive bearing is to the robot's left. The camera uses image x right
// and image y down; positive pitch points the optical axis down.
inline bool visualBearing(double pan, double tilt, double center_x, double center_y,
                          double cx, double cy, double fx, double fy,
                          double* bearing) {
  if (!bearing || !std::isfinite(pan) || !std::isfinite(tilt) ||
      !std::isfinite(center_x) || !std::isfinite(center_y) ||
      !std::isfinite(cx) || !std::isfinite(cy) ||
      !std::isfinite(fx) || !std::isfinite(fy) || fx <= 1.0 || fy <= 1.0)
    return false;
  const double image_x = (center_x - cx) / fx;
  const double image_y = (center_y - cy) / fy;
  const double forward = std::cos(tilt) - std::sin(tilt) * image_y;
  if (forward <= 0.0) return false;
  *bearing = std::atan2(std::sin(pan + std::atan2(-image_x, forward)),
                        std::cos(pan + std::atan2(-image_x, forward)));
  return true;
}

inline bool plausibleTargetHeight(double camera_z, double slant_range, double down_angle,
                                  double min_z, double max_z) {
  const double z = camera_z - slant_range * std::sin(down_angle);
  return std::isfinite(z) && z >= min_z && z <= max_z;
}

inline bool visualStopReached(bool detected, double contour_ratio, double bearing,
                              double required_ratio, double facing_tolerance) {
  // ROS observations carry the area in float32; allow its representation
  // error so an exact configured threshold is not rejected as 0.15999999.
  return detected && std::isfinite(contour_ratio) && std::isfinite(bearing) &&
         contour_ratio + 1e-7 >= required_ratio && std::fabs(bearing) <= facing_tolerance;
}

// Keep the goal on the sight line from the current base to the target so a
// forward-only planner steers the car toward the target as it approaches.
inline bool visualApproachGoal(double robot_x, double robot_y, double target_x,
                               double target_y, double standoff, double* goal_x,
                               double* goal_y, double* yaw) {
  if (!goal_x || !goal_y || !yaw || !std::isfinite(robot_x) || !std::isfinite(robot_y) ||
      !std::isfinite(target_x) || !std::isfinite(target_y) || !std::isfinite(standoff) ||
      standoff < 0.0) return false;
  const double dx = target_x - robot_x;
  const double dy = target_y - robot_y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) return false;
  *yaw = std::atan2(dy, dx);
  const double advance = std::max(0.0, distance - standoff);
  *goal_x = robot_x + advance * dx / distance;
  *goal_y = robot_y + advance * dy / distance;
  return true;
}

}  // namespace maze_planner
