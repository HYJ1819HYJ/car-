#pragma once

#include <sensor_msgs/LaserScan.h>
#include <stm32_bridge/safety_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace maze_planner {

inline void approachVelocityLimits(double target_distance,
                                   double slowdown_distance, double full_slow_distance,
                                   double cruise_v, double cruise_w,
                                   double approach_v, double approach_w,
                                   double* max_v, double* max_w) {
  *max_v = approach_v;
  *max_w = approach_w;
  if (!std::isfinite(target_distance) || !std::isfinite(slowdown_distance) ||
      !std::isfinite(full_slow_distance) || slowdown_distance <= 0.0 ||
      slowdown_distance <= full_slow_distance ||
      target_distance <= std::max(0.0, full_slow_distance)) return;
  const double alpha = std::min(1.0, (target_distance - full_slow_distance) /
                                     (slowdown_distance - full_slow_distance));
  *max_v += alpha * std::max(0.0, cruise_v - approach_v);
  *max_w += alpha * std::max(0.0, cruise_w - approach_w);
}

struct HandoffCorridorParameters {
  double laser_x{0.315};
  double laser_y{0.0};
  double laser_yaw{0.0};
  // Endpoint used only when checking the corridor, not a stopping distance.
  double check_standoff{0.48};
  double sample_step{0.10};
  double ray_margin{0.04};
  double turn_rate{0.25};
};

// Test the initial in-place turn and the straight swept footprint. Ray samples
// check that the visible part of the corridor was actually observed: NaN,
// infinity and self-occluded rays are not treated as free space.
inline bool handoffCorridorClear(const sensor_msgs::LaserScan& scan,
                                 const std::vector<stm32_bridge::SafetyPoint>& obstacles,
                                 const stm32_bridge::SafetyFootprint& footprint,
                                 double marker_x, double marker_y,
                                 const HandoffCorridorParameters& p,
                                 std::string* reason) {
  if (!reason) return false;
  const double target_distance = std::hypot(marker_x, marker_y);
  if (!std::isfinite(target_distance) || target_distance < .05 ||
      scan.ranges.empty() || scan.angle_increment <= 0.0 ||
      p.sample_step <= 0.0 || p.turn_rate <= 0.0 || p.check_standoff < 0.0) {
    *reason = "invalid geometry or scan";
    return false;
  }
  const double heading = std::atan2(marker_y, marker_x);
  const double turn = heading >= 0.0 ? p.turn_rate : -p.turn_rate;
  if (std::fabs(heading) > 1e-3 &&
      stm32_bridge::trajectoryIntersects(obstacles, footprint, 0.0, turn,
                                         std::fabs(heading)/p.turn_rate, 0.10)) {
    *reason = "initial turn intersects a lidar return";
    return false;
  }
  const double c = std::cos(heading), s = std::sin(heading);
  const double advance = std::max(0.0, target_distance-p.check_standoff);
  const int path_steps = std::max(1, static_cast<int>(std::ceil(advance/p.sample_step)));
  for (int i = 0; i <= path_steps; ++i) {
    const double progress = advance*static_cast<double>(i)/path_steps;
    const stm32_bridge::SafetyPoint pose{c*progress, s*progress};
    for (const auto& point : obstacles) {
      if (stm32_bridge::pointInsideFootprint(point, pose, heading, footprint)) {
        *reason = "lidar return in the vehicle corridor";
        return false;
      }
    }
  }

  stm32_bridge::SafetyFootprint current_body = footprint;
  current_body.padding = 0.0;
  const double corridor_end = advance + footprint.front + footprint.padding;
  const double half_width = footprint.half_width + footprint.padding;
  const int longitudinal_steps = std::max(1, static_cast<int>(std::ceil(corridor_end/p.sample_step)));
  const int lateral_steps = std::max(2, static_cast<int>(std::ceil(2.0*half_width/p.sample_step)));
  for (int i = 0; i <= longitudinal_steps; ++i) {
    const double forward = corridor_end*static_cast<double>(i)/longitudinal_steps;
    for (int j = 0; j <= lateral_steps; ++j) {
      const double side = -half_width+2.0*half_width*static_cast<double>(j)/lateral_steps;
      const stm32_bridge::SafetyPoint point{c*forward-s*side, s*forward+c*side};
      if (stm32_bridge::pointInsideFootprint(point, {0.0,0.0}, 0.0, current_body))
        continue;
      // The enclosing rectangle includes space beside the tapered nose that
      // the vehicle never sweeps. Do not require that irrelevant space to be
      // clear: it may contain the wall beside the intended final position.
      // Unknown rays *within* the swept, padded footprint still block handoff.
      bool in_swept_footprint = false;
      for (int k = 0; k <= path_steps; ++k) {
        const double progress = advance*static_cast<double>(k)/path_steps;
        if (stm32_bridge::pointInsideFootprint(
                point, {c*progress, s*progress}, heading, footprint)) {
          in_swept_footprint = true;
          break;
        }
      }
      if (!in_swept_footprint) continue;
      const double dx = point.x-p.laser_x, dy = point.y-p.laser_y;
      // The scanner cannot observe cells behind its own origin or inside its
      // near-range blind spot; these are not claimed as confirmed clear.
      if (dx*c+dy*s <= 0.0 || std::hypot(dx,dy) <= scan.range_min+.02) continue;
      const double distance = std::hypot(dx,dy);
      const double angle = std::atan2(std::sin(std::atan2(dy,dx)-p.laser_yaw),
                                      std::cos(std::atan2(dy,dx)-p.laser_yaw));
      const int beam = static_cast<int>(std::lround((angle-scan.angle_min)/scan.angle_increment));
      if (beam <= 0 || beam+1 >= static_cast<int>(scan.ranges.size())) {
        *reason = "corridor outside lidar field of view";
        return false;
      }
      // The sensor occasionally leaves individual rays invalid. A sample is
      // supported by at least two finite rays passing within a quarter grid
      // step of it; invalid rays themselves never count as free observations.
      const int neighbors = std::min(50, std::max(1, static_cast<int>(std::ceil(
          std::atan2(.25*p.sample_step, distance)/scan.angle_increment))));
      int observed_rays = 0;
      for (int index = std::max(0, beam-neighbors);
           index <= std::min(static_cast<int>(scan.ranges.size())-1, beam+neighbors); ++index) {
        const double range = scan.ranges[index];
        if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max) continue;
        ++observed_rays;
        if (range <= distance+p.ray_margin) {
          *reason = "corridor not observed clear by lidar";
          return false;
        }
      }
      if (observed_rays < 2) {
        *reason = "corridor not observed clear by lidar";
        return false;
      }
    }
  }
  *reason = "observed corridor clear";
  return true;
}

}  // namespace maze_planner
