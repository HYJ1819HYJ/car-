#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <maze_planner/visual_tracking.h>
#include <maze_planner/wall_parking.h>
#include <stm32_bridge/safety_geometry.hpp>

namespace maze_planner {

struct ReactiveParameters {
  double max_speed{0.10};
  double min_approach_speed{0.10};
  double max_turn{0.25};
  double turn_gain{1.0};
  double turn_before_drive{0.44};
  double stop_ratio{0.16};
  double stop_bearing{5.0 * M_PI / 180.0};
  double avoid_distance{0.65};
  double target_distance_margin{0.25};
  double lookahead_time{2.0};
  double metric_stop_depth_min{.48};
  double metric_stop_depth_max{.63};
  bool terminal_only{false};
  double wall_align_enable_distance{0.65};
  double wall_align_lateral_deadband{0.10};
};

struct ReactiveInput {
  bool target_visible{false};
  double contour_ratio{0.0};
  double bearing{0.0};
  double target_front_distance{std::numeric_limits<double>::infinity()};
  double front_clearance{std::numeric_limits<double>::infinity()};
  bool wall_valid{false};
  bool permit_visual_stop{true};
  Point2 wall_marker;
  Point2 wall_outward{1.0, 0.0};
  std::vector<stm32_bridge::SafetyPoint> obstacles;
  stm32_bridge::SafetyFootprint footprint;
};

struct ReactiveCommand {
  double v{0.0};
  double w{0.0};
  bool complete{false};
  int avoid_side{0};  // ROS left is positive.
  const char* state{"WAIT"};
};

inline double clipped(double value, double limit) {
  return std::max(-limit, std::min(limit, value));
}

inline bool reactivePathClear(const ReactiveInput& input, double v, double w,
                              double duration) {
  return !stm32_bridge::trajectoryIntersects(input.obstacles, input.footprint,
                                               v, w, duration, 0.10);
}

inline double reactiveApproachSpeed(const ReactiveInput& input,
                                    const ReactiveParameters& p) {
  const double low = std::min(p.max_speed, std::max(0.05, p.min_approach_speed));
  if (p.max_speed <= low + 1e-6) return p.max_speed;

  double area_scale = 1.0;
  if (p.stop_ratio > 1e-6) {
    const double start_ratio = 0.45 * p.stop_ratio;
    if (input.contour_ratio > start_ratio) {
      area_scale = std::max(
          0.0, std::min(1.0, (p.stop_ratio - input.contour_ratio) /
                                 (p.stop_ratio - start_ratio)));
    }
  }
  double dist_scale = 1.0;
  if (std::isfinite(input.front_clearance) && input.front_clearance < 0.55) {
    dist_scale = std::max(0.0, std::min(1.0, (input.front_clearance - 0.20) / 0.35));
  }
  return low + (p.max_speed - low) * std::min(area_scale, dist_scale);
}

inline bool parkingStopReached(const ReactiveInput& input, const ReactiveParameters& p) {
  if (!input.wall_valid) {
    return input.permit_visual_stop && visualStopReached(input.target_visible, input.contour_ratio,
        input.bearing, p.stop_ratio, p.stop_bearing);
  }
  const WallObservation wall{input.wall_marker, input.wall_outward};
  const double depth = -dot(wall.marker, wall.outward);
  const double lateral = std::abs(dot(wall.marker, {-wall.outward.y, wall.outward.x}));
  if (depth > .70 || std::abs(parkingYaw(wall)) > 8.0 * M_PI / 180.0) return false;
  std::vector<Point2> footprint;
  for (const auto& vertex : input.footprint.vertices) footprint.emplace_back(vertex.x, vertex.y);
  if (footprint.size() < 3) {
    const auto& f = input.footprint;
    footprint = {{f.front, f.half_width}, {f.front, -f.half_width},
                 {f.rear, -f.half_width}, {f.rear, f.half_width}};
  }
  if (depth >= p.metric_stop_depth_min && depth <= p.metric_stop_depth_max &&
      lateral <= .14 && footprintInsideFinish(wall, {}, 0.0, footprint, .80, .02)) return true;
  // The relaxed measured-depth path still checks all vertices at actual yaw.
  return depth >= std::min(.47, p.metric_stop_depth_min) && depth <= p.metric_stop_depth_max &&
         lateral <= .14 && std::abs(input.bearing) <= M_PI / 30.0 &&
         footprintInsideFinish(wall, {}, 0.0, footprint);
}

inline double wallAlignedSteeringBearing(const ReactiveInput& input,
                                         const ReactiveParameters& p) {
  if (!input.wall_valid) {
    return input.bearing;
  }
  const double normal_dist =
      dot(Point2(-input.wall_marker.x, -input.wall_marker.y), input.wall_outward);
  const Point2 tangent(-input.wall_outward.y, input.wall_outward.x);
  const double lat_offset = std::fabs(dot(input.wall_marker, tangent));
  if (normal_dist <= p.wall_align_enable_distance ||
      lat_offset <= p.wall_align_lateral_deadband) {
    return input.bearing;
  }
  const double blend =
      std::min(1.0, (lat_offset - p.wall_align_lateral_deadband) / 0.25);
  const double lead = blend * std::min(0.80, std::max(0.55, 0.55 * normal_dist));
  const Point2 aim = input.wall_marker + lead * input.wall_outward;
  return norm(aim) > 0.10 ? std::atan2(aim.y, aim.x) : input.bearing;
}

// This is local, scan-based steering: it has no map, localization or global
// planner. The serial bridge continues to apply its existing collision checks.
inline ReactiveCommand reactiveVisualCommand(const ReactiveInput& input,
                                             const ReactiveParameters& p,
                                             int previous_avoid_side,
                                             int search_side) {
  ReactiveCommand result;
  if (!input.target_visible && !input.wall_valid) {
    result.state = "SEARCH";
    if (p.terminal_only) return result;
    const int side = search_side < 0 ? -1 : 1;
    if (reactivePathClear(input, 0.0, side*p.max_turn, 1.0)) {
      result.w = side*p.max_turn;
    } else if (reactivePathClear(input, 0.0, -side*p.max_turn, 1.0)) {
      result.w = -side*p.max_turn;
    } else {
      result.state = "BLOCKED";
    }
    return result;
  }

  if (parkingStopReached(input, p)) {
    result.complete = true;
    result.state = "COMPLETE";
    return result;
  }

  const double steer_bearing = wallAlignedSteeringBearing(input, p);

  // At the requested area, finish turning before accepting the stop; also
  // turn first when the paper (or centerline lead point) is far off heading.
  const bool visual_align = !input.wall_valid && input.permit_visual_stop &&
                            input.contour_ratio + 1e-7 >= p.stop_ratio;
  const double wall_depth = -dot(input.wall_marker, input.wall_outward);
  const double normal_yaw = parkingYaw({input.wall_marker, input.wall_outward});
  const bool final_yaw_align = input.wall_valid && wall_depth <= .65 &&
      std::abs(normal_yaw) > 8.0 * M_PI / 180.0;
  if (visual_align || final_yaw_align || std::fabs(steer_bearing) >= p.turn_before_drive) {
    result.state = "ALIGN";
    const double align_target =
        final_yaw_align ? normal_yaw : (visual_align ? input.bearing : steer_bearing);
    const double turn = clipped(p.turn_gain*align_target, p.max_turn);
    if (reactivePathClear(input, 0.0, turn, 1.0)) result.w = turn;
    else result.state = "BLOCKED";
    return result;
  }

  double drive_speed = reactiveApproachSpeed(input, p);
  if (input.wall_valid && wall_depth <= p.metric_stop_depth_min) {
    result.state = "BLOCKED";
    return result;
  }
  if (input.wall_valid)
    drive_speed = std::min(drive_speed, std::max(.04, (wall_depth - p.metric_stop_depth_min) * .8));

  // A nearby return is a bypass obstacle when it is clearly in front of the
  // estimated paper plane. The paper's own wall remains the approach target.
  const bool obstacle = input.front_clearance < p.avoid_distance &&
      input.front_clearance + p.target_distance_margin < input.target_front_distance;
  if (obstacle) {
    if (p.terminal_only) {
      result.state = "BLOCKED";
      return result;
    }
    double left_near = std::numeric_limits<double>::infinity();
    double right_near = std::numeric_limits<double>::infinity();
    for (const auto& point : input.obstacles) {
      if (point.x < 0.0 || point.x > 1.2 || std::fabs(point.y) > 1.0) continue;
      const double distance = std::hypot(point.x, point.y);
      if (point.y >= 0.0) left_near = std::min(left_near, distance);
      else right_near = std::min(right_near, distance);
    }
    int preferred = previous_avoid_side;
    if (preferred == 0) preferred = left_near > right_near ? 1 : -1;
    const int sides[] = {preferred, -preferred};
    for (int side : sides) {
      const double turn = side*p.max_turn;
      if (reactivePathClear(input, drive_speed, turn, p.lookahead_time)) {
        result.v = drive_speed;
        result.w = turn;
        result.avoid_side = side;
        result.state = "AVOID";
        return result;
      }
    }
    for (int side : sides) {
      const double turn = side*p.max_turn;
      if (reactivePathClear(input, 0.0, turn, 1.0)) {
        result.w = turn;
        result.avoid_side = side;
        result.state = "AVOID";
        return result;
      }
    }
    result.state = "BLOCKED";
    return result;
  }

  result.v = drive_speed;
  result.w = clipped(p.turn_gain*steer_bearing, p.max_turn);
  result.state = "APPROACH";
  if (p.terminal_only && !reactivePathClear(input, result.v, result.w, .60)) {
    result.v = result.w = 0.0;
    result.state = "BLOCKED";
  }
  return result;
}

}  // namespace maze_planner
