#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <maze_planner/visual_tracking.h>
#include <stm32_bridge/safety_geometry.hpp>

namespace maze_planner {

struct ReactiveParameters {
  double max_speed{0.10};
  double max_turn{0.25};
  double turn_gain{1.0};
  double turn_before_drive{0.44};
  double stop_ratio{0.16};
  double stop_bearing{5.0 * M_PI / 180.0};
  double avoid_distance{0.65};
  double target_distance_margin{0.25};
  double lookahead_time{2.0};
};

struct ReactiveInput {
  bool target_visible{false};
  double contour_ratio{0.0};
  double bearing{0.0};
  double target_front_distance{std::numeric_limits<double>::infinity()};
  double front_clearance{std::numeric_limits<double>::infinity()};
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

// This is local, scan-based steering: it has no map, localization or global
// planner. The serial bridge continues to apply its existing collision checks.
inline ReactiveCommand reactiveVisualCommand(const ReactiveInput& input,
                                             const ReactiveParameters& p,
                                             int previous_avoid_side,
                                             int search_side) {
  ReactiveCommand result;
  if (!input.target_visible) {
    result.state = "SEARCH";
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

  if (visualStopReached(true, input.contour_ratio, input.bearing,
                        p.stop_ratio, p.stop_bearing)) {
    result.complete = true;
    result.state = "COMPLETE";
    return result;
  }

  // At the requested area, finish turning before accepting the stop; also
  // turn first when the paper is far off the current heading.
  if (input.contour_ratio + 1e-7 >= p.stop_ratio ||
      std::fabs(input.bearing) >= p.turn_before_drive) {
    result.state = "ALIGN";
    const double turn = clipped(p.turn_gain*input.bearing, p.max_turn);
    if (reactivePathClear(input, 0.0, turn, 1.0)) result.w = turn;
    else result.state = "BLOCKED";
    return result;
  }

  // A nearby return is a bypass obstacle when it is clearly in front of the
  // estimated paper plane. The paper's own wall remains the approach target.
  const bool obstacle = input.front_clearance < p.avoid_distance &&
      input.front_clearance + p.target_distance_margin < input.target_front_distance;
  if (obstacle) {
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
      if (reactivePathClear(input, p.max_speed, turn, p.lookahead_time)) {
        result.v = p.max_speed;
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

  result.v = p.max_speed;
  result.w = clipped(p.turn_gain*input.bearing, p.max_turn);
  result.state = "APPROACH";
  return result;
}

}  // namespace maze_planner
