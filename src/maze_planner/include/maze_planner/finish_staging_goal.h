#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <queue>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <maze_planner/navigation_grid.h>

namespace maze_planner {

struct StagingPose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double clearance{0.0};
  double wall_ahead{std::numeric_limits<double>::infinity()};
};

struct FinishStagingOptions {
  double min_clearance{0.37};
  double preferred_clearance{0.50};
  double max_clearance_reward{1.20};
  double passable_clearance{0.30};
  double anchor_search_radius{1.25};
  double target_wall_standoff{1.95};
  double max_wall_ray{3.50};
  double avoid_point_radius{0.45};
};

namespace detail {

inline bool findAutonomousFinishAnchor(const nav_msgs::OccupancyGrid& map,
                                       double origin_yaw,
                                       const std::vector<double>& clearance,
                                       double start_x, double start_y, double start_yaw,
                                       const FinishStagingOptions& opts,
                                       Point2* anchor, double* incoming_yaw) {
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  int sx = 0, sy = 0;
  if (!worldToGrid(map, origin_yaw, start_x, start_y, &sx, &sy)) return false;

  for (double pass_clr : {opts.passable_clearance, 0.22, 0.12}) {
    std::vector<double> geo(static_cast<std::size_t>(w) * h, -1.0);
    using Node = std::pair<double, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    const int seed_r = std::max(2, static_cast<int>(std::ceil(0.35 / res)));
    for (int dy = -seed_r; dy <= seed_r; ++dy) {
      for (int dx = -seed_r; dx <= seed_r; ++dx) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const int idx = ny * w + nx;
        if (map.data[idx] == 0) {
          geo[idx] = 0.0;
          pq.emplace(0.0, idx);
        }
      }
    }
    if (pq.empty()) continue;

    const int dx8[8] = {-1, 1, 0, 0, -1, 1, -1, 1};
    const int dy8[8] = {0, 0, -1, 1, -1, -1, 1, 1};
    const double step8[8] = {res, res, res, res,
                             res * 1.41421356, res * 1.41421356,
                             res * 1.41421356, res * 1.41421356};
    while (!pq.empty()) {
      const Node top = pq.top();
      pq.pop();
      if (top.first > geo[top.second] + 1e-9) continue;
      const int cx = top.second % w;
      const int cy = top.second / w;
      for (int k = 0; k < 8; ++k) {
        const int nx = cx + dx8[k];
        const int ny = cy + dy8[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const int nidx = ny * w + nx;
        if (map.data[nidx] != 0 || clearance[nidx] < pass_clr) continue;
        const double nd = top.first + step8[k];
        if (geo[nidx] < 0.0 || nd + 1e-9 < geo[nidx]) {
          geo[nidx] = nd;
          pq.emplace(nd, nidx);
        }
      }
    }

    for (double open_clr : {opts.preferred_clearance, opts.min_clearance, 0.25}) {
      double max_geo = -1.0;
      for (std::size_t i = 0; i < geo.size(); ++i) {
        if (geo[i] > 0.25 && clearance[i] >= open_clr) {
          max_geo = std::max(max_geo, geo[i]);
        }
      }
      if (max_geo < 0.0) continue;

      int best_idx = -1;
      double best_score = -std::numeric_limits<double>::infinity();
      const double min_far_geo = std::max(0.55 * max_geo, max_geo - 1.40);
      for (std::size_t i = 0; i < geo.size(); ++i) {
        if (geo[i] < min_far_geo || clearance[i] < open_clr) continue;
        const double s = std::min(clearance[i], opts.max_clearance_reward) + 0.25 * geo[i];
        if (s > best_score) {
          best_score = s;
          best_idx = static_cast<int>(i);
        }
      }
      if (best_idx < 0) continue;

      *anchor = gridToWorld(map, origin_yaw, best_idx % w, best_idx / w);
      const double anchor_geo = geo[best_idx];
      Point2 earlier_sum(0.0, 0.0);
      int earlier_count = 0;
      for (std::size_t i = 0; i < geo.size(); ++i) {
        if (geo[i] >= anchor_geo - 1.80 && geo[i] <= anchor_geo - 0.60 &&
            geo[i] >= 0.0) {
          earlier_sum = earlier_sum + gridToWorld(
              map, origin_yaw, static_cast<int>(i) % w, static_cast<int>(i) / w);
          ++earlier_count;
        }
      }
      if (earlier_count > 0) {
        const Point2 prev = (1.0 / earlier_count) * earlier_sum;
        if (norm(*anchor - prev) > 0.15) {
          *incoming_yaw = std::atan2(anchor->y - prev.y, anchor->x - prev.x);
          return true;
        }
      }
      const Point2 from_start = *anchor - Point2(start_x, start_y);
      *incoming_yaw = norm(from_start) > 0.15
          ? std::atan2(from_start.y, from_start.x)
          : start_yaw;
      return true;
    }
  }
  return false;
}

}  // namespace detail

// Select a single obstacle-free staging goal in the ~2.5 m open finish zone
// before the blue-marker wall so map navigation runs continuously without
// stopping or spinning at intermediate waypoints.
inline bool selectFinishStagingGoal(
    const nav_msgs::OccupancyGrid& map,
    double start_x, double start_y, double start_yaw,
    const std::vector<Point2>& course_waypoints,
    const std::vector<Point2>& live_obstacles,
    const std::vector<Point2>& avoid_points,
    const FinishStagingOptions& opts,
    StagingPose* out) {
  if (!out || map.info.width == 0 || map.info.height == 0 ||
      map.info.resolution <= 0.0) {
    return false;
  }
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const double origin_yaw = detail::gridOriginYaw(map);
  const std::vector<double> clearance = detail::computeClearanceField(
      map, origin_yaw, live_obstacles, std::max(1.60, opts.max_clearance_reward + 0.25));

  Point2 anchor;
  double in_yaw = start_yaw;
  if (!course_waypoints.empty()) {
    anchor = course_waypoints.back();
    Point2 prev(start_x, start_y);
    bool found_prev = false;
    for (std::size_t i = course_waypoints.size(); i > 1; --i) {
      const Point2& candidate = course_waypoints[i - 2];
      if (norm(anchor - candidate) >= 0.55) {
        prev = candidate;
        found_prev = true;
        break;
      }
    }
    if (!found_prev && norm(anchor - prev) < 0.20) {
      in_yaw = start_yaw;
    } else {
      in_yaw = std::atan2(anchor.y - prev.y, anchor.x - prev.x);
    }
  } else if (!detail::findAutonomousFinishAnchor(
                 map, origin_yaw, clearance, start_x, start_y, start_yaw, opts,
                 &anchor, &in_yaw)) {
    return false;
  }

  int amx = 0, amy = 0;
  if (!detail::worldToGrid(map, origin_yaw, anchor.x, anchor.y, &amx, &amy)) {
    amx = std::max(0, std::min(w - 1, amx));
    amy = std::max(0, std::min(h - 1, amy));
  }
  // If the nominal anchor cell itself is blocked (e.g. a dynamic obstacle was
  // placed there), snap to the nearest clear cell within 1.1 m.
  if (map.data[static_cast<std::size_t>(amy) * w + amx] != 0 ||
      clearance[static_cast<std::size_t>(amy) * w + amx] < 0.25) {
    const int snap_r = std::max(1, static_cast<int>(std::ceil(1.10 / res)));
    int best_sx = -1, best_sy = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int dy = -snap_r; dy <= snap_r; ++dy) {
      for (int dx = -snap_r; dx <= snap_r; ++dx) {
        const int nx = amx + dx;
        const int ny = amy + dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const std::size_t nidx = static_cast<std::size_t>(ny) * w + nx;
        if (map.data[nidx] == 0 && clearance[nidx] >= opts.min_clearance) {
          const double d2 = static_cast<double>(dx * dx + dy * dy);
          if (d2 < best_d2) {
            best_d2 = d2;
            best_sx = nx;
            best_sy = ny;
          }
        }
      }
    }
    if (best_sx < 0) return false;
    amx = best_sx;
    amy = best_sy;
  }

  // Flood-fill connected open cells around the finish-zone anchor so candidates
  // never jump across a thin divider wall into another corridor.
  const int rad = std::max(1, static_cast<int>(std::ceil(opts.anchor_search_radius / res)));
  std::vector<uint8_t> visited(static_cast<std::size_t>(w) * h, 0);
  std::deque<std::pair<int, int>> q;
  q.emplace_back(amx, amy);
  visited[static_cast<std::size_t>(amy) * w + amx] = 1;
  std::vector<std::pair<int, int>> region;
  const int dx4[4] = {-1, 1, 0, 0};
  const int dy4[4] = {0, 0, -1, 1};
  while (!q.empty()) {
    const auto cur = q.front();
    q.pop_front();
    region.push_back(cur);
    for (int k = 0; k < 4; ++k) {
      const int nx = cur.first + dx4[k];
      const int ny = cur.second + dy4[k];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if ((nx - amx) * (nx - amx) + (ny - amy) * (ny - amy) > rad * rad) continue;
      const std::size_t nidx = static_cast<std::size_t>(ny) * w + nx;
      if (visited[nidx] || map.data[nidx] != 0 || clearance[nidx] < 0.25) continue;
      visited[nidx] = 1;
      q.emplace_back(nx, ny);
    }
  }

  for (double req_clr : {opts.min_clearance, 0.25}) {
    bool found = false;
    double best_score = -std::numeric_limits<double>::infinity();
    StagingPose best;
    const std::size_t stride = region.size() > 400 ? 2 : 1;
    for (std::size_t idx_r = 0; idx_r < region.size(); idx_r += stride) {
      const int cx = region[idx_r].first;
      const int cy = region[idx_r].second;
      const double clr = clearance[static_cast<std::size_t>(cy) * w + cx];
      if (clr < req_clr) continue;
      const Point2 pt = detail::gridToWorld(map, origin_yaw, cx, cy);

      double avoid_penalty = 0.0;
      bool hard_avoided = false;
      for (const Point2& bad : avoid_points) {
        const double d = norm(pt - bad);
        if (d < opts.avoid_point_radius) {
          hard_avoided = true;
          avoid_penalty += 4.0 * (opts.avoid_point_radius - d);
        }
      }
      if (hard_avoided && region.size() > 25) continue;

      double min_wall = std::numeric_limits<double>::infinity();
      for (int deg : {-28, -14, 0, 14, 28}) {
        double d = 0.0;
        if (detail::raycastOccupiedWall(
                map, origin_yaw, pt.x, pt.y, in_yaw + deg * M_PI / 180.0,
                opts.max_wall_ray, &d, nullptr)) {
          min_wall = std::min(min_wall, d);
        }
      }

      double score = 2.0 * std::min(clr, opts.max_clearance_reward) -
                     0.25 * norm(pt - anchor) - avoid_penalty;
      if (std::isfinite(min_wall)) {
        score += 0.60 - 0.65 * std::fabs(min_wall - opts.target_wall_standoff);
        if (min_wall < 0.95) score -= 1.5 * (0.95 - min_wall);
      }

      if (!found || score > best_score) {
        found = true;
        best_score = score;
        best.x = pt.x;
        best.y = pt.y;
        best.clearance = clr;
        best.wall_ahead = min_wall;
      }
    }
    if (found) {
      best.yaw = detail::refineHeadingTowardWall(
          map, origin_yaw, best.x, best.y, in_yaw, opts.max_wall_ray);
      *out = best;
      return true;
    }
  }
  return false;
}

}  // namespace maze_planner
