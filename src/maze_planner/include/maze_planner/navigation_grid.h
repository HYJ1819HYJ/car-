#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <queue>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <maze_planner/wall_parking.h>

namespace maze_planner {
namespace detail {

inline double gridOriginYaw(const nav_msgs::OccupancyGrid& map) {
  const auto& q = map.info.origin.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline bool worldToGrid(const nav_msgs::OccupancyGrid& map, double origin_yaw,
                        double wx, double wy, int* mx, int* my) {
  const double res = map.info.resolution;
  if (res <= 0.0 || !std::isfinite(wx) || !std::isfinite(wy)) return false;
  const double dx = wx - map.info.origin.position.x;
  const double dy = wy - map.info.origin.position.y;
  const double lx = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
  const double ly = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
  *mx = static_cast<int>(std::floor(lx / res));
  *my = static_cast<int>(std::floor(ly / res));
  return *mx >= 0 && *my >= 0 &&
         *mx < static_cast<int>(map.info.width) &&
         *my < static_cast<int>(map.info.height);
}

inline Point2 gridToWorld(const nav_msgs::OccupancyGrid& map, double origin_yaw,
                          int mx, int my) {
  const double lx = (mx + 0.5) * map.info.resolution;
  const double ly = (my + 0.5) * map.info.resolution;
  return Point2(
      map.info.origin.position.x + std::cos(origin_yaw) * lx - std::sin(origin_yaw) * ly,
      map.info.origin.position.y + std::sin(origin_yaw) * lx + std::cos(origin_yaw) * ly);
}

inline std::vector<double> computeClearanceField(
    const nav_msgs::OccupancyGrid& map, double origin_yaw,
    const std::vector<Point2>& live_obstacles, double cap_meters = 1.60,
    int obstacle_threshold = 1) {
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const std::size_t total = static_cast<std::size_t>(w) * h;
  std::vector<double> dist(total, cap_meters);
  if (w <= 0 || h <= 0 || res <= 0.0 || map.data.size() < total) return dist;

  using Node = std::pair<double, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = y * w + x;
      const bool border = (x == 0 || y == 0 || x == w - 1 || y == h - 1);
      if (map.data[idx] < 0 || map.data[idx] >= obstacle_threshold || border) {
        dist[idx] = 0.0;
        pq.emplace(0.0, idx);
      }
    }
  }
  for (const Point2& obs : live_obstacles) {
    int ox = 0, oy = 0;
    if (worldToGrid(map, origin_yaw, obs.x, obs.y, &ox, &oy)) {
      const int idx = oy * w + ox;
      if (dist[idx] > 0.0) {
        dist[idx] = 0.0;
        pq.emplace(0.0, idx);
      }
    }
  }

  const int dx8[8] = {-1, 1, 0, 0, -1, 1, -1, 1};
  const int dy8[8] = {0, 0, -1, 1, -1, -1, 1, 1};
  const double step8[8] = {res, res, res, res,
                           res * 1.41421356, res * 1.41421356,
                           res * 1.41421356, res * 1.41421356};

  while (!pq.empty()) {
    const Node top = pq.top();
    pq.pop();
    if (top.first > dist[top.second] + 1e-9) continue;
    if (top.first >= cap_meters) continue;
    const int cx = top.second % w;
    const int cy = top.second / w;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dx8[k];
      const int ny = cy + dy8[k];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const int nidx = ny * w + nx;
      const double nd = top.first + step8[k];
      if (nd + 1e-9 < dist[nidx] && nd <= cap_meters) {
        dist[nidx] = nd;
        pq.emplace(nd, nidx);
      }
    }
  }
  return dist;
}

inline bool raycastOccupiedWall(const nav_msgs::OccupancyGrid& map, double origin_yaw,
                                double wx, double wy, double angle, double max_range,
                                double* hit_dist, Point2* hit_pt) {
  const double res = map.info.resolution;
  if (res <= 0.0 || max_range <= 0.0 ||
      map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height)
    return false;
  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double step = 0.5 * res;
  const int steps = std::max(1, static_cast<int>(std::ceil(max_range / step)));
  const double ca = std::cos(angle);
  const double sa = std::sin(angle);
  for (int i = 1; i <= steps; ++i) {
    const double d = std::min(max_range, i * step);
    const double px = wx + d * ca;
    const double py = wy + d * sa;
    int mx = 0, my = 0;
    if (!worldToGrid(map, origin_yaw, px, py, &mx, &my)) return false;
    const int8_t cell = map.data[static_cast<std::size_t>(my) * w + mx];
    if (cell >= 50) {
      if (hit_dist) *hit_dist = d;
      if (hit_pt) *hit_pt = Point2(px, py);
      return true;
    }
    if (cell < 0) {
      // Unknown boundary is not a physical wall.
      return false;
    }
  }
  return false;
}

inline double refineHeadingTowardWall(const nav_msgs::OccupancyGrid& map,
                                      double origin_yaw, double gx, double gy,
                                      double nominal_yaw, double max_ray) {
  std::vector<Point2> hits;
  hits.reserve(32);
  for (int deg = -56; deg <= 56; deg += 4) {
    const double ang = nominal_yaw + deg * M_PI / 180.0;
    double d = 0.0;
    Point2 pt;
    if (raycastOccupiedWall(map, origin_yaw, gx, gy, ang, max_ray, &d, &pt)) {
      hits.push_back(pt);
    }
  }
  if (hits.size() < 6) return nominal_yaw;

  // RANSAC dominant wall segment so side-wall hits do not skew the finish-wall normal.
  std::vector<std::size_t> best_inliers;
  for (std::size_t i = 0; i < hits.size(); ++i) {
    for (std::size_t j = i + 1; j < hits.size(); ++j) {
      const Point2 edge = hits[j] - hits[i];
      const double len = norm(edge);
      if (len < 0.25) continue;
      const Point2 normal(-edge.y / len, edge.x / len);
      std::vector<std::size_t> inliers;
      for (std::size_t k = 0; k < hits.size(); ++k) {
        if (std::fabs(dot(hits[k] - hits[i], normal)) <= 0.05) inliers.push_back(k);
      }
      if (inliers.size() > best_inliers.size()) best_inliers.swap(inliers);
    }
  }
  if (best_inliers.size() < 5) return nominal_yaw;

  Point2 mean(0.0, 0.0);
  for (std::size_t idx : best_inliers) mean = mean + hits[idx];
  mean = (1.0 / static_cast<double>(best_inliers.size())) * mean;
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (std::size_t idx : best_inliers) {
    const Point2 d = hits[idx] - mean;
    xx += d.x * d.x;
    xy += d.x * d.y;
    yy += d.y * d.y;
  }
  const double line_ang = 0.5 * std::atan2(2.0 * xy, xx - yy);
  Point2 into_wall(-std::sin(line_ang), std::cos(line_ang));
  const Point2 to_wall = mean - Point2(gx, gy);
  if (dot(into_wall, to_wall) < 0.0) into_wall = -1.0 * into_wall;

  const Point2 nominal_dir(std::cos(nominal_yaw), std::sin(nominal_yaw));
  if (dot(into_wall, nominal_dir) < std::cos(65.0 * M_PI / 180.0)) return nominal_yaw;
  return std::atan2(into_wall.y, into_wall.x);
}

}  // namespace detail
}  // namespace maze_planner
