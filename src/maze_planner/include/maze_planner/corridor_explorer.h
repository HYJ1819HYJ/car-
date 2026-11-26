#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <maze_planner/wall_parking.h>

#include <array>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>

namespace maze_planner {

// OccupancyGrid encoding from costmap_2d: 99 is inscribed, 100 is lethal.
class CorridorGrid {
 public:
  explicit CorridorGrid(const nav_msgs::OccupancyGrid& map)
      : width(map.info.width), height(map.info.height), resolution(map.info.resolution),
        origin(map.info.origin.position.x, map.info.origin.position.y),
        yaw(2 * std::atan2(map.info.origin.orientation.z, map.info.origin.orientation.w)),
        cost(map.data) {}

  bool valid() const {
    return width > 0 && height > 0 && resolution > 0 &&
           cost.size() == static_cast<std::size_t>(width) * height;
  }
  int index(int x, int y) const {
    return x >= 0 && y >= 0 && x < width && y < height ? y * width + x : -1;
  }
  Point2 local(const Point2& p) const { return (1.0 / resolution) * rotate(p - origin, -yaw); }
  int cell(const Point2& p) const {
    const auto v = local(p);
    if (!(v.x >= 0 && v.y >= 0 && v.x < width && v.y < height)) return -1;
    return index(std::floor(v.x), std::floor(v.y));
  }
  Point2 center(int i) const {
    return origin + resolution * rotate({i % width + .5, i / width + .5}, yaw);
  }
  bool lethal(int i) const { return i < 0 || cost[i] < 0 || cost[i] >= 100; }

  bool clear(const Point2& p, double radius, int center_limit) const {
    if (!valid()) return false;
    const int i = cell(p);
    if (i < 0 || lethal(i) || cost[i] >= center_limit) return false;
    const int cells = std::ceil(radius / resolution);
    const auto v = local(p);
    for (int y = i / width - cells; y <= i / width + cells; ++y) {
      for (int x = i % width - cells; x <= i % width + cells; ++x) {
        if (!lethal(index(x, y))) continue;
        const double dx = v.x - std::max<double>(x, std::min<double>(v.x, x + 1));
        const double dy = v.y - std::max<double>(y, std::min<double>(v.y, y + 1));
        if (std::hypot(dx, dy) * resolution < radius) return false;
      }
    }
    return true;
  }

  bool footprintClear(const Point2& pose, double heading,
                      const std::vector<Point2>& footprint, double padding = 0.0) const {
    if (!valid() || footprint.size() < 3) return false;
    const int i = cell(pose);
    if (lethal(i) || cost[i] >= 99) return false;
    std::vector<Point2> polygon;
    double x_min = width, y_min = height, x_max = 0, y_max = 0;
    for (const auto& p : footprint) {
      // costmap_2d::padFootprint adds sign0(coordinate) * padding per axis.
      const Point2 padded(p.x + (p.x < 0.0 ? -padding : padding),
                          p.y + (p.y < 0.0 ? -padding : padding));
      const auto v = local(pose + rotate(padded, heading));
      polygon.push_back(v);
      x_min = std::min(x_min, v.x); y_min = std::min(y_min, v.y);
      x_max = std::max(x_max, v.x); y_max = std::max(y_max, v.y);
    }
    if (x_min < 0 || y_min < 0 || x_max >= width || y_max >= height) return false;
    // The STL footprint is a convex hull. Test overlap with the whole cell
    // using separating axes from the polygon and from the cell.
    for (int y = std::floor(y_min); y <= std::floor(y_max); ++y) {
      for (int x = std::floor(x_min); x <= std::floor(x_max); ++x) {
        if (!lethal(index(x, y))) continue;
        bool separated = false;
        for (std::size_t k = 0; k < polygon.size(); ++k) {
          const auto edge = polygon[(k + 1) % polygon.size()] - polygon[k];
          const Point2 axis(-edge.y, edge.x);
          double low = dot(axis, polygon.front()), high = low;
          for (const auto& v : polygon) {
            low = std::min(low, dot(axis, v));
            high = std::max(high, dot(axis, v));
          }
          const double middle = dot(axis, {x + .5, y + .5});
          const double radius = .5 * (std::abs(axis.x) + std::abs(axis.y));
          if (high < middle - radius || low > middle + radius) {
            separated = true;
            break;
          }
        }
        if (!separated) return false;
      }
    }
    return true;
  }

  int width, height;
  double resolution;
  Point2 origin;
  double yaw;
  std::vector<int8_t> cost;
};

class CorridorExplorer {
 public:
  void reset(const Point2& start) {
    start_ = start;
    visited_.clear();
    track_.assign(1, start);
  }

  void track(const Point2& robot) {
    if (track_.empty() || norm(robot - track_.back()) >= .4) track_.push_back(robot);
  }

  void visit(const Point2& point) { visited_.push_back(point); }

  bool select(const CorridorGrid& grid, const Point2& robot, Point2* target,
              double* yaw, const Point2* advance_past = nullptr,
              const std::vector<Point2>* footprint = nullptr,
              double padding = 0.0) const {
    if (!grid.valid()) return false;
    std::vector<int> obstacles;
    for (int i = 0; i < grid.width * grid.height; ++i)
      if (grid.lethal(i) || i % grid.width == 0 || i % grid.width == grid.width - 1 ||
          i / grid.width == 0 || i / grid.width == grid.height - 1) obstacles.push_back(i);
    const auto clearance = distances(grid, obstacles, nullptr, 1.5);
    const auto from_robot = distances(grid, {grid.cell(robot)}, &clearance);
    const int start = grid.cell(start_);
    auto from_start = distances(grid, {start}, &clearance);
    if (start < 0 || grid.lethal(start)) from_start = from_robot;
    const int previous = advance_past ? grid.cell(*advance_past) : -1;
    double score = -std::numeric_limits<double>::infinity();
    int best = -1;
    double best_heading = 0.0;
    for (int i = 0; i < grid.width * grid.height; ++i) {
      const double clearance_m = std::min(1.5, clearance[i]);
      const double depth = std::isfinite(from_start[i]) ? from_start[i] : from_robot[i];
      if (grid.cost[i] >= 95 || clearance_m < .38 || from_robot[i] <= .40 ||
          !std::isfinite(from_robot[i])) continue;
      if (advance_past && (previous < 0 || depth <= (std::isfinite(from_start[previous])
              ? from_start[previous] : from_robot[previous]) + .15))
        continue;
      const Point2 p = grid.center(i);
      if (std::any_of(visited_.begin(), visited_.end(),
                      [&](const Point2& v) { return norm(p - v) <= 1.4; })) continue;
      const bool tracked = std::any_of(track_.begin(), track_.end(),
                      [&](const Point2& v) { return norm(p - v) < .4; });
      const double candidate = depth + 1.6 * std::min(clearance_m, 1.0) -
                               .15 * from_robot[i] - (tracked ? .20 : 0.0);
      if (candidate <= score || !grid.clear(p, .38, 95)) continue;
      const double heading = arrivalHeading(grid, from_robot, i);
      if (footprint && !grid.footprintClear(p, heading, *footprint, padding)) continue;
      best = i;
      score = candidate;
      best_heading = heading;
    }
    if (best < 0) return false;
    *target = grid.center(best);
    *yaw = best_heading;
    return true;
  }

 private:
  // Use a short backward geodesic trace to suppress grid-direction noise.
  static double arrivalHeading(const CorridorGrid& grid, const std::vector<double>& from_robot,
                               int target) {
    int behind = target;
    for (int step = 0; step < std::ceil(.35 / grid.resolution); ++step) {
      int next = behind;
      for (const int id : neighbors(grid, behind)) {
        if (id >= 0 && from_robot[id] < from_robot[next]) next = id;
      }
      if (next == behind) break;
      behind = next;
    }
    const auto direction = grid.center(target) - grid.center(behind);
    return std::atan2(direction.y, direction.x);
  }

  static std::array<int, 8> neighbors(const CorridorGrid& grid, int i) {
    const int x = i % grid.width, y = i / grid.width;
    return {{grid.index(x - 1, y), grid.index(x + 1, y), grid.index(x, y - 1),
             grid.index(x, y + 1), grid.index(x - 1, y - 1), grid.index(x + 1, y - 1),
             grid.index(x - 1, y + 1), grid.index(x + 1, y + 1)}};
  }

  static std::vector<double> distances(const CorridorGrid& grid, const std::vector<int>& seeds,
                                       const std::vector<double>* clearance,
                                       double limit = std::numeric_limits<double>::infinity()) {
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    std::vector<double> distance(grid.cost.size(), std::numeric_limits<double>::infinity());
    for (int seed : seeds) {
      if (seed < 0 || (clearance && grid.lethal(seed))) continue;
      distance[seed] = 0;
      queue.emplace(0, seed);
    }
    const auto passable = [&](int i) {
      return i >= 0 && (!clearance ||
             (!grid.lethal(i) && grid.cost[i] < 99 && (*clearance)[i] >= .28));
    };
    while (!queue.empty()) {
      const auto entry = queue.top();
      queue.pop();
      if (entry.first != distance[entry.second]) continue;
      const auto adjacent = neighbors(grid, entry.second);
      for (std::size_t k = 0; k < adjacent.size(); ++k) {
        const int i = adjacent[k];
        if (!passable(i)) continue;
        if (k >= 4 && (!passable(adjacent[(k - 4) % 2]) || !passable(adjacent[k / 2])))
          continue;
        const double d = entry.first + grid.resolution * (k < 4 ? 1.0 : std::sqrt(2.0));
        if (d < distance[i] && d <= limit) {
          distance[i] = d;
          queue.emplace(d, i);
        }
      }
    }
    return distance;
  }

  Point2 start_;
  std::vector<Point2> visited_, track_;
};

inline bool selectFinishEntrance(const CorridorGrid& grid, const WallObservation& wall,
                                 const std::vector<Point2>& footprint,
                                 const std::vector<Point2>& failed, Point2* goal,
                                 double padding = 0.0, const Point2* robot = nullptr,
                                 double robot_yaw = 0.0, bool alternate_view = false) {
  const Point2 tangent(-wall.outward.y, wall.outward.x);
  const double yaw = parkingYaw(wall);
  const std::array<double, 5> depths = alternate_view
      ? std::array<double, 5>{1.15, .68, .80, .95, 1.35}
      : std::array<double, 5>{.80, .95, 1.15, .68, 1.35};
  for (double depth : depths) {
    for (double side : {0.0, -.12, .12, -.25, .25}) {
      const auto p = parkingPosition(wall, depth) + side * tangent;
      if (std::any_of(failed.begin(), failed.end(),
                      [&](const Point2& other) { return norm(p - other) < .09; }))
        continue;
      if (robot) {
        const auto offset = p - *robot;
        const double distance = norm(offset);
        if (distance < .55 && dot(offset, {std::cos(robot_yaw), std::sin(robot_yaw)}) <
                                  -.5 * distance) continue;
      }
      if (grid.footprintClear(p, yaw, footprint, padding)) {
        *goal = p;
        return true;
      }
    }
  }
  return false;
}

}  // namespace maze_planner
