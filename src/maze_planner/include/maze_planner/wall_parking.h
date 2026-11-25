#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace maze_planner {

struct Point2 {
  double x{0.0};
  double y{0.0};
  Point2() = default;
  Point2(double x_value, double y_value) : x(x_value), y(y_value) {}
};

inline Point2 operator+(const Point2& a, const Point2& b) {
  return Point2(a.x + b.x, a.y + b.y);
}
inline Point2 operator-(const Point2& a, const Point2& b) {
  return Point2(a.x - b.x, a.y - b.y);
}
inline Point2 operator*(double scale, const Point2& point) {
  return Point2(scale * point.x, scale * point.y);
}
inline double dot(const Point2& a, const Point2& b) {
  return a.x * b.x + a.y * b.y;
}
inline double norm(const Point2& point) { return std::hypot(point.x, point.y); }
inline Point2 rotate(const Point2& point, double yaw) {
  return Point2(std::cos(yaw) * point.x - std::sin(yaw) * point.y,
                std::sin(yaw) * point.x + std::cos(yaw) * point.y);
}

struct WallObservation {
  Point2 marker;   // Intersection of the camera bearing with the wall.
  Point2 outward;  // Unit normal from the wall toward the camera.
  std::size_t inliers{0};
  double span{0.0};
};

// Laser points, camera position and bearing are expressed in the same frame.
// Fit an observed wall around the blue-paper bearing, allowing paper-sized gaps.
inline bool fitTargetWall(const std::vector<Point2>& scan, const Point2& camera,
                          double bearing, WallObservation* wall,
                          double max_distance = 4.0) {
  if (!wall || !std::isfinite(bearing) || !std::isfinite(max_distance) ||
      max_distance <= 0.15) return false;
  const Point2 ray(std::cos(bearing), std::sin(bearing));
  const Point2 across(-ray.y, ray.x);
  std::vector<Point2> points;
  points.reserve(scan.size());
  for (const Point2& point : scan) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
    const Point2 delta = point - camera;
    const double forward = dot(delta, ray);
    if (forward > 0.15 && forward < max_distance && std::fabs(dot(delta, across)) < 0.55)
      points.push_back(point);
  }
  if (points.size() < 20) return false;

  std::mt19937 generator(20261124);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
  std::vector<std::size_t> best;
  double best_span = 0.0;
  for (int attempt = 0; attempt < 512; ++attempt) {
    const Point2 a = points[pick(generator)];
    const Point2 b = points[pick(generator)];
    const Point2 edge = b - a;
    const double length = norm(edge);
    if (length < 0.25) continue;
    const Point2 tangent((b.x - a.x) / length, (b.y - a.y) / length);
    const Point2 normal(-tangent.y, tangent.x);
    const double denominator = dot(normal, ray);
    if (std::fabs(denominator) < 0.35) continue;
    const double distance = dot(normal, a - camera) / denominator;
    if (distance <= 0.15 || distance >= max_distance) continue;
    std::vector<std::size_t> inliers;
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (std::fabs(dot(points[i] - a, normal)) >= 0.025) continue;
      inliers.push_back(i);
      const double projection = dot(points[i], tangent);
      lo = std::min(lo, projection);
      hi = std::max(hi, projection);
    }
    if (inliers.size() < 20 || hi - lo < 0.30) continue;
    const Point2 intersection = camera + distance * ray;
    const double projection = dot(intersection, tangent);
    if (projection < lo - 0.15 || projection > hi + 0.15) continue;
    if (inliers.size() > best.size()) {
      best.swap(inliers);
      best_span = hi - lo;
    }
  }
  if (best.empty()) return false;

  Point2 mean;
  for (std::size_t index : best) mean = mean + points[index];
  mean = (1.0 / best.size()) * mean;
  double xx = 0.0, xy = 0.0, yy = 0.0;
  for (std::size_t index : best) {
    const Point2 delta = points[index] - mean;
    xx += delta.x * delta.x;
    xy += delta.x * delta.y;
    yy += delta.y * delta.y;
  }
  const double angle = 0.5 * std::atan2(2.0 * xy, xx - yy);
  const Point2 tangent(std::cos(angle), std::sin(angle));
  Point2 normal(-tangent.y, tangent.x);
  if (dot(normal, camera - mean) < 0.0) normal = -1.0 * normal;
  const double denominator = dot(normal, ray);
  if (std::fabs(denominator) < 0.35) return false;
  const double distance = dot(normal, mean - camera) / denominator;
  if (distance <= 0.15 || distance >= max_distance) return false;
  wall->marker = camera + distance * ray;
  wall->outward = normal;
  wall->inliers = best.size();
  wall->span = best_span;
  return true;
}

inline Point2 parkingPosition(const WallObservation& wall, double wall_distance) {
  return wall.marker + wall_distance * wall.outward;
}

inline double parkingYaw(const WallObservation& wall) {
  return std::atan2(-wall.outward.y, -wall.outward.x);
}

inline bool footprintInsideFinish(const WallObservation& wall,
                                  const Point2& robot, double yaw,
                                  const std::vector<Point2>& footprint,
                                  double size = 0.8) {
  if (footprint.size() < 3) return false;
  const Point2 tangent(-wall.outward.y, wall.outward.x);
  for (const Point2& vertex : footprint) {
    const Point2 delta = robot + rotate(vertex, yaw) - wall.marker;
    const double depth = dot(delta, wall.outward);
    const double lateral = dot(delta, tangent);
    if (depth < 0.0 || depth > size || std::fabs(lateral) > size * 0.5)
      return false;
  }
  return true;
}

}  // namespace maze_planner
