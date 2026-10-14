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
  // Empty retains the original rectangular model. Otherwise use a validated
  // convex polygon in base_footprint, counter-clockwise, without closing point.
  std::vector<SafetyPoint> vertices;
  bool polygon_bounds_valid{false};
};

inline double cross(const SafetyPoint& a, const SafetyPoint& b,
                    const SafetyPoint& p) {
  return (b.x-a.x)*(p.y-a.y) - (b.y-a.y)*(p.x-a.x);
}

inline bool normalizeConvexFootprint(std::vector<SafetyPoint>* vertices) {
  auto& v = *vertices;
  if (v.size() > 3 && std::hypot(v.front().x-v.back().x,
                               v.front().y-v.back().y) < 1e-9) v.pop_back();
  if (v.size() < 3 || v.size() > 64) return false;
  double twice_area = 0.0;
  for (size_t i=0; i<v.size(); ++i) {
    const auto& a=v[i]; const auto& b=v[(i+1)%v.size()];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        std::hypot(a.x-b.x,a.y-b.y) < 1e-9) return false;
    twice_area += a.x*b.y-b.x*a.y;
  }
  if (std::fabs(twice_area) < 1e-10) return false;
  if (twice_area < 0.0) std::reverse(v.begin(),v.end());
  for (size_t i=0; i<v.size(); ++i) {
    const auto& a=v[i]; const auto& b=v[(i+1)%v.size()];
    // Checking every vertex also rejects self-intersecting star polygons.
    for (const auto& p:v) if (cross(a,b,p) < -1e-10) return false;
    if (cross(a,b,SafetyPoint{0.0,0.0}) < -1e-10) return false;
  }
  return true;
}

inline double squaredDistanceToSegment(const SafetyPoint& p,
                                      const SafetyPoint& a,
                                      const SafetyPoint& b) {
  const double dx=b.x-a.x,dy=b.y-a.y;
  const double t=std::max(0.0,std::min(1.0,
      ((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy)));
  const double ex=p.x-a.x-t*dx,ey=p.y-a.y-t*dy;
  return ex*ex+ey*ey;
}

// Front/rear clearance must start at the actual outline at this lateral
// position, rather than the rectangular envelope's furthest nose. Clamp only
// the narrow safety corridor outside the body's lateral bounds to its edge.
inline void longitudinalBodyLimits(const SafetyFootprint& footprint, double y,
                                   double* rear, double* front) {
  *rear=footprint.rear; *front=footprint.front;
  if (footprint.vertices.empty()) return;
  double ymin=footprint.vertices.front().y,ymax=ymin;
  for (const auto& p:footprint.vertices) {ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);}
  y=std::max(ymin,std::min(ymax,y));
  bool found=false;
  for (size_t i=0;i<footprint.vertices.size();++i) {
    const auto& a=footprint.vertices[i];
    const auto& b=footprint.vertices[(i+1)%footprint.vertices.size()];
    if (y < std::min(a.y,b.y)-1e-10 || y > std::max(a.y,b.y)+1e-10) continue;
    double lo,hi;
    if (std::fabs(a.y-b.y)<1e-10) {lo=std::min(a.x,b.x);hi=std::max(a.x,b.x);}
    else {lo=hi=a.x+(b.x-a.x)*std::max(0.,std::min(1.,(y-a.y)/(b.y-a.y)));}
    if (!found) {*rear=lo;*front=hi;found=true;}
    else {*rear=std::min(*rear,lo);*front=std::max(*front,hi);}
  }
}

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
  if (!footprint.vertices.empty()) {
    if (!std::isfinite(local_x) || !std::isfinite(local_y)) return false;
    const SafetyPoint p{local_x,local_y};
    const double padding=std::max(0.0,footprint.padding);
    if (footprint.polygon_bounds_valid &&
        (local_x < footprint.rear-padding || local_x > footprint.front+padding ||
         std::fabs(local_y) > footprint.half_width+padding)) return false;
    bool inside=true;
    bool within_margin=false;
    for (size_t i=0; i<footprint.vertices.size(); ++i) {
      const auto& a=footprint.vertices[i];
      const auto& b=footprint.vertices[(i+1)%footprint.vertices.size()];
      if (cross(a,b,p) < -1e-10) inside=false;
      if (squaredDistanceToSegment(p,a,b) <= padding*padding+1e-12)
        within_margin=true;
    }
    return inside || within_margin;
  }
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
