#pragma once
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace maze_smooth_planner {
struct Pose { double x, y, yaw; };
struct Config {
  double sample_spacing = 0.025;
  double shortcut_deviation = 0.06;
  double shortcut_length = 0.80;
  double preferred_radius = 0.35;
  double smoothing_clearance = 0.05;
  double collision_padding = 0.02;
  double max_compute_ms = 80.0;
  double max_lateral_adjustment = 0.15;
  double goal_tangent_length = 0.30;
};
struct Statistics { int rounded_corners = 0; int repair_steps = 0; bool fallback = false; double elapsed_ms = 0; Pose failed_pose{0,0,0}; };
// Unknown cells and map boundaries are blocked. Checks polygon interiors,
// not just footprint edges. Rotational sweep uses angular subdivisions.
bool poseFree(const Pose&, const costmap_2d::Costmap2D&,
              const std::vector<geometry_msgs::Point>&, double padding);
bool pathFree(const std::vector<Pose>&, const costmap_2d::Costmap2D&,
              const std::vector<geometry_msgs::Point>&, double padding, Pose* failed_pose=nullptr);
bool smooth(const std::vector<Pose>&, const costmap_2d::Costmap2D&,
            const std::vector<geometry_msgs::Point>&, const Config&,
            std::vector<Pose>&, Statistics&);
}
