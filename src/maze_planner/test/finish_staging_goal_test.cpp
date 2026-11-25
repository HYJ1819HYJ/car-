#include <gtest/gtest.h>
#include <maze_planner/finish_staging_goal.h>

#include <cmath>
#include <vector>

namespace {

// Build an L-shaped arena resembling the competition layout:
// Horizontal entry corridor: x in [-0.5, 4.0], y in [-1.0, 1.0]
// Vertical finish leg: x in [1.0, 4.0], y in [-6.5, 1.0], with a box obstacle
// at x in [2.6, 3.2], y in [-3.5, -2.9], and an obstacle-free 2.5 m finish zone
// at y in [-6.5, -3.9] before the bottom finish wall y = -6.5.
nav_msgs::OccupancyGrid makeLShapedCompetitionMap() {
  nav_msgs::OccupancyGrid grid;
  grid.info.resolution = 0.05;
  grid.info.width = 100;   // x: -1.0 .. 4.0
  grid.info.height = 160;  // y: -7.0 .. 1.0
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -7.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(grid.info.width * grid.info.height, -1);

  for (unsigned int my = 0; my < grid.info.height; ++my) {
    const double y = grid.info.origin.position.y + (my + 0.5) * grid.info.resolution;
    for (unsigned int mx = 0; mx < grid.info.width; ++mx) {
      const double x = grid.info.origin.position.x + (mx + 0.5) * grid.info.resolution;
      const bool in_top = (x >= -0.5 && x <= 3.9 && y >= -1.0 && y <= 0.9);
      const bool in_leg = (x >= 1.0 && x <= 3.9 && y >= -6.5 && y <= 0.9);
      const bool in_free = in_top || in_leg;
      const bool on_wall =
          (std::fabs(y - 0.9) <= 0.06 && x >= -0.5 && x <= 3.95) ||
          (std::fabs(x + 0.5) <= 0.06 && y >= -1.0 && y <= 0.95) ||
          (std::fabs(y + 1.0) <= 0.06 && x >= -0.5 && x <= 1.05) ||
          (std::fabs(x - 1.0) <= 0.06 && y >= -6.55 && y <= -1.0) ||
          (std::fabs(x - 3.9) <= 0.06 && y >= -6.55 && y <= 0.95) ||
          (std::fabs(y + 6.5) <= 0.06 && x >= 1.0 && x <= 3.95);
      const bool on_box = (x >= 2.6 && x <= 3.2 && y >= -3.5 && y <= -2.9);
      const std::size_t idx = static_cast<std::size_t>(my) * grid.info.width + mx;
      if (on_wall || on_box) {
        grid.data[idx] = 100;
      } else if (in_free) {
        grid.data[idx] = 0;
      }
    }
  }
  return grid;
}

}  // namespace

TEST(FinishStagingGoal, CentersInObstacleFreeFinishZoneAndFacesFinishWall) {
  const auto map = makeLShapedCompetitionMap();
  // Waypoints ending near the right side of the 2.5 m finish zone, with a skew segment.
  const std::vector<maze_planner::Point2> course = {
      {0.8, -0.2}, {2.4, -0.1}, {3.3, -1.0}, {2.2, -3.2}, {2.3, -4.0}, {3.2, -4.0}, {3.1, -4.6}};
  maze_planner::FinishStagingOptions opts;
  maze_planner::StagingPose pose;
  ASSERT_TRUE(maze_planner::selectFinishStagingGoal(
      map, 0.0, -0.2, 0.0, course, {}, {}, opts, &pose));

  // Should land in the obstacle-free zone y in [-5.3, -4.2], well away from side walls
  // (x=1.0 and x=3.9) and facing -90 deg toward the y=-6.5 finish wall.
  EXPECT_GT(pose.x, 1.8);
  EXPECT_LT(pose.x, 3.1);
  EXPECT_GT(pose.y, -5.4);
  EXPECT_LT(pose.y, -4.1);
  EXPECT_GT(pose.clearance, 0.80);
  EXPECT_NEAR(pose.yaw, -M_PI / 2.0, 0.20);
}

TEST(FinishStagingGoal, DiscoversFinishZoneAutonomouslyWithoutWaypoints) {
  const auto map = makeLShapedCompetitionMap();
  maze_planner::FinishStagingOptions opts;
  maze_planner::StagingPose pose;
  ASSERT_TRUE(maze_planner::selectFinishStagingGoal(
      map, 0.0, -0.2, 0.0, {}, {}, {}, opts, &pose));

  // Even without course_waypoints, geodesic analysis should place the staging goal
  // in the far 2.5 m obstacle-free finish zone facing the bottom wall.
  EXPECT_GT(pose.x, 1.7);
  EXPECT_LT(pose.x, 3.2);
  EXPECT_LT(pose.y, -4.0);
  EXPECT_GT(pose.y, -5.6);
  EXPECT_GT(pose.clearance, 0.75);
  EXPECT_NEAR(pose.yaw, -M_PI / 2.0, 0.25);
}

TEST(FinishStagingGoal, AvoidsPreviouslyFailedCandidateAndLiveObstacle) {
  const auto map = makeLShapedCompetitionMap();
  const std::vector<maze_planner::Point2> course = {{2.3, -3.9}, {2.5, -4.7}};
  maze_planner::FinishStagingOptions opts;
  maze_planner::StagingPose first;
  ASSERT_TRUE(maze_planner::selectFinishStagingGoal(
      map, 0.0, -0.2, 0.0, course, {}, {}, opts, &first));

  maze_planner::StagingPose retry;
  ASSERT_TRUE(maze_planner::selectFinishStagingGoal(
      map, 0.0, -0.2, 0.0, course, {{first.x, first.y}}, {{first.x, first.y}},
      opts, &retry));
  EXPECT_GE(std::hypot(retry.x - first.x, retry.y - first.y), opts.avoid_point_radius);
  EXPECT_GE(retry.clearance, opts.min_clearance);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
