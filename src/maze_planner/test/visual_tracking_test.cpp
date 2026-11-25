#include <gtest/gtest.h>
#include <maze_planner/visual_tracking.h>

#include <cmath>
#include <limits>

TEST(VisualTracking, CombinesPanAndImageOffsetWithRosSigns) {
  double bearing = 0;
  ASSERT_TRUE(maze_planner::visualBearing(-.30, 0, 320, 240, 320, 240, 450, 450, &bearing));
  EXPECT_NEAR(bearing, -.30, 1e-9);  // Camera reports target to the right.
  ASSERT_TRUE(maze_planner::visualBearing(0, 0, 365, 240, 320, 240, 450, 450, &bearing));
  EXPECT_NEAR(bearing, -std::atan(.1), 1e-9);  // Pixel to the right is ROS negative.
  ASSERT_TRUE(maze_planner::visualBearing(.10, 0, 365, 240, 320, 240, 450, 450, &bearing));
  EXPECT_NEAR(bearing, .10-std::atan(.1), 1e-9);
  EXPECT_FALSE(maze_planner::visualBearing(0, 0, 320, 240, 320, 240, 0, 450, &bearing));
}

TEST(VisualTracking, StopsOnlyWhenFacingAndAreaAreBothReached) {
  const double five_degrees = 5.0 * M_PI / 180.0;
  EXPECT_TRUE(maze_planner::visualStopReached(true, .16, five_degrees, .16, five_degrees));
  EXPECT_TRUE(maze_planner::visualStopReached(true, static_cast<float>(.16), 0, .16, five_degrees));
  EXPECT_FALSE(maze_planner::visualStopReached(false, .25, 0, .16, five_degrees));
  EXPECT_FALSE(maze_planner::visualStopReached(true, .159, 0, .16, five_degrees));
  EXPECT_FALSE(maze_planner::visualStopReached(true, .25, .10, .16, five_degrees));
  EXPECT_FALSE(maze_planner::visualStopReached(true, std::numeric_limits<double>::quiet_NaN(),
                                               0, .16, five_degrees));
}

TEST(VisualTracking, GoalPointsTowardMarkerWithoutRequestingReverse) {
  double x, y, yaw;
  ASSERT_TRUE(maze_planner::visualApproachGoal(0, 0, 2, -1, .48, &x, &y, &yaw));
  EXPECT_LT(y, 0.0);
  EXPECT_LT(yaw, 0.0);
  EXPECT_NEAR(std::hypot(2-x, -1-y), .48, 1e-9);
  ASSERT_TRUE(maze_planner::visualApproachGoal(0, 0, .2, 0, .48, &x, &y, &yaw));
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_FALSE(maze_planner::visualApproachGoal(0, 0, 0, 0, .48, &x, &y, &yaw));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
