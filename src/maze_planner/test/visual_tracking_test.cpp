#include <gtest/gtest.h>
#include <maze_planner/visual_tracking.h>
#include <maze_planner/gimbal_history.h>

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

TEST(VisualTracking, TargetHeightUsesPositiveImageAndJointPitchDown) {
  const auto plausible = [](double tilt, double y, double range) {
    return maze_planner::plausibleTargetHeight(.42, range,
                                               tilt + std::atan2(y - 240, 331.4), .02, .72);
  };
  EXPECT_FALSE(plausible(0.0, 35, 4.2));
  EXPECT_FALSE(plausible(0.0, 474, 4.2));
  EXPECT_TRUE(plausible(.15, 190, 2.0));
  EXPECT_TRUE(plausible(-.15, 300, 2.0));
  EXPECT_FALSE(plausible(-.18, 240, 4.2));
  EXPECT_FALSE(plausible(.18, 240, 4.2));
  for (const double range : {.50, 2.0, 4.2, 8.0}) {
    const double pixel_y = 240 + 331.4 * std::tan(std::asin((.42 - .30) / range));
    EXPECT_TRUE(plausible(0.0, pixel_y, range));
  }
}

TEST(GimbalHistory, InterpolatesThroughDroppedFramesAndSupportsLegacyRate) {
  maze_planner::GimbalHistory history;
  sensor_msgs::JointState joint;
  joint.name = {"camera_pan_joint", "camera_tilt_joint"};
  joint.position = {0.0, -.20};
  joint.header.stamp = ros::Time(10.0);
  history.add(joint);
  double pan, tilt;
  ASSERT_TRUE(history.angles(ros::Time(10.28), &pan, &tilt));
  EXPECT_DOUBLE_EQ(tilt, -.20);

  joint.header.stamp = ros::Time(10.28);
  joint.position = {.28, .08};
  history.add(joint);
  ASSERT_TRUE(history.angles(ros::Time(10.14), &pan, &tilt));
  EXPECT_NEAR(pan, .14, 1e-9);
  EXPECT_NEAR(tilt, -.06, 1e-9);
  EXPECT_TRUE(history.angles(ros::Time(10.78), &pan, &tilt));
  EXPECT_FALSE(history.angles(ros::Time(10.84), &pan, &tilt));
  EXPECT_FALSE(history.angles(ros::Time(10.78), &pan, &tilt, .25));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
