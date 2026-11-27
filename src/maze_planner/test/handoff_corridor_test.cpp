#include <gtest/gtest.h>
#include <maze_planner/handoff_corridor.h>

#include <cmath>
#include <limits>

namespace {
sensor_msgs::LaserScan wallScan() {
  sensor_msgs::LaserScan scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0*M_PI/1440;
  scan.range_min = .10;
  scan.range_max = 10.0;
  scan.ranges.resize(1440, 6.0);
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double a = scan.angle_min+i*scan.angle_increment;
    if (std::cos(a) > .2) scan.ranges[i] = (2.0-.315)/std::cos(a);
  }
  return scan;
}

std::vector<stm32_bridge::SafetyPoint> points(const sensor_msgs::LaserScan& scan) {
  std::vector<stm32_bridge::SafetyPoint> result;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    const double a = scan.angle_min+i*scan.angle_increment;
    if (std::isfinite(r) && r > scan.range_min && r < scan.range_max)
      result.push_back({.315+r*std::cos(a), r*std::sin(a)});
  }
  return result;
}
}

TEST(HandoffCorridor, ClearObservedPathAllowsSwitch) {
  auto scan = wallScan();
  auto obstacles = points(scan);
  stm32_bridge::SafetyFootprint footprint;
  footprint.padding = .03;
  maze_planner::HandoffCorridorParameters p;
  std::string reason;
  EXPECT_TRUE(maze_planner::handoffCorridorClear(scan, obstacles, footprint, 2, 0, p, &reason)) << reason;
}

TEST(HandoffCorridor, ObstacleInVehicleWidthBlocksSwitch) {
  auto scan = wallScan();
  const double a = std::atan2(.15, 1.0-.315);
  const int beam = static_cast<int>(std::lround((a-scan.angle_min)/scan.angle_increment));
  for (int i = beam-3; i <= beam+3; ++i) scan.ranges[i] = std::hypot(1.0-.315,.15);
  auto obstacles = points(scan);
  stm32_bridge::SafetyFootprint footprint;
  footprint.padding = .03;
  maze_planner::HandoffCorridorParameters p;
  std::string reason;
  EXPECT_FALSE(maze_planner::handoffCorridorClear(scan, obstacles, footprint, 2, 0, p, &reason));
}

TEST(HandoffCorridor, ObstacleBesideTaperedNoseDoesNotBlockSweep) {
  auto scan = wallScan();
  stm32_bridge::SafetyFootprint footprint;
  footprint.vertices = {{-.15,-.24}, {.14,-.24}, {.35,0}, {.14,.24}, {-.15,.24}};
  ASSERT_TRUE(stm32_bridge::normalizeConvexFootprint(&footprint.vertices));
  footprint.front = .35;
  footprint.rear = -.15;
  footprint.half_width = .24;
  footprint.padding = .03;
  footprint.polygon_bounds_valid = true;
  // This return lies inside the old bounding rectangle, but outside every
  // position of the actual padded footprint along the approach.
  const double x = 1.80, y = .18;
  const double angle = std::atan2(y, x-.315);
  const int beam = static_cast<int>(std::lround((angle-scan.angle_min)/scan.angle_increment));
  for (int i = beam-2; i <= beam+2; ++i) scan.ranges[i] = std::hypot(x-.315,y);
  auto obstacles = points(scan);
  maze_planner::HandoffCorridorParameters p;
  std::string reason;
  EXPECT_TRUE(maze_planner::handoffCorridorClear(scan, obstacles, footprint, 2, 0, p, &reason)) << reason;
}

TEST(HandoffCorridor, MissingRaysAreNotCountedAsClear) {
  auto scan = wallScan();
  const int center = static_cast<int>(std::lround(-scan.angle_min/scan.angle_increment));
  for (int i = center-25; i <= center+25; ++i)
    scan.ranges[i] = std::numeric_limits<float>::infinity();
  auto obstacles = points(scan);
  stm32_bridge::SafetyFootprint footprint;
  footprint.padding = .03;
  maze_planner::HandoffCorridorParameters p;
  std::string reason;
  EXPECT_FALSE(maze_planner::handoffCorridorClear(scan, obstacles, footprint, 2, 0, p, &reason));
  EXPECT_NE(reason.find("not observed"), std::string::npos);
}

TEST(ApproachVelocityLimits, InterpolatesAndFallsBackToNearSpeed) {
  const auto expect_limits = [](double distance, double expected_v, double expected_w,
                                double slowdown_distance = 2.20) {
    SCOPED_TRACE(distance);
    double max_v, max_w;
    maze_planner::approachVelocityLimits(distance, slowdown_distance, 1.35,
                                         .60, .60, .35, .40, &max_v, &max_w);
    EXPECT_DOUBLE_EQ(max_v, expected_v);
    EXPECT_DOUBLE_EQ(max_w, expected_w);
  };
  expect_limits(4.5, .60, .60);
  expect_limits(2.20, .60, .60);
  expect_limits((2.20 + 1.35) / 2.0, .475, .50);
  expect_limits(1.35, .35, .40);
  expect_limits(.80, .35, .40);
  expect_limits(0.0, .35, .40);
  expect_limits(std::numeric_limits<double>::quiet_NaN(), .35, .40);
  expect_limits(4.5, .35, .40, 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
