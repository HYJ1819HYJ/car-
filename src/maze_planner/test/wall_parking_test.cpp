#include <gtest/gtest.h>

#include <maze_planner/wall_parking.h>

#include <cmath>
#include <vector>

using maze_planner::Point2;
using maze_planner::WallObservation;

TEST(WallParking, FitsWallThroughPaperBearingWithGapAndClutter) {
  std::vector<Point2> scan;
  for (int i = -50; i <= 50; ++i) {
    const double y = i * 0.01;
    if (std::fabs(y - 0.18) > 0.11) scan.emplace_back(2.0, y);
  }
  for (int i = 0; i < 15; ++i) scan.emplace_back(0.8 + 0.02 * i, 0.4);
  WallObservation wall;
  ASSERT_TRUE(maze_planner::fitTargetWall(scan, Point2(0.145, 0), 0.1, &wall));
  EXPECT_NEAR(wall.marker.x, 2.0, 0.01);
  EXPECT_NEAR(wall.marker.y, 1.855 * std::tan(0.1), 0.02);
  EXPECT_NEAR(wall.outward.x, -1.0, 0.02);
  EXPECT_NEAR(wall.outward.y, 0.0, 0.02);
  const Point2 goal = maze_planner::parkingPosition(wall, 0.56);
  EXPECT_NEAR(goal.x, 1.44, 0.01);
  EXPECT_NEAR(maze_planner::parkingYaw(wall), 0.0, 0.02);
}

TEST(WallParking, RotationAndTranslationPreserveResult) {
  const double yaw = 0.7;
  const Point2 origin(1.2, -0.4);
  std::vector<Point2> scan;
  for (int i = -50; i <= 50; ++i)
    scan.push_back(origin + maze_planner::rotate(Point2(2.0, i * .01), yaw));
  const Point2 camera = origin + maze_planner::rotate(Point2(.145, 0), yaw);
  WallObservation wall;
  ASSERT_TRUE(maze_planner::fitTargetWall(scan, camera, yaw, &wall));
  const Point2 expected = origin + maze_planner::rotate(Point2(2, 0), yaw);
  EXPECT_NEAR(wall.marker.x, expected.x, .01);
  EXPECT_NEAR(wall.marker.y, expected.y, .01);
  EXPECT_NEAR(maze_planner::parkingYaw(wall), yaw, .02);
}

TEST(WallParking, DoesNotTreatShortClusterAsWall) {
  std::vector<Point2> scan;
  for (int i = 0; i < 30; ++i) scan.emplace_back(2.0, i * .005);
  WallObservation wall;
  EXPECT_FALSE(maze_planner::fitTargetWall(scan, Point2(.145, 0), 0, &wall));
}

TEST(WallParking, ChecksEveryFootprintVertexInSquare) {
  WallObservation wall;
  wall.marker = Point2(0, 0);
  wall.outward = Point2(-1, 0);
  const std::vector<Point2> footprint = {
      Point2(.3483, .245), Point2(.3483, -.245),
      Point2(-.1517, -.245), Point2(-.1517, .245)};
  EXPECT_TRUE(maze_planner::footprintInsideFinish(wall, Point2(-.56, 0), 0, footprint));
  EXPECT_FALSE(maze_planner::footprintInsideFinish(wall, Point2(-.65, 0), 0, footprint));
  EXPECT_FALSE(maze_planner::footprintInsideFinish(wall, Point2(-.56, .20), 0, footprint));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
