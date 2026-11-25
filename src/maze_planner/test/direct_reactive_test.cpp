#include <gtest/gtest.h>
#include <maze_planner/direct_reactive_policy.h>

#include <cmath>

namespace {
maze_planner::ReactiveInput visible(double bearing, double area) {
  maze_planner::ReactiveInput input;
  input.target_visible = true;
  input.bearing = bearing;
  input.contour_ratio = area;
  input.target_front_distance = 2.0;
  input.footprint.padding = .03;
  return input;
}
}

TEST(DirectReactive, TurnsTowardTargetAndOnlyStopsWhenBothConditionsHold) {
  maze_planner::ReactiveParameters p;
  auto input = visible(-.8, .02);
  auto command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_LT(command.w, 0.0);  // Target on the robot's right.
  input = visible(.03, .159);
  command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_GT(command.v, 0.0);
  EXPECT_GT(command.w, 0.0);
  input = visible(.12, .16);
  command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_FALSE(command.complete);
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_GT(command.w, 0.0);
  input.bearing = .03;
  command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_TRUE(command.complete);
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_DOUBLE_EQ(command.w, 0.0);
}

TEST(DirectReactive, AvoidsCloserObstacleButApproachesTheTargetWall) {
  maze_planner::ReactiveParameters p;
  auto input = visible(0.0, .05);
  input.front_clearance = .35;
  input.obstacles = {{.70, .12}, {.70, .35}};
  auto command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_STREQ(command.state, "AVOID");
  EXPECT_LT(command.w, 0.0);  // Prefer the open right side.

  input.front_clearance = .13;
  input.target_front_distance = .20;  // The near surface is the target wall.
  input.obstacles = {{.478, 0.0}};
  command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_STREQ(command.state, "APPROACH");
  EXPECT_GT(command.v, 0.0);  // Existing serial bridge enforces the 12 cm stop.
}

TEST(DirectReactive, SearchesWhenTheTargetIsAbsent) {
  maze_planner::ReactiveParameters p;
  maze_planner::ReactiveInput input;
  auto command = maze_planner::reactiveVisualCommand(input, p, 0, -1);
  EXPECT_STREQ(command.state, "SEARCH");
  EXPECT_DOUBLE_EQ(command.v, 0.0);
  EXPECT_LT(command.w, 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
