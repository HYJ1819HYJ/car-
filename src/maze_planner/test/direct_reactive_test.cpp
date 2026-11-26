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

TEST(DirectReactive, SteersTowardWallNormalCenterlineAndTapersSpeedNearStop) {
  maze_planner::ReactiveParameters p;
  p.max_speed = 0.30;
  p.min_approach_speed = 0.10;
  p.max_turn = 0.50;

  // Marker is at x=2.0, y=-0.65 (right of robot), wall outward normal is (-1, 0).
  // Direct bearing to marker is atan2(-0.65, 2.0) = -0.314 rad, whereas the lead
  // point on the wall normal (1.20, -0.65) has bearing atan2(-0.65, 1.20) = -0.496 rad,
  // steering more strongly onto the 0.8 m square's centerline before final approach.
  auto far_offset = visible(std::atan2(-0.65, 2.0), 0.02);
  far_offset.wall_valid = true;
  far_offset.wall_marker = maze_planner::Point2(2.0, -0.65);
  far_offset.wall_outward = maze_planner::Point2(-1.0, 0.0);
  auto cmd_align = maze_planner::reactiveVisualCommand(far_offset, p, 0, -1);
  EXPECT_LT(cmd_align.w, -0.40);

  // Centered far away -> full boosted cruise speed (0.30 m/s).
  auto far_centered = visible(0.0, 0.02);
  far_centered.front_clearance = 1.80;
  auto cmd_fast = maze_planner::reactiveVisualCommand(far_centered, p, 0, 1);
  EXPECT_NEAR(cmd_fast.v, 0.30, 1e-6);

  // Close to 16% stop threshold -> smoothly tapered toward 0.10 m/s soft landing.
  auto near_stop = visible(0.0, 0.15);
  near_stop.front_clearance = 0.24;
  auto cmd_slow = maze_planner::reactiveVisualCommand(near_stop, p, 0, 1);
  EXPECT_GT(cmd_slow.v, 0.09);
  EXPECT_LT(cmd_slow.v, 0.15);
}

TEST(ParkingStop, RejectsLargeContourOutsideFinishAndStopsWithSmallContourInside) {
  maze_planner::ReactiveParameters p;
  auto input = visible(0.0, .35);
  input.wall_valid = true;
  input.wall_outward = {-1.0, 0.0};
  input.wall_marker = {.85, 0.0};
  EXPECT_FALSE(maze_planner::parkingStopReached(input, p));
  EXPECT_GT(maze_planner::reactiveVisualCommand(input, p, 0, 1).v, 0.0);
  input.wall_marker = {.55, 0.0};
  input.contour_ratio = .09;
  EXPECT_TRUE(maze_planner::parkingStopReached(input, p));
  input.target_visible = false;
  EXPECT_TRUE(maze_planner::reactiveVisualCommand(input, p, 0, 1).complete);
}

TEST(ParkingStop, ChecksRotatedFootprintBeforeAcceptingMetricFallback) {
  maze_planner::ReactiveParameters p;
  auto input = visible(.02, .30);
  input.wall_valid = true;
  input.wall_outward = maze_planner::rotate({-1.0, 0.0}, .13);
  input.wall_marker = maze_planner::rotate({.55, .138}, .13);
  EXPECT_FALSE(maze_planner::parkingStopReached(input, p));
  input.wall_valid = false;
  EXPECT_TRUE(maze_planner::parkingStopReached(input, p));
}

TEST(DirectReactive, AlignChecksRequestedTurnWithoutSweepingPastIt) {
  maze_planner::ReactiveParameters p;
  p.terminal_only = true;
  p.max_turn = .50;
  p.turn_gain = 4.0;
  auto input = visible(.18, .16);
  input.obstacles = {{.15, .35}};
  EXPECT_FALSE(maze_planner::reactivePathClear(input, 0.0, .50, 1.0));
  auto command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_STREQ(command.state, "ALIGN");
  EXPECT_DOUBLE_EQ(command.w, .50);

  input.obstacles.push_back({.32, .29});
  command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_STREQ(command.state, "BLOCKED");
  EXPECT_DOUBLE_EQ(command.w, 0.0);
}

TEST(DirectReactive, ApproachSweepsToStopDepthAndStillBlocksNearbyObstacles) {
  maze_planner::ReactiveParameters p;
  p.terminal_only = true;
  p.turn_gain = 0.0;
  auto input = visible(.01, .09);
  input.wall_valid = true;
  input.wall_marker = {.485, .18};
  input.wall_outward = {-1.0, 0.0};
  input.target_front_distance = .137;
  input.obstacles = {{.395, 0.0}};
  EXPECT_FALSE(maze_planner::reactivePathClear(input, .04, 0.0, .60));
  auto command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_STREQ(command.state, "APPROACH");
  EXPECT_GT(command.v, 0.0);

  input.obstacles.push_back({.381, 0.0});
  command = maze_planner::reactiveVisualCommand(input, p, 0, 1);
  EXPECT_STREQ(command.state, "BLOCKED");
  EXPECT_DOUBLE_EQ(command.v, 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
