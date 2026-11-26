#include <gtest/gtest.h>
#include <maze_planner/corridor_explorer.h>
#include <maze_planner/finish_target_verification.h>

namespace {

nav_msgs::OccupancyGrid room(bool fork = false) {
  nav_msgs::OccupancyGrid map;
  map.info.width = 160;
  map.info.height = 100;
  map.info.resolution = .05;
  map.info.origin.orientation.w = 1.0;
  for (int y = 0; y < 100; ++y) {
    for (int x = 0; x < 160; ++x) {
      const bool corridor = x >= 8 && x < 142 && y >= 37 && y < 63;
      const bool branch = fork && x >= 110 && x < 136 && y >= 10 && y < 94;
      map.data.push_back(corridor || branch ? 0 : 100);
    }
  }
  return map;
}

const std::vector<maze_planner::Point2> rectangle{
    {.3483, .245}, {.3483, -.245}, {-.1517, -.245}, {-.1517, .245}};

}  // namespace

TEST(CorridorExplorer, SelectsTheDeepReachablePointInOneCorridor) {
  const maze_planner::CorridorGrid grid(room());
  maze_planner::CorridorExplorer explorer;
  const maze_planner::Point2 start(1.0, 2.50);
  explorer.reset(start);
  maze_planner::Point2 goal;
  double yaw = 0;
  ASSERT_TRUE(explorer.select(grid, start, &goal, &yaw));
  EXPECT_GT(goal.x, 6.3);
  EXPECT_NEAR(goal.y, 2.5, .15);
  EXPECT_NEAR(yaw, 0.0, .15);
  EXPECT_TRUE(grid.clear(goal, .38, 95));
}

TEST(CorridorExplorer, ReplacesFrontierAsSoonAsTheCostmapBlocksIt) {
  maze_planner::CorridorGrid grid(room());
  maze_planner::CorridorExplorer explorer;
  const maze_planner::Point2 start(1.0, 2.50);
  explorer.reset(start);
  maze_planner::Point2 original, replacement;
  double yaw = 0;
  ASSERT_TRUE(explorer.select(grid, start, &original, &yaw));
  const int gx = grid.cell(original) % grid.width;
  for (int y = 37; y < 63; ++y)
    for (int x = gx - 8; x < gx + 1; ++x)
      grid.cost[grid.index(x, y)] = 100;
  EXPECT_FALSE(grid.clear(original, .30, 95));
  ASSERT_TRUE(explorer.select(grid, start, &replacement, &yaw));
  EXPECT_LT(replacement.x, original.x - .5);
}

TEST(CorridorExplorer, VisitsTheOtherSideOfTheForkAndClearsVisitsOnReset) {
  const maze_planner::CorridorGrid grid(room(true));
  maze_planner::CorridorExplorer explorer;
  const maze_planner::Point2 start(1.0, 2.5);
  explorer.reset(start);
  maze_planner::Point2 first, second, next_round;
  double yaw = 0;
  ASSERT_TRUE(explorer.select(grid, start, &first, &yaw));
  explorer.visit(first);
  ASSERT_TRUE(explorer.select(grid, first, &second, &yaw));
  EXPECT_GT(first.y, 3.6);
  EXPECT_LT(second.y, 1.5);
  explorer.reset(start);
  ASSERT_TRUE(explorer.select(grid, start, &next_round, &yaw));
  EXPECT_LT(maze_planner::norm(first - next_round), .05);
}

TEST(FinishEntrance, AllowsCornerWithSoftInflationAndRejectsLethalFootprintCells) {
  auto map = room();
  for (int y = 37; y < 45; ++y)
    for (int x = 8; x < 142; ++x)
      map.data[y * map.info.width + x] = 90;
  maze_planner::CorridorGrid grid(map);
  const maze_planner::WallObservation wall{{7.10, 2.25}, {-1, 0}};
  maze_planner::Point2 goal;
  ASSERT_TRUE(maze_planner::selectFinishEntrance(grid, wall, rectangle, {}, &goal));
  EXPECT_NEAR(goal.x, 6.30, 1e-6);
  EXPECT_NEAR(goal.y, 2.25, 1e-6);
  grid.cost[grid.cell({goal.x + .30, goal.y})] = 100;
  EXPECT_FALSE(grid.footprintClear(goal, 0.0, rectangle));
  ASSERT_TRUE(maze_planner::selectFinishEntrance(grid, wall, rectangle, {}, &goal));
  EXPECT_GT(maze_planner::norm(goal - maze_planner::Point2(6.30, 2.25)), .1);
}

TEST(FinishEntrance, AvoidsGoalsCloseBehindTheRobotDuringReselection) {
  const maze_planner::CorridorGrid grid(room());
  const maze_planner::WallObservation wall{{7.15, 2.50}, {-1, 0}};
  const maze_planner::Point2 robot(6.60, 2.50);
  maze_planner::Point2 goal;
  ASSERT_TRUE(maze_planner::selectFinishEntrance(grid, wall, rectangle, {}, &goal,
                                                .025, &robot, 0));
  const auto offset = goal - robot;
  const double distance = maze_planner::norm(offset);
  EXPECT_FALSE(distance < .55 && offset.x < -.5 * distance);
}

TEST(FinishEntrance, SelectFinishEntranceRejectsDisconnectedPocketAndStaticUnknown) {
  auto map = room();
  const maze_planner::WallObservation wall{{7.10,2.5},{-1,0}};
  for (int y = 37; y < 63; ++y) map.data[y*map.info.width+120] = 99;
  maze_planner::CorridorGrid grid(map);
  const maze_planner::Point2 robot{2,2.5};
  maze_planner::Point2 entrance;
  grid.cost[grid.cell(robot)] = 99;
  auto static_map = room();
  const int pocket = grid.cell({6.30,2.5});
  static_map.data[pocket] = -1;
  ASSERT_TRUE(grid.footprintClear({6.30,2.5},0,rectangle));
  EXPECT_FALSE(maze_planner::reachableMask(grid,robot)[pocket]);
  EXPECT_EQ(static_map.data[pocket],-1);
  ASSERT_TRUE(maze_planner::selectFinishEntrance(
      grid,wall,rectangle,{},&entrance,.02,&robot,0,false,&static_map,true));
  EXPECT_NEAR(entrance.x,5.95,1e-6);
  EXPECT_EQ(static_map.data[grid.cell(entrance)],0);
  const auto clear = maze_planner::CorridorGrid(room());
  ASSERT_TRUE(maze_planner::selectFinishEntrance(
      clear,wall,rectangle,{},&entrance,.02,&robot,0,false,&static_map,true));
  EXPECT_GT(maze_planner::norm(entrance-maze_planner::Point2(6.30,2.5)),.1);
}

TEST(FinishVerification, SuppressesOnlyAfterTwoBlindEntranceScans) {
  using namespace maze_planner;
  using Result = FinishTargetVerification::Result;
  FinishTargetVerification verify;
  const Point2 target(2.87, .05);
  FalseFinishTargets blacklist;
  verify.seen(ros::Time(10.0));
  verify.arrive(ros::Time(10.5));
  EXPECT_EQ(Result::Waiting, verify.result(ros::Time(13.5), 2.6));
  EXPECT_EQ(Result::Missing, verify.result(ros::Time(14.0), 2.6));
  ASSERT_TRUE(verify.needsAlternateView());
  verify.arrive(ros::Time(15.0));
  EXPECT_EQ(Result::Waiting, verify.result(ros::Time(17.5), 2.6));
  const ros::Time rejected_at(17.7);
  ASSERT_EQ(Result::FalseTarget, verify.result(rejected_at, 2.6));
  blacklist.add(target, rejected_at);
  EXPECT_TRUE(blacklist.blocks({target.x + .8, target.y}, .002, rejected_at));
  EXPECT_FALSE(blacklist.blocks({target.x + .91, target.y}, .002, rejected_at));
  EXPECT_FALSE(blacklist.blocks(target, static_cast<float>(.03), rejected_at));
  EXPECT_FALSE(blacklist.blocks(target, .002, ros::Time(62.7)));
}

TEST(FinishVerification, ValidSeenTargetDoesNotCountAsAFalseMarker) {
  using namespace maze_planner;
  using Result = FinishTargetVerification::Result;
  FinishTargetVerification verify;
  verify.arrive(ros::Time(10.0));
  ASSERT_EQ(Result::Missing, verify.result(ros::Time(12.7), 2.6));
  verify.arrive(ros::Time(14.0));
  verify.seen(ros::Time(14.2));
  EXPECT_EQ(Result::Seen, verify.result(ros::Time(16.7), 2.6));
  EXPECT_FALSE(verify.needsAlternateView());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
