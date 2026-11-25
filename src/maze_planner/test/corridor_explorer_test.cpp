#include <gtest/gtest.h>
#include <maze_planner/corridor_explorer.h>

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
