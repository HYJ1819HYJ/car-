#include <gtest/gtest.h>
#include <maze_planner/search_waypoint_order.h>

#include <vector>

struct Point { double x, y; };

TEST(SearchWaypointOrder, PrefersForwardOverCloserBehindAndSideways) {
  const std::vector<Point> points{{-1.0, 0.0}, {0.0, 0.8}, {3.0, 0.2}};
  EXPECT_EQ(2u, maze_planner::searchStartIndex(points, 0.0, 0.0, 0.0, true));
  EXPECT_EQ(1u, maze_planner::searchStartIndex(points, 0.0, 0.0, 0.0, false));
}

TEST(SearchWaypointOrder, UsesHeadingAndFallsBackWhenNoForwardPointExists) {
  const std::vector<Point> points{{-1.0, 0.0}, {0.0, 2.0}, {0.0, -0.5}};
  EXPECT_EQ(1u, maze_planner::searchStartIndex(points, 0.0, 0.0, 1.57079632679, true));
  EXPECT_EQ(2u, maze_planner::searchStartIndex(points, 0.0, 0.0, 0.0, true));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
