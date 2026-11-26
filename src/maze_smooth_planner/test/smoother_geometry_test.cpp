#include <gtest/gtest.h>
#include <maze_smooth_planner/smoother.h>
#include <maze_planner/corridor_explorer.h>
#include <chrono>
#include <costmap_2d/cost_values.h>
#include <cmath>
#include <fstream>
using namespace maze_smooth_planner;
int main(int argc,char** argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
std::vector<geometry_msgs::Point> body() {
  std::vector<geometry_msgs::Point> p(4);
  p[0].x=p[1].x=.3483;p[2].x=p[3].x=-.1517;
  p[0].y=p[3].y=.24;p[1].y=p[2].y=-.24;return p;
}
costmap_2d::Costmap2D freeMap(){return costmap_2d::Costmap2D(240,240,.025,-3,-3,0);}
TEST(Collision,OccupiedInteriorIsBlocked) {
  auto map=freeMap();unsigned int x,y;map.worldToMap(.10,0,x,y);map.setCost(x,y,254);
  EXPECT_FALSE(poseFree({0,0,0},map,body(),0));
}
TEST(Collision,UnknownAndBoundaryAreBlocked) {
  auto map=freeMap();unsigned int x,y;map.worldToMap(.1,0,x,y);map.setCost(x,y,255);
  EXPECT_FALSE(poseFree({0,0,0},map,body(),0));
  EXPECT_FALSE(poseFree({2.9,0,0},map,body(),0));
}
TEST(Collision,InflationIsNotDoubleCountedAsWall) {
  auto map=freeMap();unsigned int x,y;map.worldToMap(.1,0,x,y);map.setCost(x,y,253);
  EXPECT_TRUE(poseFree({0,0,0},map,body(),0));
}
TEST(Collision,RotationalSweepBetweenFreeEndpointsIsBlocked) {
  auto map=freeMap();unsigned int x,y;map.worldToMap(.30,.26,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(poseFree({0,0,0},map,body(),0));
  EXPECT_TRUE(poseFree({0,0,M_PI/2},map,body(),0));
  EXPECT_FALSE(pathFree({{0,0,0},{0,0,M_PI/2}},map,body(),0));
}
TEST(Smoothing,OpenCornerHasContinuousTangentAndPreservedEndpoints) {
  auto map=freeMap();std::vector<Pose> out;Statistics s;Config c;
  ASSERT_TRUE(smooth({{-1,0,0},{0,0,0},{0,1,M_PI/2}},map,body(),c,out,s));
  ASSERT_GT(s.rounded_corners,0);ASSERT_FALSE(s.fallback);ASSERT_GT(out.size(),20u);
  EXPECT_DOUBLE_EQ(out.front().x,-1);EXPECT_DOUBLE_EQ(out.back().y,1);
  double max_turn=0;
  for(size_t i=1;i<out.size();++i)max_turn=std::max(max_turn,std::abs(std::atan2(std::sin(out[i].yaw-out[i-1].yaw),std::cos(out[i].yaw-out[i-1].yaw))));
  EXPECT_LT(max_turn,.2);EXPECT_TRUE(pathFree(out,map,body(),c.collision_padding));
}
TEST(Smoothing,BlockedCutInRetainsCollisionFreeRoute) {
  auto map=freeMap();
  for(unsigned int y=0;y<map.getSizeInCellsY();++y)for(unsigned int x=0;x<map.getSizeInCellsX();++x){double wx,wy;map.mapToWorld(x,y,wx,wy);if(wx<-.32&&wy>.32)map.setCost(x,y,254);}
  std::vector<Pose> out;Statistics s;Config c;c.preferred_radius=.8;
  ASSERT_TRUE(smooth({{-1,0,0},{0,0,M_PI/2},{0,1,M_PI/2}},map,body(),c,out,s));
  EXPECT_TRUE(pathFree(out,map,body(),c.collision_padding));
  std::vector<Pose> open;Statistics open_stats;auto empty=freeMap();
  ASSERT_TRUE(smooth({{-1,0,0},{0,0,M_PI/2},{0,1,M_PI/2}},empty,body(),c,open,open_stats));
  auto closest=[](const std::vector<Pose>& p){double d=100;for(const auto& v:p)d=std::min(d,std::hypot(v.x,v.y));return d;};
  EXPECT_LT(closest(out),closest(open)*.9);
}
TEST(Smoothing,InfeasibleReferenceReturnsNoUnsafeFallback) {
  auto map=freeMap();unsigned int x,y;map.worldToMap(0,0,x,y);map.setCost(x,y,254);
  std::vector<Pose> out;Statistics s;Config c;
  EXPECT_FALSE(smooth({{-1,0,0},{1,0,0}},map,body(),c,out,s));EXPECT_TRUE(out.empty());
}
TEST(Smoothing,CenterSafeRouteMovesAwayFromBodyCollision) {
  auto map=freeMap();
  for(unsigned int y=0;y<map.getSizeInCellsY();++y)for(unsigned int x=0;x<map.getSizeInCellsX();++x){double wx,wy;map.mapToWorld(x,y,wx,wy);if(std::abs(wx)<.1&&wy>.20&&wy<.40)map.setCost(x,y,254);}
  std::vector<Pose> out;Statistics s;Config c;
  ASSERT_TRUE(smooth({{-1,0,0},{0,0,0},{1,0,0}},map,body(),c,out,s));
  EXPECT_GT(s.repair_steps,0);EXPECT_FALSE(s.fallback);
  EXPECT_TRUE(pathFree(out,map,body(),c.collision_padding));
}
TEST(Smoothing,DuplicatesAreRemovedWithoutNonfiniteOutput) {
  auto map=freeMap();std::vector<Pose> out;Statistics s;Config c;
  ASSERT_TRUE(smooth({{-1,0,0},{-1,0,0},{0,0,0},{0,0,0},{1,0,0}},map,body(),c,out,s));
  for(const auto& p:out)EXPECT_TRUE(std::isfinite(p.x+p.y+p.yaw));
}
TEST(Smoothing,ExactGoalAfterGridOvershootDoesNotCreateReverseTangent) {
  auto map=freeMap();std::vector<Pose> out;Statistics s;Config c;
  ASSERT_TRUE(smooth({{-1,0,0},{0,0,0},{1.025,0,0},{1,0,0}},map,body(),c,out,s));
  EXPECT_DOUBLE_EQ(out.back().x,1);
  for(size_t i=1;i<out.size();++i){EXPECT_GE(out[i].x-out[i-1].x,-1e-8);EXPECT_NEAR(out[i].yaw,0,1e-8);}
}
TEST(Smoothing,GoalApproachNearUnknownBoundaryKeepsValidatedFallback) {
  auto map=freeMap();
  for(unsigned int y=0;y<map.getSizeInCellsY();++y)for(unsigned int x=0;x<map.getSizeInCellsX();++x){double wx,wy;map.mapToWorld(x,y,wx,wy);if(wy<-.30)map.setCost(x,y,255);}
  std::vector<Pose> out;Statistics s;Config c;
  ASSERT_TRUE(smooth({{1,1,-M_PI/2},{.2,.5,-M_PI/2},{0,0,M_PI}},map,body(),c,out,s));
  ASSERT_GE(out.size(),2u);
  EXPECT_DOUBLE_EQ(out.back().yaw,M_PI);
  EXPECT_TRUE(pathFree(out,map,body(),c.collision_padding));
}
TEST(Smoothing,GoalTangentIsRespectedByContinuousOpenApproach) {
  auto map=freeMap();std::vector<Pose> out;Statistics s;Config c;
  ASSERT_TRUE(smooth({{1,1,-M_PI/2},{.2,.5,-M_PI/2},{0,0,M_PI}},map,body(),c,out,s));
  ASSERT_GT(out.size(),20u);EXPECT_FALSE(s.fallback);
  EXPECT_NEAR(std::sin(out[out.size()-2].yaw-M_PI),0,.02);
  EXPECT_TRUE(pathFree(out,map,body(),c.collision_padding));
}

TEST(FootprintSearch,RecordedTurnFindsDifferentSweptSafeRoute) {
  std::ifstream f(std::string(MAZE_PLANNER_TEST_DATA)+"/blocked_turn_20261014.txt");
  ASSERT_TRUE(f.good());
  unsigned int w,h;double res,ox,oy;f>>w>>h>>res>>ox>>oy;
  costmap_2d::Costmap2D map(w,h,res,ox,oy,0);
  for(unsigned int i=0;i<w*h;++i){int value;f>>value;map.getCharMap()[i]=value;}
  size_t n;f>>n;std::vector<geometry_msgs::Point> outline(n);
  for(auto& p:outline)f>>p.x>>p.y;
  Pose start,goal;f>>start.x>>start.y>>start.yaw>>goal.x>>goal.y>>goal.yaw;
  f>>n;std::vector<Pose> old(n);for(auto& p:old)f>>p.x>>p.y>>p.yaw;
  ASSERT_TRUE(f.good());ASSERT_FALSE(pathFree(old,map,outline,.02));
  Config cfg;cfg.replan_compute_ms=1500;std::vector<Pose> out;
  ASSERT_TRUE(footprintReplan(start,goal,map,outline,cfg,out));
  ASSERT_GT(out.size(),10u);ASSERT_TRUE(pathFree(out,map,outline,.02));
  EXPECT_DOUBLE_EQ(out.front().x,start.x);EXPECT_DOUBLE_EQ(out.front().yaw,start.yaw);
  EXPECT_DOUBLE_EQ(out.back().x,goal.x);EXPECT_DOUBLE_EQ(out.back().yaw,goal.yaw);
  // All interior translation primitives move forward along their heading.
  // The exact start/goal connectors are independently sweep checked above.
  for(size_t i=2;i+1<out.size();++i){
    double dx=out[i].x-out[i-1].x,dy=out[i].y-out[i-1].y;
    if(std::hypot(dx,dy)<1e-8)continue;
    EXPECT_GT(dx*std::cos(out[i-1].yaw)+dy*std::sin(out[i-1].yaw),0);
    EXPECT_NEAR(dx*std::sin(out[i-1].yaw)-dy*std::cos(out[i-1].yaw),0,1e-8);
  }
}

TEST(FootprintSearch,UnknownBarrierNeverProducesUnsafePath) {
  auto map=freeMap();
  for(unsigned int y=0;y<map.getSizeInCellsY();++y)
    for(unsigned int x=115;x<125;++x)map.setCost(x,y,255);
  Config cfg;cfg.replan_compute_ms=30;std::vector<Pose> out;
  EXPECT_FALSE(footprintReplan({-1,0,0},{1,0,0},map,body(),cfg,out));
  EXPECT_TRUE(out.empty());
}

TEST(FootprintSearch,BlockedStartAndInvalidBudgetAreRejected) {
  auto map=freeMap();Config cfg;std::vector<Pose> out;
  EXPECT_FALSE(footprintReplan({2.9,0,0},{0,0,0},map,body(),cfg,out));
  cfg.replan_compute_ms=0;
  EXPECT_FALSE(footprintReplan({-1,0,0},{1,0,0},map,body(),cfg,out));
  EXPECT_TRUE(out.empty());
}

namespace {

// attempt-02/navigation_0.bag. Each fixture stores the latest preceding
// published costmap, the raw path and the nearest AMCL pose to anchor its start.
// For t100 the preceding full costmap publication was at t=92.73 s.
bool loadMockSnapshot(const std::string& name, nav_msgs::OccupancyGrid* grid,
                      costmap_2d::Costmap2D* map, std::vector<geometry_msgs::Point>* footprint,
                      std::vector<Pose>* raw) {
  std::ifstream f(std::string(MAZE_PLANNER_TEST_DATA) + "/" + name + ".txt");
  auto& info = grid->info;
  auto& origin = info.origin;
  std::size_t runs;
  f >> info.width >> info.height >> info.resolution >> origin.position.x >> origin.position.y >> runs;
  if (!f) return false;
  origin.orientation.w = 1.0;
  map->resizeMap(info.width, info.height, info.resolution, origin.position.x, origin.position.y);
  grid->data.reserve(info.width * info.height);
  for (std::size_t i = 0; i < runs; ++i) {
    int occupancy;
    unsigned int count;
    if (!(f >> occupancy >> count) || grid->data.size() + count > info.width * info.height)
      return false;
    // Invert the published costmap_2d translation table.
    const int cost = occupancy < 0 ? 255 : occupancy == 100 ? 254 :
        occupancy == 99 ? 253 : occupancy == 0 ? 0 :
        1 + static_cast<int>(std::lround((occupancy - 1) * 251.0 / 97));
    std::fill_n(map->getCharMap() + grid->data.size(), count, cost);
    grid->data.insert(grid->data.end(), count, occupancy);
  }
  if (grid->data.size() != info.width * info.height) return false;
  std::size_t count;
  if (!(f >> count)) return false;
  footprint->resize(count);
  for (auto& p : *footprint) f >> p.x >> p.y;
  if (!(f >> count)) return false;
  raw->resize(count);
  for (auto& p : *raw) f >> p.x >> p.y >> p.yaw;
  return f.good() && !raw->empty() && footprint->size() >= 3;
}

}  // namespace

TEST(Collision, CorridorEntranceAgreesWithPlannerOnRecordedGoal9) {
  nav_msgs::OccupancyGrid occupancy;
  costmap_2d::Costmap2D map;
  std::vector<geometry_msgs::Point> footprint;
  std::vector<Pose> raw;
  ASSERT_TRUE(loadMockSnapshot("mock1126_goal9", &occupancy, &map, &footprint, &raw));
  const maze_planner::CorridorGrid grid(occupancy);
  std::vector<maze_planner::Point2> outline;
  for (const auto& p : footprint) outline.emplace_back(p.x, p.y);
  const auto& goal = raw.back();
  EXPECT_NEAR(goal.x, 2.869, .001);
  EXPECT_NEAR(goal.y, .054, .001);
  EXPECT_TRUE(grid.footprintClear({goal.x, goal.y}, goal.yaw, outline));
  EXPECT_FALSE(grid.footprintClear({goal.x, goal.y}, goal.yaw, outline, .025));
  EXPECT_FALSE(poseFree(goal, map, footprint, .02));
  int checked_clear = 0;
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      for (double yaw : {goal.yaw - .20, goal.yaw, goal.yaw + .20}) {
        const Pose sample{goal.x + .02 * x, goal.y + .02 * y, yaw};
        if (!grid.footprintClear({sample.x, sample.y}, yaw, outline, .025)) continue;
        ++checked_clear;
        EXPECT_TRUE(poseFree(sample, map, footprint, .02));
      }
    }
  }
  EXPECT_GT(checked_clear, 0);
}

TEST(FootprintSearch, MockRaceNarrowPassageSnapshotsRespectPlanningBudget) {
  for (const auto* name : {"mock1126_t100", "mock1126_t110"}) {
    SCOPED_TRACE(name);
    nav_msgs::OccupancyGrid occupancy;
    costmap_2d::Costmap2D map;
    std::vector<geometry_msgs::Point> footprint;
    std::vector<Pose> raw, out;
    ASSERT_TRUE(loadMockSnapshot(name, &occupancy, &map, &footprint, &raw));
    Config config;
    config.max_compute_ms = 50.0;
    config.replan_compute_ms = 120.0;
    config.max_lateral_adjustment = .22;
    Statistics stats;
    const auto start = std::chrono::steady_clock::now();
    const bool found = smooth(raw, map, footprint, config, out, stats) ||
        footprintReplan(raw.front(), raw.back(), map, footprint, config, out);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << name << ": " << ms << " ms, path_found=" << found << '\n';
    if (found) {
      EXPECT_LT(ms, 150.0);
      EXPECT_TRUE(pathFree(out, map, footprint, config.collision_padding));
    } else {
      EXPECT_LT(ms, 170.0);
      EXPECT_TRUE(out.empty());
    }
  }
}
