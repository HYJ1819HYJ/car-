#include <gtest/gtest.h>
#include <maze_smooth_planner/smoother.h>
#include <costmap_2d/cost_values.h>
#include <cmath>
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
