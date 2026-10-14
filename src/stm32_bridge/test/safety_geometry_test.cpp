#include <gtest/gtest.h>

#include <vector>

#include "stm32_bridge/safety_geometry.hpp"

namespace {

using stm32_bridge::SafetyFootprint;
using stm32_bridge::SafetyPoint;
using stm32_bridge::trajectoryIntersects;

const SafetyFootprint kCar{0.3483, -0.1517, 0.24, 0.03};

TEST(SafetyGeometry, StraightMotionUsesFullVehicleWidth) {
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{0.55, 0.20}}, kCar,
                                   0.50, 0.0, 0.60, 0.02));
  EXPECT_FALSE(trajectoryIntersects({SafetyPoint{0.55, 0.40}}, kCar,
                                    0.50, 0.0, 0.60, 0.02));
}

TEST(SafetyGeometry, ReverseMotionChecksRearOfVehicle) {
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{-0.30, 0.0}}, kCar,
                                   -0.25, 0.0, 0.60, 0.02));
}

TEST(SafetyGeometry, ExactTurnAllowsPointOutsideSweptRectangle) {
  // The old radius-only guard rejected this point because radius < 0.45 m.
  // During this left turn it never enters the asymmetric rectangle.
  EXPECT_FALSE(trajectoryIntersects({SafetyPoint{0.40, 0.15}}, kCar,
                                    0.0, 0.75, 0.60, 0.01));
}

TEST(SafetyGeometry, ExactTurnRejectsCornerSweepCollision) {
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{0.30, 0.30}}, kCar,
                                   0.0, 0.75, 0.60, 0.01));
}

TEST(SafetyGeometry, CurvedForwardTrajectoryIsChecked) {
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{0.42, 0.25}}, kCar,
                                   0.30, 0.75, 0.80, 0.01));
}

SafetyFootprint polygonCar() {
  SafetyFootprint body;
  body.padding=0.0;
  body.vertices={{-.15,-.24},{.20,-.24},{.35,0.0},{.20,.24},{-.15,.24}};
  EXPECT_TRUE(stm32_bridge::normalizeConvexFootprint(&body.vertices));
  return body;
}

TEST(SafetyGeometryPolygon, RoundedFrontDoesNotRetainBoundingRectangleCorner) {
  auto body=polygonCar();
  const SafetyPoint corner{.32,.20};
  EXPECT_TRUE(trajectoryIntersects({corner},kCar,0.,0.,.6,.02));
  EXPECT_FALSE(trajectoryIntersects({corner},body,0.,0.,.6,.02));
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{.30,.02}},body,0.,0.,.6,.02));
}

TEST(SafetyGeometryPolygon, PaddingUsesDistanceToActualBevel) {
  auto body=polygonCar();
  // Edge x=.35-.625*y, at y=.16 x=.25. Shift 10mm outward along its normal.
  const double n=std::hypot(1.,.625);
  const SafetyPoint obstacle{.25+.01/n,.16+.00625/n};
  body.padding=.009;
  EXPECT_FALSE(trajectoryIntersects({obstacle},body,0.,0.,.6,.02));
  body.padding=.011;
  EXPECT_TRUE(trajectoryIntersects({obstacle},body,0.,0.,.6,.02));
}

TEST(SafetyGeometryPolygon, DirectionalClearanceUsesActualRoundedFront) {
  auto body=polygonCar();
  double rear,front;
  stm32_bridge::longitudinalBodyLimits(body,.16,&rear,&front);
  EXPECT_NEAR(front,.25,1e-12);
  EXPECT_NEAR(rear,-.15,1e-12);
  EXPECT_NEAR(.37-front,.12,1e-12);
  stm32_bridge::longitudinalBodyLimits(body,0.,&rear,&front);
  EXPECT_NEAR(front,.35,1e-12);
  stm32_bridge::longitudinalBodyLimits(kCar,.16,&rear,&front);
  EXPECT_NEAR(front,kCar.front,1e-12);
}

TEST(SafetyGeometryPolygon, TurnChecksIntermediateSweptBody) {
  auto body=polygonCar();
  const double yaw=.4;
  const SafetyPoint obstacle{.34*std::cos(yaw),.34*std::sin(yaw)};
  EXPECT_FALSE(stm32_bridge::pointInsideFootprint(obstacle,{0.,0.},0.,body));
  EXPECT_FALSE(stm32_bridge::pointInsideFootprint(obstacle,{0.,0.},.8,body));
  EXPECT_TRUE(trajectoryIntersects({obstacle},body,0.,1.,.8,.01));
}

TEST(SafetyGeometryPolygon, ReverseChecksRearPolygon) {
  auto body=polygonCar();
  EXPECT_FALSE(trajectoryIntersects({SafetyPoint{-.24,.10}},body,0.,0.,.6,.02));
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{-.24,.10}},body,-.2,0.,.6,.02));
}

TEST(SafetyGeometryPolygon, AcceptsClockwiseAndRepeatedClosingPoint) {
  auto body=polygonCar();
  std::reverse(body.vertices.begin(),body.vertices.end());
  body.vertices.push_back(body.vertices.front());
  EXPECT_TRUE(stm32_bridge::normalizeConvexFootprint(&body.vertices));
  EXPECT_EQ(body.vertices.size(),5u);
  EXPECT_TRUE(trajectoryIntersects({SafetyPoint{.30,.02}},body,0.,0.,.6,.02));
}

TEST(SafetyGeometryPolygon, RejectsUnsupportedOrMalformedBody) {
  std::vector<SafetyPoint> concave={{-.2,-.2},{.2,-.2},{0.,0.},{.2,.2},{-.2,.2}};
  EXPECT_FALSE(stm32_bridge::normalizeConvexFootprint(&concave));
  std::vector<SafetyPoint> duplicate={{-.2,-.2},{.2,-.2},{.2,-.2},{.2,.2},{-.2,.2}};
  EXPECT_FALSE(stm32_bridge::normalizeConvexFootprint(&duplicate));
  std::vector<SafetyPoint> line={{-.2,0.},{0.,0.},{.2,0.}};
  EXPECT_FALSE(stm32_bridge::normalizeConvexFootprint(&line));
  std::vector<SafetyPoint> outside={{1.,1.},{2.,1.},{2.,2.},{1.,2.}};
  EXPECT_FALSE(stm32_bridge::normalizeConvexFootprint(&outside));
  std::vector<SafetyPoint> nonfinite={{-.2,-.2},{.2,-.2},{0.,NAN}};
  EXPECT_FALSE(stm32_bridge::normalizeConvexFootprint(&nonfinite));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
