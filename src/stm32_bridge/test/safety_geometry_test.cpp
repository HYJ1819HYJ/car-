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

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
