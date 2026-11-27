#include <gtest/gtest.h>

#include <stm32_bridge/gimbal_status.hpp>

#include <cstdio>

TEST(GimbalYawFeedforward, PredictsImageMotionBeforeMirroringAndClampsToFrame) {
  using stm32_bridge::applyYawFeedforwardPx;
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(320,400,.5,.12,120,640),344);
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(320,400,-.5,.12,120,640),296);
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(320,400,10,.12,120,640),440);
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(320,400,-10,.12,120,640),200);
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(620,400,10,.12,120,640),639);
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(20,400,-10,.12,120,640),0);
  EXPECT_DOUBLE_EQ(applyYawFeedforwardPx(320,400,.5,0,120,640),320);
}

TEST(GimbalSearchDirection, FavorsInsideOfTurnOnlyWhileSearching) {
  using stm32_bridge::desiredSearchDirectionForTurn;
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, -.22, 12.0, 1.0), -1);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, -.22, 12.0, -1.0), 1);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, .22, -12.0, 1.0), 1);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, .22, -12.0, -1.0), -1);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, .0, -20.0, -1.0), 0);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, -.21, 20.0, -1.0), 0);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, -.50, 11.9, -1.0), 0);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, true, -.50, -20.0, -1.0), 0);
  EXPECT_EQ(desiredSearchDirectionForTurn(true, false, -.50, 20.0, -1.0), 0);
  EXPECT_EQ(desiredSearchDirectionForTurn(false, true, -.50, 20.0, -1.0), 0);
}

using stm32_bridge::GimbalStartup;
using stm32_bridge::GimbalStatus;
using stm32_bridge::GimbalStatusParser;
using stm32_bridge::GimbalTargetCommand;
using stm32_bridge::GimbalTiltConfig;
using stm32_bridge::GimbalTiltController;

namespace {
const std::string kLegacy =
    "STAT,mode=0,ready=1,found=0,lock=0,search=1,x=559,y=294,rx=153,cmd=0,bearing10=-21,tilt10=-135";

std::string checked(const std::string& payload) {
  unsigned int checksum = 0;
  for (unsigned char ch : payload) checksum ^= ch;
  char suffix[4];
  std::snprintf(suffix, sizeof(suffix), "*%02X", checksum);
  return payload + suffix;
}
}

TEST(GimbalStatus, LegacyAndChecksumUpgrade) {
  GimbalStatusParser parser;
  GimbalStatus status{};
  ASSERT_TRUE(parser.parse(kLegacy, &status));
  EXPECT_FALSE(status.enabled);
  EXPECT_TRUE(status.ready);
  EXPECT_TRUE(status.searching);
  EXPECT_DOUBLE_EQ(status.pan_deg, -2.1);
  EXPECT_DOUBLE_EQ(status.tilt_deg, -13.5);
  EXPECT_FALSE(parser.parse(kLegacy + "*zz", &status));
  ASSERT_TRUE(parser.parse(checked(kLegacy), &status));
  EXPECT_FALSE(parser.parse(kLegacy, &status));

  GimbalStatusParser strict(true);
  EXPECT_FALSE(strict.parse(kLegacy, &status));
  EXPECT_TRUE(strict.parse(checked(kLegacy), &status));
}

TEST(GimbalStatus, RejectsBadFramesWithoutPartialUpdates) {
  GimbalStatusParser parser;
  GimbalStatus status{};
  ASSERT_TRUE(parser.parse(kLegacy, &status));
  const auto prefix = kLegacy.substr(0, kLegacy.find("bearing10="));
  for (const auto& broken : {kLegacy + "MCSTAT,mode=1", kLegacy + "\x80",
                            kLegacy + "*00", kLegacy + ",mode=1",
                            prefix + "bearing10=1801,tilt10=0",
                            prefix + "bearing10=0,tilt10=-901",
                            prefix + "bearing10=0", prefix + "bearing10=0,tilt10=1junk"}) {
    EXPECT_FALSE(parser.parse(broken, &status)) << broken;
    EXPECT_DOUBLE_EQ(status.tilt_deg, -13.5);
    EXPECT_FALSE(status.enabled);
  }
}

TEST(GimbalStartup, RecoversWhileTheSerialPortRemainsOpen) {
  using Action = GimbalStartup::Action;
  GimbalStartup startup;
  startup.reset(false);
  EXPECT_EQ(Action::None, startup.next(1.0, false, true, false, -13.5));
  EXPECT_EQ(Action::Center, startup.next(1.1, true, true, false, -13.5));
  EXPECT_EQ(Action::None, startup.next(1.8, true, true, false, -13.5));
  EXPECT_EQ(Action::Enable, startup.next(2.0, true, true, false, 0.0));
  EXPECT_EQ(Action::None, startup.next(3.0, true, true, false, 0.0));
  EXPECT_EQ(Action::None, startup.next(4.0, true, true, true, 0.0));
  EXPECT_EQ(Action::Enable, startup.next(5.0, true, true, false, 0.0));
  startup.requestCenter();
  EXPECT_EQ(Action::Center, startup.next(6.0, true, false, true, 0.0));
  EXPECT_EQ(Action::None, startup.next(7.0, true, false, true, 0.0));
  // With target_tilt_deg = -10.0, an already-pitched-down gimbal enables directly.
  EXPECT_EQ(Action::Enable, startup.next(9.0, true, true, false, -10.0, -10.0));
}

namespace {

// Deterministic simulation of the unmodified STM32 gimbal_tracker.c tilt loop
// (40 ms step period, KP=0.012, max step 1.5 deg, 12 px dead zone, 360 ms STAT).
class SimulatedMcuGimbal {
 public:
  explicit SimulatedMcuGimbal(double initial_tilt) : tilt_(initial_tilt) {}

  void receiveTarget(double now, const GimbalTargetCommand& cmd) {
    if (cmd.found) {
      found_ = true;
      target_x_ = cmd.x;
      target_y_ = cmd.y;
      last_rx_ = now;
    } else {
      found_ = false;
    }
  }

  bool step(double now, double* reported_tilt) {
    if (found_ && now - last_rx_ > 0.5) found_ = false;
    while (now - last_track_ >= 0.04) {
      last_track_ += 0.04;
      if (found_) {
        const int err_y = static_cast<int>(target_y_) - 240;
        if (err_y > 12 || err_y < -12) {
          double delta = -0.012 * static_cast<double>(err_y);
          delta = std::max(-1.5, std::min(1.5, delta));
          tilt_ = std::max(-75.0, std::min(75.0, tilt_ + delta));
        }
      }
    }
    if (now - last_stat_ >= 0.36) {
      last_stat_ = now;
      *reported_tilt = std::round(tilt_ * 10.0) / 10.0;
      return true;
    }
    return false;
  }

  double actualTilt() const { return tilt_; }

 private:
  double tilt_{0.0};
  bool found_{false};
  uint16_t target_x_{320};
  uint16_t target_y_{240};
  double last_rx_{-1.0};
  double last_track_{0.0};
  double last_stat_{-0.36};
};

}  // namespace

TEST(GimbalTiltController, InitializesDownwardTenDegWithoutOvershoot) {
  GimbalTiltConfig cfg;
  cfg.min_tilt_deg = -25.0;
  cfg.max_tilt_deg = 0.0;
  cfg.default_tilt_deg = -10.0;
  GimbalTiltController ctrl(cfg);
  ctrl.requestLevel();

  SimulatedMcuGimbal mcu(0.0);
  double reported_tilt = 0.0;
  double min_seen = 0.0;
  double max_seen = 0.0;

  for (int tick = 0; tick <= 80; ++tick) {
    const double now = tick * 0.05;
    if (mcu.step(now, &reported_tilt)) ctrl.onStatus(reported_tilt);
    const auto cmd = ctrl.compute(now, true, true, reported_tilt,
                                  false, 320.0, 240.0, 0.0, false);
    if (cmd.found) EXPECT_EQ(cmd.x, 320U);
    mcu.receiveTarget(now, cmd);
    min_seen = std::min(min_seen, mcu.actualTilt());
    max_seen = std::max(max_seen, mcu.actualTilt());
  }

  EXPECT_FALSE(ctrl.leveling());
  EXPECT_NEAR(mcu.actualTilt(), -10.0, 1.5);
  EXPECT_LE(max_seen, 0.1);
  EXPECT_GE(min_seen, -12.0);
}

TEST(GimbalTiltController, ClampsTrackingBetweenNeg25AndZeroAndFiltersHighClutter) {
  GimbalTiltConfig cfg;
  cfg.min_tilt_deg = -25.0;
  cfg.max_tilt_deg = 0.0;
  cfg.default_tilt_deg = -10.0;
  cfg.min_target_down_deg = -4.0;
  GimbalTiltController ctrl(cfg);

  // High blue clutter above horizon (target_down_deg = -8 deg) is rejected.
  const auto clutter_cmd = ctrl.compute(1.0, true, true, -10.0,
                                        true, 180.0, 80.0, -8.0, true);
  EXPECT_FALSE(clutter_cmd.found);

  // Valid target near horizon requesting upward motion must never pitch above 0 deg.
  SimulatedMcuGimbal mcu(-10.0);
  double reported_tilt = -10.0;
  double max_tilt = -10.0;
  for (int tick = 0; tick <= 120; ++tick) {
    const double now = tick * 0.05;
    if (mcu.step(now, &reported_tilt)) ctrl.onStatus(reported_tilt);
    const auto cmd = ctrl.compute(now, true, true, reported_tilt,
                                  true, 260.0, 40.0, -1.0, true);
    ASSERT_TRUE(cmd.found);
    EXPECT_EQ(cmd.x, 260U);
    mcu.receiveTarget(now, cmd);
    max_tilt = std::max(max_tilt, mcu.actualTilt());
  }
  EXPECT_LE(max_tilt, 0.05);

  // Valid low target requesting downward motion must never pitch below -25 deg.
  double min_tilt = mcu.actualTilt();
  for (int tick = 121; tick <= 260; ++tick) {
    const double now = tick * 0.05;
    if (mcu.step(now, &reported_tilt)) ctrl.onStatus(reported_tilt);
    const auto cmd = ctrl.compute(now, true, true, reported_tilt,
                                  true, 410.0, 460.0, 20.0, true);
    ASSERT_TRUE(cmd.found);
    EXPECT_EQ(cmd.x, 410U);
    mcu.receiveTarget(now, cmd);
    min_tilt = std::min(min_tilt, mcu.actualTilt());
  }
  EXPECT_GE(min_tilt, -25.05);

  // After target loss > 0.8 s from deep downward tilt, controller returns to -10 deg
  // and releases found=false so MCU pan search resumes.
  for (int tick = 261; tick <= 380; ++tick) {
    const double now = tick * 0.05;
    if (mcu.step(now, &reported_tilt)) ctrl.onStatus(reported_tilt);
    const auto cmd = ctrl.compute(now, true, true, reported_tilt,
                                  false, 320.0, 240.0, 0.0, true);
    mcu.receiveTarget(now, cmd);
  }
  EXPECT_NEAR(mcu.actualTilt(), -10.0, 1.5);
  const auto released = ctrl.compute(20.0, true, true, reported_tilt,
                                     false, 320.0, 240.0, 0.0, true);
  EXPECT_FALSE(released.found);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
